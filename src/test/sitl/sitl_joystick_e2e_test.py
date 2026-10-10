#!/usr/bin/env python3
"""End-to-end test: the joystick bridge mapping against a real SITL binary.

Provisions the joystick aux table (same lines as sitl_joystick.py documents),
boots SITL with the harness physics feed, drives sitl_joystick.JoystickState
with synthetic js0 events over real UDP :9004 and asserts through MSP
(tcp :5761):

  phase A - mapping and stick boxes (disarmed)
    * MSP_RC readback equals the mapped channels (wire AETR -> rcData RPYT)
    * the 6-pos dial activates ANGLE / HORIZON (and leaves them off elsewhere)
    * the Trigger button drives AUX4
  phase B - arming and GPS flight modes
    * the ARM switch high arms the FC
    * with throttle raised, dial ALT HOLD / POS HOLD positions engage the
      corresponding flight modes (they require ARMED + wasThrottleRaised,
      core.c), and switch low disarms
  phase C - custom-link coexistence and OFFBOARD (firmware: rxUdpBridgeRcFresh)
    * with 0x23 HOST_RC + 0x20 HOST_CONTROL streams flowing on tcp :5763,
      the UDP bridge keeps exclusive ownership of the channels
    * the 3-pos toggle high engages OFFBOARD (fresh control stream + armed)
    * stopping the UDP stream hands RC back to 0x23 within ~1 s

Run:
    python3 src/test/sitl/sitl_joystick_e2e_test.py \
        [--binary obj/main/betaflight_SITL.elf] [--workdir /tmp/bf_joystick_e2e]

A stale SITL process will poison the run (ports + eeprom): the script pkills
first (anchored to the binary path so it cannot match itself) and aborts
early if another RC feeder is still streaming on UDP 9004. SITL launch and
the MSP connection reuse the harness's Sitl class (port-free wait + retry).
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sitl_joystick as sj
from sitl_harness import FdmFeed, MotorFeed, RC_PORT, Sitl, wait_for  # noqa: E402

TCP_CUSTOM_LINK = 5763
CL_SYNC = b"\xeb\x90"
MSG_HOST_CONTROL = 0x20
MSG_HOST_RC = 0x23

# permanent box ids (msp_box.c) as provisioned below
BOX_ARM, BOX_ANGLE, BOX_HORIZON, BOX_ALTHOLD = 0, 1, 2, 3
BOX_POSHOLD, BOX_OFFBOARD = 11, 58

MSP_STATUS = 101
MSP_RC = 105

CONFIG_LINES = [
    "feature GPS",
    "set gps_provider = VIRTUAL",
    "set failsafe_procedure = AUTO-LAND",
    "set failsafe_delay = 10",
    "set small_angle = 180",
    "set trust_mag = ON",
] + list(sj.CLI_CONFIG_LINES)

TX16S_MAPPING = {
    "roll": 0, "pitch": 1, "yaw": 2, "throttle": 3,
    "mode6": 4, "switch3": 5, "arm": 0, "trigger": 1,
    "invert": set(), "trigger_mode": "hold",
}

THROTTLE_RAISED_US = 1600   # satisfies wasThrottleRaised and clears the ground
RAW_RAISED = int(round((THROTTLE_RAISED_US - 1000) / 1000.0 * 65534)) - 32767


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def cl_encode(msg_id, seq, payload):
    hdr = bytes([msg_id, len(payload), seq & 0xFF])
    return CL_SYNC + hdr + payload + struct.pack("<H", crc16_xmodem(hdr + payload))


def log(msg):
    print("[e2e] {}".format(msg), flush=True)


def assert_no_stale_rc_feeder():
    """Another bridge still streaming on 9004 would fight this test's sender
    through the firmware's last-writer-wins latch; abort with a clear error
    instead of proceeding into confusing arbitration timeouts."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    try:
        s.bind(("127.0.0.1", RC_PORT))
        try:
            if s.recv(64):
                raise RuntimeError(
                    "a stale RC feeder is still streaming on UDP 9004; stop it "
                    "before running this test")
        except socket.timeout:
            pass
    finally:
        s.close()


class CustomLinkHostStream(threading.Thread):
    """0x23 HOST_RC + 0x20 HOST_CONTROL sender standing in for AirSim."""

    def __init__(self, sock, rc_channels, control):
        super().__init__(daemon=True)
        self.sock = sock
        self.rc_channels = rc_channels
        self.control = control  # dict(throttle=, rates=(x3), arm=)
        self.running = True
        self.seq = 0

    def _send(self, msg_id, payload):
        self.sock.sendall(cl_encode(msg_id, self.seq, payload))
        self.seq = (self.seq + 1) & 0xFF

    def run(self):
        next_t = time.monotonic()
        while self.running:
            try:
                ts_us = int(time.monotonic() * 1e6) & 0xFFFFFFFF
                self._send(MSG_HOST_RC, struct.pack("<I16H", ts_us, *self.rc_channels))
                ts64 = int(time.monotonic() * 1e6)
                c = self.control
                self._send(MSG_HOST_CONTROL, struct.pack(
                    "<QHBBH3h", ts64, 0, c["arm"], 2, c["throttle"],
                    c["rates"][0], c["rates"][1], c["rates"][2]))
            except OSError:
                break
            next_t += 0.02
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def stop(self):
        self.running = False
        self.join(timeout=2.0)


def ev(axis, number, value):
    return (0, value, sj.JS_EVENT_AXIS if axis else sj.JS_EVENT_BUTTON, number)


def feed_init(state):
    """Power-on state: sticks centred, throttle idle, dial/toggle at minimum."""
    for number in (0, 1, 2):
        state.feed(ev(True, number, 0))
    state.feed(ev(True, 3, -32767))
    state.feed(ev(True, 4, -32767))
    state.feed(ev(True, 5, -32767))


class Probe:
    """Transient-tolerant MSP reads: a stalled reply returns a value that can
    never satisfy a wait (so wait_for retries within its own budget) instead
    of aborting the whole test with a TimeoutError."""

    def __init__(self, sitl):
        self.sitl = sitl

    def _request(self, cmd):
        try:
            return self.sitl.msp.request(cmd)
        except TimeoutError:
            return None

    def rc(self):
        """rcData in logical order: ROLL, PITCH, YAW, THROTTLE, AUX1...

        The wire packet is AETR (slot2 = throttle, slot3 = yaw) while rcData
        stores YAW=2/THROTTLE=3, so readback positions 2 and 3 are swapped
        relative to the packet.
        """
        payload = self._request(MSP_RC)
        if payload is None:
            return (0,) * 16
        return struct.unpack("<{}H".format(len(payload) // 2), payload)

    def boxes(self):
        payload = self._request(MSP_STATUS)
        if payload is None:
            return set()
        mode_flags = struct.unpack_from("<I", payload, 6)[0]
        boxids = self.sitl.boxids
        return {boxids[i] for i in range(min(32, len(boxids))) if mode_flags & (1 << i)}

    def arming_flags(self):
        p = self._request(MSP_STATUS)
        if p is None:
            return 0xFFFFFFFF
        return struct.unpack_from("<I", p, 16 + p[15] + 1)[0]


def set_arm_switch(state, enabled):
    state.feed(ev(False, 0, 1 if enabled else 0))


def arm_with_retry(state, probe, description):
    """A transient arming-disable coinciding with the switch going high
    latches ARM_SWITCH (core.c) until the switch is cycled; retry."""
    for attempt in range(3):
        set_arm_switch(state, True)
        try:
            wait_for(description, lambda: BOX_ARM in probe.boxes(), timeout=8)
            return
        except AssertionError:
            if attempt == 2:
                raise
            log("{}: arm latched ARM_SWITCH; cycling the switch".format(description))
            set_arm_switch(state, False)
            time.sleep(1.0)


def raise_throttle(state):
    state.feed(ev(True, 3, RAW_RAISED))
    time.sleep(1.5)   # latch wasThrottleRaised and clear the ground
    state.feed(ev(True, 3, -32767))


def main():
    ap = argparse.ArgumentParser()
    default_binary = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))), "obj", "main", "betaflight_SITL.elf")
    ap.add_argument("--binary", default=default_binary)
    ap.add_argument("--workdir", default="/tmp/bf_joystick_e2e")
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)

    # stale SITL/feeder processes poison the run (ports + eeprom)
    subprocess.run(["pkill", "-f", "^{}".format(binary)], check=False)
    time.sleep(1.0)
    os.makedirs(args.workdir, exist_ok=True)
    assert_no_stale_rc_feeder()

    sitl = Sitl(binary, args.workdir)
    cl_sock = None
    cl_stream = None
    sender = None
    try:
        sitl.provision(CONFIG_LINES)
        sitl.start()  # port-free wait + 3 launch attempts + MSP connection
        probe = Probe(sitl)

        # the joystick bridge under test: synthetic events, real UDP stream
        state = sj.JoystickState(dict(TX16S_MAPPING))
        feed_init(state)
        sender = sj.RcSender(state.channels, ("127.0.0.1", RC_PORT))

        motors = MotorFeed()
        fdm = FdmFeed(motors)
        motors.start()
        fdm.start()
        sender.start()

        wait_for("GPS fix + RX recovery (arming flags clear)",
                 lambda: probe.arming_flags() == 0, timeout=40)

        # ---- phase A: mapping and stick boxes (disarmed) ------------------
        # readback is rcData order (ROLL, PITCH, YAW, THROTTLE): the wire's
        # slot2 (throttle) appears at index 3, slot3 (yaw) at index 2
        wait_for("MSP_RC readback: centred sticks, idle throttle",
                 lambda: probe.rc()[:8] == (1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000),
                 timeout=5)
        log("A: initial channels ok")

        state.feed(ev(True, 0, 32767))    # roll right
        state.feed(ev(True, 2, -32767))   # yaw left
        wait_for("roll right / yaw left reach rcData",
                 lambda: (probe.rc()[0], probe.rc()[2]) == (2000, 1000), timeout=5)
        state.feed(ev(True, 0, 0))
        state.feed(ev(True, 2, 0))

        for pos, raw, want_box, name in (
                (1, -32767, None, "ACRO"),
                (2, -19660, BOX_ANGLE, "ANGLE"),
                (3, -6554, BOX_HORIZON, "HORIZON"),
                (6, 32767, None, "GPSRESCUE (disarmed)"),
                (1, -32767, None, "back to ACRO")):
            state.feed(ev(True, 4, raw))
            want_us = sj.MODE6_VALUES_US[pos - 1]
            # the readback wait (not the mode wait) synchronises with the
            # 50 Hz sender: mode-off positions cannot order the transition
            wait_for("dial pos {} ({}) reads {} us on AUX1".format(pos, name, want_us),
                     lambda w=want_us: probe.rc()[4] == w, timeout=5)
            if want_box is None:
                wait_for("dial pos {} ({}) leaves flight modes off".format(pos, name),
                         lambda: not ({BOX_ANGLE, BOX_HORIZON, BOX_ALTHOLD, BOX_POSHOLD}
                                      & probe.boxes()), timeout=5)
            else:
                wait_for("dial pos {} activates {}".format(pos, name),
                         lambda want=want_box: want in probe.boxes(), timeout=5)
        log("A: dial positions + ANGLE/HORIZON ok")

        state.feed(ev(False, 1, 1))       # trigger held
        wait_for("trigger drives AUX4 high",
                 lambda: probe.rc()[7] == 2000, timeout=5)
        state.feed(ev(False, 1, 0))
        wait_for("trigger release drops AUX4",
                 lambda: probe.rc()[7] == 1000, timeout=5)
        log("A: trigger AUX4 ok")

        # ---- phase B: arming and GPS flight modes -------------------------
        # recalibrate like boot_and_engage: the boot-time acc calibration can
        # capture offsets from a not-yet-settled FDM feed
        sitl.acc_calibrate()
        time.sleep(2.0)
        wait_for("recalibration complete (flags clear)",
                 lambda: probe.arming_flags() == 0, timeout=20)

        arm_with_retry(state, probe, "ARM switch high arms the FC")
        wait_for("ARM switch reads 2000 us on AUX3",
                 lambda: probe.rc()[6] == 2000, timeout=5)
        log("B: armed via switch")

        # ALT HOLD / POS HOLD engage only once armed with throttle raised;
        # position 4 holds altitude, position 5 adds horizontal position hold
        raise_throttle(state)
        state.feed(ev(True, 4, 6553))     # dial position 4
        wait_for("dial pos 4 engages ALT HOLD",
                 lambda: BOX_ALTHOLD in probe.boxes(), timeout=8)
        state.feed(ev(True, 4, 19660))    # dial position 5
        wait_for("dial pos 5 engages POS HOLD + ALT HOLD",
                 lambda: {BOX_POSHOLD, BOX_ALTHOLD} <= probe.boxes(), timeout=8)
        log("B: ALT HOLD / POS HOLD engaged")
        state.feed(ev(True, 4, -32767))   # dial back to ACRO
        wait_for("dial back to ACRO drops the GPS modes",
                 lambda: not ({BOX_ALTHOLD, BOX_POSHOLD} & probe.boxes()), timeout=8)
        raise_throttle(state)             # re-clear the ground after mode tests

        set_arm_switch(state, False)
        wait_for("ARM switch low disarms",
                 lambda: BOX_ARM not in probe.boxes(), timeout=8)
        log("B: disarmed via switch")

        # ---- phase C: custom-link coexistence and OFFBOARD ----------------
        deadline = time.monotonic() + 15.0
        while True:
            try:
                cl_sock = socket.create_connection(("127.0.0.1", TCP_CUSTOM_LINK), 0.5)
                cl_sock.settimeout(2.0)
                data = cl_sock.recv(64)
                assert CL_SYNC in data, "custom link sent no sync frame"
                break
            except (OSError, AssertionError):
                if cl_sock is not None:
                    cl_sock.close()
                    cl_sock = None
                assert time.monotonic() < deadline, "custom link never came up"
                time.sleep(0.3)
        cl_sock.settimeout(None)
        # distinctive stick values that would be obvious in readback; the
        # control stream hovers so an OFFBOARD engage does not just fall out
        # of the sky mid-assert
        host_rc = [1234, 1235, 1236, 1237] + [1250] * 12
        control = {"throttle": 1500, "rates": (0, 0, 0), "arm": 0}
        cl_stream = CustomLinkHostStream(cl_sock, host_rc, control)
        cl_stream.start()

        samples = []
        for _ in range(6):
            try:
                samples.append(probe.rc()[0])
            except TimeoutError:
                pass  # transient MSP stall: skip the sample
            time.sleep(0.1)
        # steady-state ownership: the bridge must hold roll through the window
        # (one scheduler-flap sample tolerated) and own it again at the end
        assert samples and samples.count(1500) >= len(samples) - 1 and samples[-1] == 1500, \
            "UDP bridge lost authority while streaming: roll readback {}".format(samples)
        log("C: UDP bridge owns channels against the 0x23 stream")

        # OFFBOARD engages with: armed + AUX2 high (joystick) + fresh control
        arm_with_retry(state, probe,
                       "ARM switch arms again (0x20 arm=0 does not fight the switch)")
        raise_throttle(state)
        state.feed(ev(True, 5, 32767))    # 3-pos high
        wait_for("3-pos high engages OFFBOARD",
                 lambda: BOX_OFFBOARD in probe.boxes(), timeout=8)
        state.feed(ev(True, 5, -32767))   # 3-pos low
        wait_for("3-pos low disengages OFFBOARD",
                 lambda: BOX_OFFBOARD not in probe.boxes(), timeout=8)
        log("C: OFFBOARD via 3-pos toggle ok")
        set_arm_switch(state, False)
        wait_for("disarm before handover",
                 lambda: BOX_ARM not in probe.boxes(), timeout=8)

        # arbitration handover: stop the UDP bridge, 0x23 must take over
        sender.stop()
        wait_for("0x23 takes over after the bridge stops",
                 lambda: probe.rc()[0] == 1234, timeout=3)
        log("C: handover complete (arming_flags=0x{:08x})".format(probe.arming_flags()))

        cl_stream.stop()
        cl_stream = None
        cl_sock.close()
        cl_sock = None

        log("PASS")
        return 0
    finally:
        if cl_stream is not None:
            cl_stream.stop()
        if cl_sock is not None:
            cl_sock.close()
        if sender is not None:
            sender.stop()
        sitl.stop()


if __name__ == "__main__":
    sys.exit(main())
