#!/usr/bin/env python3
"""SITL end-to-end test driving the custom link as the ONLY simulator link.

Brings up Betaflight SITL with the custom companion-computer link on
UART3/tcp:5763 and replaces the whole Gazebo-style UDP bridge (FDM 9003 /
RC 9004 / motors 9002) with the simulator extension of the custom protocol:

  * 0x21 HOST_IMU   - injected gyro/acc/attitude (virtual sensor feed)
  * 0x22 HOST_ENV   - injected baro pressure and GPS solution
  * 0x23 HOST_RC    - injected RC channels (OFFBOARD switch on AUX2)
  * 0x13 FC_MOTORS  - mixer motor outputs streamed back

Checks, in order:

  1. link bring-up        : telemetry flows before any injection is sent
  2. telemetry rates      : 0x10 ~200 Hz, 0x11 ~100 Hz, 0x12 ~10 Hz
  3. IMU injection        : level acc echoes on 0x10 (z ~ +1 g)
  4. attitude             : estimator converges to ~0 on level feed,
                            then to +10 deg roll for a tilted acc feed
  5. gyro passthrough     : injected rate appears on the 0x10 gyro stream
  6. env injection        : baro pressure and GPS echo on 0x11/0x12
  7. RC injection         : AUX2 (1800 us) echoes on the 0x11 rc array
  8. motors stream        : 0x13 frames arrive, count = 4
  9. host arm + offboard  : 0x20 arm=1 arms, OFFBOARD engages, motors
                            track the commanded throttle
 10. watchdog             : stopping the 0x20 stream drops OFFBOARD and
                            disarms (host arm is level-based)
 11. re-arm + disarm      : arm again, then disarm via arm=0

Usage:
  python3 src/test/sitl/sitl_custom_link_sim_test.py \
      [--binary obj/betaflight_2026.6.1_SITL] [--workdir /tmp/bf_cl_sim]

Notes:
  * A stale SITL/feeder process will poison the run (ports + eeprom): the
    script pkills first.
  * Injection must not start before telemetry bytes flow - the SITL init
    races with early external input (same as the UDP bridge).
"""

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

LINK_TCP_PORT = 5763
SYNC = b"\xeb\x90"

BOX_ARM = 0        # permanent id
BOX_OFFBOARD = 58  # permanent id

OFFBOARD_MODE_BIT = 1 << 13
STATUS_ARMED = 0x0001

MSG_FC_FAST = 0x10
MSG_FC_MEDIUM = 0x11
MSG_FC_SLOW = 0x12
MSG_FC_MOTORS = 0x13
MSG_HOST_CONTROL = 0x20
MSG_HOST_IMU = 0x21
MSG_HOST_ENV = 0x22
MSG_HOST_RC = 0x23

MSG_NAMES = {
    MSG_FC_FAST: "0x10", MSG_FC_MEDIUM: "0x11", MSG_FC_SLOW: "0x12",
    MSG_FC_MOTORS: "0x13",
}


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_id, seq, payload):
    hdr = bytes([msg_id, len(payload), seq & 0xFF])
    crc = crc16_xmodem(hdr + payload)
    return SYNC + hdr + payload + struct.pack("<H", crc)


class Parser:
    """Byte-stream parser mirroring clParserProcessByte."""

    def __init__(self):
        self.buf = b""
        self.frames = []

    def feed(self, data):
        self.buf += data
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep the last byte: it may be a preamble
                self.buf = self.buf[-1:]
                return
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                return
            msg_id, length, seq = self.buf[2], self.buf[3], self.buf[4]
            if length == 0 or length > 64:
                self.buf = self.buf[2:]
                continue
            if len(self.buf) < 7 + length:
                return
            payload = self.buf[5:5 + length]
            crc = struct.unpack("<H", self.buf[5 + length:7 + length])[0]
            self.buf = self.buf[7 + length:]
            if crc != crc16_xmodem(bytes([msg_id, length, seq]) + payload):
                continue  # bad frame, resync happens via the buffer search
            self.frames.append((msg_id, seq, payload, time.monotonic()))


class Feed:
    """Single-socket feeder: IMU 250 Hz, ENV 10 Hz, RC 50 Hz, control 50 Hz."""

    def __init__(self, sock, send_lock):
        self.sock = sock
        self.send_lock = send_lock
        self.running = True
        self.seq = 0
        self.rc_channels = [1500, 1500, 1000, 1500] + [1000] * 12  # AETR + AUX
        self.gyro_dps = (0.0, 0.0, 0.0)
        self.acc_mg = (0.0, 0.0, 1000.0)
        self.quat = (1.0, 0.0, 0.0, 0.0)
        self.pressure_pa = 101325
        self.lat_e7, self.lon_e7 = -23000000, 134000000
        self.control = None  # dict(throttle=, rates=, arm=) or None
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _send(self, msg_id, payload):
        with self.send_lock:
            try:
                self.sock.sendall(encode(msg_id, self.seq, payload))
            except OSError:
                self.running = False
        self.seq = (self.seq + 1) & 0xFF

    def _loop(self):
        t0 = time.monotonic()
        last_imu = last_env = last_rc = last_ctrl = 0.0
        while self.running:
            now = time.monotonic() - t0
            ts_us = int(now * 1e6) & 0xFFFFFFFF

            if now - last_imu >= 0.004:  # 250 Hz
                last_imu = now
                gx, gy, gz = (int(round(v * 10)) for v in self.gyro_dps)
                ax, ay, az = (int(round(v)) for v in self.acc_mg)
                q = [max(-32768, min(32767, int(round(v * 16384))))
                     for v in self.quat]
                self._send(MSG_HOST_IMU, struct.pack(
                    "<I3h3h4h", ts_us, gx, gy, gz, ax, ay, az, *q))

            if now - last_env >= 0.1:  # 10 Hz
                last_env = now
                self._send(MSG_HOST_ENV, struct.pack(
                    "<IIhiii3hHHBB",
                    ts_us, self.pressure_pa, 2500,
                    self.lat_e7, self.lon_e7, 123456,
                    0, 0, 0, 0, 0, 1, 12))

            if now - last_rc >= 0.02:  # 50 Hz
                last_rc = now
                self._send(MSG_HOST_RC, struct.pack(
                    "<I16H", ts_us, *self.rc_channels))

            if self.control is not None and now - last_ctrl >= 0.02:  # 50 Hz
                last_ctrl = now
                c = self.control
                rx, ry, rz = (int(round(v * 10)) for v in c["rates"])
                self._send(MSG_HOST_CONTROL, struct.pack(
                    "<QHBBH3h", int(time.monotonic() * 1e6) & 0xFFFFFFFFFFFFFFFF,
                    0, c["arm"], 2, int(c["throttle"]), rx, ry, rz))

            time.sleep(0.001)

    def stop(self):
        self.running = False
        self.t.join(timeout=2.0)


class Telemetry:
    """Reader thread capturing downlink frames."""

    def __init__(self, sock):
        self.sock = sock
        self.parser = Parser()
        self.running = True
        self.rx_counts = {}
        self.last = {}
        self.first_seen = {}
        self.motor_values = None
        self.motor_stamp = 0.0
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        while self.running:
            try:
                data = self.sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            self.parser.feed(data)
            for msg_id, seq, payload, stamp in self.parser.frames:
                self.rx_counts[msg_id] = self.rx_counts.get(msg_id, 0) + 1
                self.first_seen.setdefault(msg_id, stamp)
                self._decode(msg_id, payload, stamp)
            self.parser.frames = []

    def _decode(self, msg_id, p, stamp):
        try:
            if msg_id == MSG_FC_FAST:
                ts, gx, gy, gz, ax, ay, az, r, pi, yaw = struct.unpack(
                    "<I3h3h3h", p[:22])
                self.last["gyro"] = (gx, gy, gz)
                self.last["acc"] = (ax, ay, az)
                self.last["attitude"] = (r, pi, yaw)
            elif msg_id == MSG_FC_MEDIUM:
                ts, pa, alt, temp = struct.unpack("<IIih", p[:14])
                rc = struct.unpack("<16H", p[14:46])
                self.last["baro_pa"] = pa
                self.last["rc"] = rc
            elif msg_id == MSG_FC_SLOW:
                ts, vbat, cur, mah, fix, sats, lat, lon, alt, spd, crs, \
                    mode_flags, status = struct.unpack("<IHihBBiiiHHIH", p[:36])
                self.last["gps"] = (lat, lon, alt, fix, sats)
                self.last["mode_flags"] = mode_flags
                self.last["status"] = status
            elif msg_id == MSG_FC_MOTORS:
                count = p[0]
                motors = struct.unpack("<8H", p[1:17])[:count]
                self.motor_values = motors
                self.motor_stamp = time.time()
        except struct.error:
            pass

    def stop(self):
        self.running = False
        self.t.join(timeout=2.0)

    # convenience accessors
    def gyro(self):
        return self.last.get("gyro")

    def attitude_deg(self):
        att = self.last.get("attitude")
        return None if att is None else tuple(v * 0.1 for v in att)

    def armed(self):
        s = self.last.get("status")
        return bool(s is not None and (s & STATUS_ARMED))

    def offboard(self):
        f = self.last.get("mode_flags")
        return bool(f is not None and (f & OFFBOARD_MODE_BIT))


def wait_for(desc, predicate, timeout=15.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print(f"  [ok] {desc}")
            return True
        time.sleep(interval)
    print(f"  [FAIL] {desc} (timeout {timeout}s)")
    return False


def approx(value, target, tol):
    return value is not None and abs(value - target) <= tol


def main():
    ap = argparse.ArgumentParser()
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    default_bin = os.path.join(repo_root, "obj", "betaflight_SITL.elf")
    ap.add_argument("--binary", default=os.path.normpath(default_bin),
                    help="path to the betaflight SITL executable")
    ap.add_argument("--workdir", default="/tmp/bf_custom_link_sim")
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        # the build also copies to obj/betaflight_<version>_SITL
        alt = os.path.join(os.path.dirname(binary), "betaflight_2026.6.1_SITL")
        if os.path.exists(alt):
            binary = alt
        else:
            print(f"binary not found: {args.binary} (build with: make TARGET=SITL)")
            return 1

    results = {}

    # 清理可能残留的旧进程：残留 SITL 会抢占 tcp/5763 且旧 eeprom 会串味。
    # 行首锚定匹配 SITL 二进制路径，避免匹配到本脚本的 --binary 参数而自杀。
    subprocess.run(["pkill", "-f", f"^{binary}"], check=False)
    time.sleep(1.0)

    os.makedirs(args.workdir, exist_ok=True)
    eeprom = os.path.join(args.workdir, "eeprom.bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    # ---- provision: ARM on AUX1, OFFBOARD on AUX2 ----
    cfg = os.path.join(args.workdir, "config.txt")
    with open(cfg, "w") as f:
        f.write("\n".join([
            "set small_angle = 180",
            f"aux 0 {BOX_ARM} 0 1700 2100 0 0",
            f"aux 1 {BOX_OFFBOARD} 1 1700 2100 0 0",
        ]) + "\n")
    res = subprocess.run([binary, "--config", cfg], cwd=args.workdir,
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0 or not os.path.exists(eeprom):
        print("provisioning failed:", res.stdout[-500:], res.stderr[-500:])
        return 1

    # ---- start SITL ----
    sitl_log = open(os.path.join(args.workdir, "sitl.log"), "w")
    sitl = subprocess.Popen(["stdbuf", "-oL", "-eL", binary], cwd=args.workdir,
                            stdout=sitl_log, stderr=sitl_log,
                            preexec_fn=os.setsid)

    sock = None
    feed = tel = None
    try:
        # ---- connect + wait for telemetry (init race: inject only after) ----
        def connect_and_wait(timeout=30.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    s = socket.create_connection(("127.0.0.1", LINK_TCP_PORT),
                                                 timeout=0.5)
                    s.settimeout(2.0)
                    try:
                        data = s.recv(64)
                    except socket.timeout:
                        data = b""
                    if SYNC in data:
                        return s, data
                    s.close()
                except OSError:
                    pass
                time.sleep(0.3)
            return None, b""

        print("== 1. link bring-up ==")
        sock, first = connect_and_wait()
        if sock is None:
            print("  [FAIL] SITL did not stream custom link telemetry on tcp/5763")
            return 1
        print("  [ok] telemetry streaming (custom link only, no UDP bridge)")
        results["link_up"] = True

        sock.settimeout(None)
        send_lock = threading.Lock()
        tel = Telemetry(sock)
        tel.parser.feed(first)
        time.sleep(0.5)

        feed = Feed(sock, send_lock)
        feed.rc_channels[5] = 1800  # AUX2 high: BOXOFFBOARD switch on

        # ---- 2. telemetry rates ----
        print("== 2. telemetry rates (3 s window) ==")
        c0 = dict(tel.rx_counts)
        time.sleep(3.0)
        def hz(mid):
            return (tel.rx_counts.get(mid, 0) - c0.get(mid, 0)) / 3.0
        f10, f11, f12, f13 = hz(MSG_FC_FAST), hz(MSG_FC_MEDIUM), hz(MSG_FC_SLOW), hz(MSG_FC_MOTORS)
        print(f"  0x10 {f10:.1f} Hz, 0x11 {f11:.1f} Hz, 0x12 {f12:.1f} Hz, 0x13 {f13:.1f} Hz")
        results["rates"] = 160 <= f10 <= 240 and 80 <= f11 <= 120 and 8 <= f12 <= 12
        results["motors_stream_rate"] = 160 <= f13 <= 240

        # ---- 3. IMU injection: level acc echo ----
        print("== 3. IMU injection (level) ==")
        results["acc_level"] = wait_for(
            "0x10 acc z ~ +1 g",
            lambda: approx((tel.last.get("acc") or (0, 0, 0))[2], 1000, 60))
        att = tel.attitude_deg()
        print(f"  attitude now: {att}")

        # ---- 4. attitude estimator convergence ----
        print("== 4. attitude convergence ==")
        results["attitude_level"] = wait_for(
            "level attitude |roll|,|pitch| < 5 deg",
            lambda: (tel.attitude_deg() is not None
                     and abs(tel.attitude_deg()[0]) < 5
                     and abs(tel.attitude_deg()[1]) < 5),
            timeout=20)
        # tilt 10 deg right: specific force leans toward the raised left side
        feed.acc_mg = (0.0, 173.6, 984.8)
        results["attitude_tilt"] = wait_for(
            "roll ~ +10 deg on tilted acc",
            lambda: approx((tel.attitude_deg() or (0,))[0], 10.0, 4.0),
            timeout=25)
        feed.acc_mg = (0.0, 0.0, 1000.0)
        wait_for("level again", lambda: abs((tel.attitude_deg() or (99,))[0]) < 5,
                 timeout=25)

        # ---- 5. gyro passthrough ----
        print("== 5. gyro passthrough ==")
        feed.gyro_dps = (50.0, 0.0, 0.0)
        results["gyro_passthrough"] = wait_for(
            "0x10 gyro roll ~ +50 deg/s",
            lambda: approx((tel.gyro() or (0,))[0], 500, 150),
            timeout=5)
        feed.gyro_dps = (0.0, 0.0, 0.0)

        # ---- 6. environment injection ----
        print("== 6. env injection ==")
        results["baro"] = wait_for(
            "0x11 baro ~ 101325 Pa",
            lambda: approx(tel.last.get("baro_pa"), 101325, 2000))
        gps = tel.last.get("gps")
        print(f"  gps now: {gps}")
        results["gps"] = wait_for(
            "0x12 gps matches injected lat/lon",
            lambda: (tel.last.get("gps") is not None
                     and abs(tel.last["gps"][0] - feed.lat_e7) <= 2
                     and abs(tel.last["gps"][1] - feed.lon_e7) <= 2))

        # ---- 7. RC injection echo ----
        print("== 7. RC injection ==")
        results["rc_echo"] = wait_for(
            "0x11 rc[5] == 1800 (OFFBOARD switch)",
            lambda: (tel.last.get("rc") is not None
                     and tel.last["rc"][5] == 1800))

        # ---- 8. motors frame ----
        print("== 8. motors stream ==")
        m = tel.motor_values
        print(f"  motors (disarmed): {m}")
        results["motors_count"] = m is not None and len(m) == 4

        # ---- 9. host arm + offboard ----
        print("== 9. host arm + offboard ==")
        feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=0)
        time.sleep(0.5)
        feed.control["arm"] = 1
        results["host_arm"] = wait_for("armed via 0x20", lambda: tel.armed(),
                                       timeout=12)
        results["offboard_mode"] = wait_for("OFFBOARD mode flag",
                                            lambda: tel.offboard(), timeout=5)
        feed.control["throttle"] = 1400
        results["motor_response"] = wait_for(
            "motors track throttle ~1400 us",
            lambda: (tel.motor_values is not None and len(tel.motor_values) == 4
                     and all(abs(v - 1400) <= 120 for v in tel.motor_values)),
            timeout=5)
        print(f"  motors: {tel.motor_values}")

        # ---- 10. watchdog ----
        print("== 10. watchdog (stop control stream) ==")
        feed.control = None
        results["watchdog_offboard"] = wait_for(
            "OFFBOARD dropped", lambda: not tel.offboard(), timeout=3)
        results["watchdog_disarm"] = wait_for(
            "disarmed (host arm is level-based)", lambda: not tel.armed(),
            timeout=5)

        # ---- 11. re-arm + explicit disarm ----
        print("== 11. re-arm + host disarm ==")
        feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1)
        re_ok = wait_for("armed again", lambda: tel.armed(), timeout=12)
        feed.control["arm"] = 0
        results["host_disarm"] = re_ok and wait_for(
            "disarmed via arm=0", lambda: not tel.armed(), timeout=5)
    finally:
        if feed:
            feed.stop()
        if tel:
            tel.stop()
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        try:
            os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
        except OSError:
            pass
        try:
            sitl.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
        sitl_log.close()

    print("\n===== summary =====")
    all_ok = True
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
        all_ok &= bool(v)
    print("RESULT:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
