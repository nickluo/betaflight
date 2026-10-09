#!/usr/bin/env python3
"""Betaflight SITL joystick bridge: /dev/input/js* -> RC over UDP port 9004.

Maps a Linux joystick onto the SITL RC input protocol. The wire format is the
firmware's rc_packet (target.h): struct '<d16H' - a double timestamp in
seconds followed by 16 little-endian uint16 channel values in microseconds.
Channel order matches the default rcmap AETR1234: ch0 Roll, ch1 Pitch,
ch2 Throttle, ch3 Yaw, ch4..15 AUX1..AUX12. The firmware accepts datagrams
from any sender; the only validity test is the exact 40-byte length.

Default layout, written for an OpenTX/EdgeTX radio in USB joystick mode
(e.g. a Radiomaster TX16S, which enumerates as a 6-axis + 2-button HID
joystick), but any joystick with enough axes works:

    axis         RC channel      default binding
    -----------  --------------  ------------------------------------------
    Rx           ch1             Roll      (right stick horizontal)
    Ry           ch2             Pitch     (right stick vertical)
    Throttle     ch3             Throttle  (idle at the raw minimum)
    Rz           ch4             Yaw       (left stick horizontal)
    5th axis     ch5 AUX1        6-position mode dial
    6th axis     ch6 AUX2        3-position toggle
    button 0     ch7 AUX3        ARM switch (high = arm, low = disarm)
    button 1     ch8 AUX4        Trigger (held = high; --trigger-mode toggle)

The 6-position dial uses the firmware's native 6-pos convention
(fc/rc_adjustments.c): the 900..2100 us span split into six 200 us bands,
so the tool emits 1000/1200/1400/1600/1800/2000 us for positions 1..6.
Suggested aux provisioning (CLI lines via `SITL.elf --config`, permanent
box ids from msp_box.c; export with --print-config):

    aux 0 0  2 1700 2100 0 0   # ARM      on AUX3 (switch level)
    aux 1 1  0 1100 1300 0 0   # ANGLE    on AUX1 dial position 2
    aux 2 2  0 1300 1500 0 0   # HORIZON  on AUX1 dial position 3
    aux 3 3  0 1500 1900 0 0   # ALT HOLD on AUX1 dial positions 4 and 5
    aux 4 11 0 1700 1900 0 0   # POS HOLD on AUX1 dial position 5
    aux 5 58 1 1700 2100 0 0   # OFFBOARD on AUX2 3-position high

Arming notes: this fork defaults enable_stick_arming = OFF, so the ARM
switch (AUX3, exposed as a joystick button) is the arming path; the FC additionally requires throttle
below mincheck (1050 us) and the mode dial outside the ALT HOLD / POS HOLD
positions when arming.

Usage:
    python3 sitl_joystick.py --probe                 # inspect the device
    python3 sitl_joystick.py --print-config          # export matching RC/AUX CLI
    python3 sitl_joystick.py                         # bridge to 127.0.0.1:9004
    python3 sitl_joystick.py --monitor               # bridge + live status
    python3 sitl_joystick.py --invert pitch,yaw      # fix directions

Requires read access to the joystick device (input group or a udev ACL).
"""

import argparse
import fcntl
import os
import select
import socket
import struct
import sys
import threading
import time

# --- Linux joystick API (linux/joystick.h) --------------------------------

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12
JSIOCGAXMAP = lambda n: 0x80000000 | (n << 16) | 0x6A32  # noqa: E731

# Axis identifier codes reported by JSIOCGAXMAP
AXIS_IDS = {
    0x00: "X", 0x01: "Y", 0x02: "Z",
    0x03: "Rx", 0x04: "Ry", 0x05: "Rz",
    0x06: "Throttle", 0x07: "Rudder", 0x08: "Wheel", 0x09: "Gear",
    0x0A: "Eileron", 0x0B: "Aileron", 0x0C: "Brake", 0x0D: "Elevator",
}

JS_EVENT_STRUCT = struct.Struct("<IhBB")  # u32 time, s16 value, u8 type, u8 number
JS_EVENT_SIZE = JS_EVENT_STRUCT.size       # 8, no padding

# --- RC mapping ------------------------------------------------------------

RC_MIN_US = 1000
RC_MID_US = 1500
RC_MAX_US = 2000
AXIS_MIN = -32767
AXIS_MAX = 32767

NUM_CHANNELS = 16
CHANNEL_NAMES = ["Roll", "Pitch", "Thr", "Yaw"] + [f"AUX{i}" for i in range(1, 13)]
RC_CHANNEL_INDICES = {
    "roll": 0, "pitch": 1, "throttle": 2, "yaw": 3,
    "mode6": 4, "switch3": 5, "arm": 6, "trigger": 7,
}
CLI_CONFIG_LINES = (
    "map AETR1234",
    "aux 0 0 2 1700 2100 0 0",
    "aux 1 1 0 1100 1300 0 0",
    "aux 2 2 0 1300 1500 0 0",
    "aux 3 3 0 1500 1900 0 0",
    "aux 4 11 0 1700 1900 0 0",
    "aux 5 58 1 1700 2100 0 0",
)

# 6-pos switch values: the firmware's native 200 us bands over 900..2100
MODE6_VALUES_US = [1000, 1200, 1400, 1600, 1800, 2000]
MODE6_NAMES = ["ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD+ALTHOLD", "reserved"]
SWITCH3_VALUES_US = [RC_MIN_US, RC_MID_US, RC_MAX_US]
SWITCH3_NAMES = ["LOW", "MID", "HIGH"]

# Raw thresholds: a 6-pos axis spans -32768..32767 in six equal bands; the
# hysteresis margin keeps a value wobbling on a band edge from flapping.
MODE6_HYST = 700


def axis_to_us(value, invert=False):
    """Map a raw signed axis value linearly onto 1000..2000 us."""
    v = -value if invert else value
    frac = (v - AXIS_MIN) / (AXIS_MAX - AXIS_MIN)
    frac = min(1.0, max(0.0, frac))
    return int(round(RC_MIN_US + frac * (RC_MAX_US - RC_MIN_US)))


def quantize_mode6(raw, last_position):
    """Raw axis value -> 6-pos index 0..5 with hysteresis around band edges.

    Bands are equal sixths of the raw range; `last_position` is the index the
    caller currently reports (or None). An adjacent new index is only adopted
    once the value clears the shared band edge by MODE6_HYST raw counts, so a
    value wobbling on an edge cannot flap between positions.
    """
    raw = min(AXIS_MAX, max(AXIS_MIN, raw))
    span = AXIS_MAX - AXIS_MIN + 1
    pos = min(5, max(0, int((raw - AXIS_MIN) * 6 // span)))
    if last_position is not None and pos != last_position and abs(pos - last_position) == 1:
        edge = AXIS_MIN + (max(pos, last_position) * span) // 6
        if pos > last_position and raw < edge + MODE6_HYST:
            return last_position
        if pos < last_position and raw > edge - MODE6_HYST:
            return last_position
    return pos


def quantize_switch3(raw):
    """Raw axis value -> 3-pos index 0 (low) / 1 (mid) / 2 (high)."""
    if raw > 16384:
        return 2
    if raw < -16384:
        return 0
    return 1


class JoystickState:
    """Pure event-to-channel mapping; no I/O, unit-testable.

    The mapping dict names axis roles ("roll", "pitch", "yaw", "throttle",
    "mode6", "switch3") to joystick axis indices, button roles ("arm",
    "trigger") to button indices, plus the invert set and trigger mode.
    """

    def __init__(self, mapping):
        self.mapping = dict(mapping)
        self.axis_values = {}     # axis index -> last raw value
        self.button_values = {}   # button index -> 0/1
        self.mode6_position = None
        self.switch3_position = None
        self.armed = False
        self.trigger_on = False
        self.changes = []         # human-readable transitions since last drain

    def _axis(self, role):
        return self.axis_values.get(self.mapping[role])

    def feed(self, event):
        """Update state from one (time, value, type, number) event tuple."""
        _, value, ev_type, number = event
        kind = ev_type & ~JS_EVENT_INIT
        if kind == JS_EVENT_AXIS:
            self.axis_values[number] = value
            # independent checks: with fewer axes than roles the dial and the
            # toggle can alias to the same axis, and both positions must
            # still track it (an elif would starve switch3 forever)
            if number == self.mapping["mode6"]:
                self.mode6_position = quantize_mode6(value, self.mode6_position)
            if number == self.mapping["switch3"]:
                self.switch3_position = quantize_switch3(value)
        elif kind == JS_EVENT_BUTTON:
            pressed = bool(value)
            was = bool(self.button_values.get(number, 0))
            self.button_values[number] = 1 if pressed else 0
            if number == self.mapping["arm"]:
                self.armed = pressed
                if pressed != was:
                    self.changes.append(
                        "ARM switch -> {}".format("ON" if pressed else "OFF"))
            elif pressed and not was:
                if number == self.mapping["trigger"]:
                    if self.mapping.get("trigger_mode", "hold") == "toggle":
                        self.trigger_on = not self.trigger_on
                        self.changes.append(
                            "Trigger -> {}".format("ON" if self.trigger_on else "OFF"))
                    else:
                        self.trigger_on = True
                        self.changes.append("Trigger -> ON (held)")
            elif not pressed and was:
                if number == self.mapping["trigger"] and \
                        self.mapping.get("trigger_mode", "hold") == "hold":
                    self.trigger_on = False
                    self.changes.append("Trigger -> OFF")

    def channels(self):
        """The 16 RC channel values in microseconds for the current state."""
        inv = self.mapping.get("invert", set())
        chans = [RC_MID_US] * NUM_CHANNELS
        chans[RC_CHANNEL_INDICES["throttle"]] = RC_MIN_US
        for role in ("roll", "pitch", "yaw"):
            v = self._axis(role)
            if v is not None:
                chans[RC_CHANNEL_INDICES[role]] = axis_to_us(v, role in inv)
        v = self._axis("throttle")
        if v is not None:
            chans[RC_CHANNEL_INDICES["throttle"]] = axis_to_us(v, "throttle" in inv)
        # an unset position (before the first js event, incl. the INIT burst)
        # defaults to the lowest detent rather than midrange, so the very
        # first packets cannot blip an unintended mode band
        chans[RC_CHANNEL_INDICES["mode6"]] = MODE6_VALUES_US[self.mode6_position if self.mode6_position is not None else 0]
        chans[RC_CHANNEL_INDICES["switch3"]] = SWITCH3_VALUES_US[self.switch3_position if self.switch3_position is not None else 0]
        chans[RC_CHANNEL_INDICES["arm"]] = RC_MAX_US if self.armed else RC_MIN_US
        chans[RC_CHANNEL_INDICES["trigger"]] = RC_MAX_US if self.trigger_on else RC_MIN_US
        return chans

    def drain_changes(self):
        out = self.changes
        self.changes = []
        return out

    def describe(self):
        mode = "-" if self.mode6_position is None else "{} ({})".format(
            self.mode6_position + 1, MODE6_NAMES[self.mode6_position])
        sw = "-" if self.switch3_position is None else SWITCH3_NAMES[self.switch3_position]
        return "mode:{} switch:{} arm:{} trigger:{}".format(
            mode, sw, "ON" if self.armed else "off",
            "ON" if self.trigger_on else "off")


def parse_event_stream(buf):
    """Split a byte buffer into js_event tuples: (events, remainder).

    Handles any chunking: reads from /dev/input/js* deliver whole 8-byte
    events, but a stream reader must tolerate arbitrary split points.
    """
    events = []
    offset = 0
    while offset + JS_EVENT_SIZE <= len(buf):
        events.append(JS_EVENT_STRUCT.unpack_from(buf, offset))
        offset += JS_EVENT_SIZE
    return events, buf[offset:]


def open_joystick(path):
    """Open a joystick device and query its layout.

    Returns (fd, info) with info = {name, axes, buttons, axmap} where axmap
    maps axis index -> semantic name from JSIOCGAXMAP (or None).
    """
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    buf = bytearray(128)
    try:
        n = fcntl.ioctl(fd, 0x80006A13, buf, True)  # JSIOCGNAME(128)
        name = bytes(buf[:n]).split(b"\0")[0].decode(errors="replace")
    except OSError:
        name = "?"
    def ioctl_byte(req):
        return struct.unpack("B", fcntl.ioctl(fd, req, b"\0"))[0]
    axes = ioctl_byte(JSIOCGAXES)
    buttons = ioctl_byte(JSIOCGBUTTONS)
    axmap = {}
    try:
        raw = bytearray(axes)
        fcntl.ioctl(fd, JSIOCGAXMAP(axes), raw, True)
        axmap = {i: AXIS_IDS.get(c) for i, c in enumerate(raw)}
    except OSError:
        pass
    return fd, {"name": name, "axes": axes, "buttons": buttons, "axmap": axmap}


def default_mapping(info, overrides=None, invert=(), arm_button=0,
                    trigger_button=1, trigger_mode="hold"):
    """Pick axis/button indices for each role from the device layout.

    Sticks are matched by JSIOCGAXMAP name first (Rx/Ry/Rz/Throttle, which
    is how OpenTX radios enumerate), falling back to positional indices.
    The two axes left over after the sticks become the 6-pos dial and the
    3-pos toggle, in ascending index order.
    """
    axes = info["axes"]
    axmap = info.get("axmap") or {}
    by_name = {}
    for idx, nm in axmap.items():
        if nm:
            by_name.setdefault(nm, idx)
    picked = {}
    for role, names in (("roll", ("Rx", "X")), ("pitch", ("Ry", "Y")),
                        ("yaw", ("Rz", "Z", "Rudder")), ("throttle", ("Throttle",))):
        for nm in names:
            if nm in by_name:
                picked[role] = by_name[nm]
                break
    # positional fallback for roles the axmap did not name: never reuse an
    # index the name matcher already claimed (partially-named layouts are
    # common - e.g. xpad reports X/Y/Z/Rx/Ry/Rz with no Throttle axis)
    claimed = {i for i in picked.values() if i is not None}
    for role, idx in (("roll", 0), ("pitch", 1), ("yaw", 2), ("throttle", 3)):
        if role in picked:
            continue
        free = next((i for i in range(idx, axes) if i not in claimed), None)
        if free is None:
            free = next((i for i in range(axes) if i not in claimed), None)
        if free is not None:
            claimed.add(free)
        picked[role] = free
    stick_idx = {i for i in picked.values() if i is not None}
    leftover = sorted(i for i in range(axes) if i not in stick_idx)
    # Two extra axes expected (dial + toggle); more than two: take the first
    # two, fewer: reuse the last stick axis so the mapping still validates.
    dial = leftover[0] if len(leftover) > 0 else axes - 1
    toggle = leftover[1] if len(leftover) > 1 else dial
    picked["mode6"] = dial
    picked["switch3"] = toggle
    if overrides:
        picked.update({k: v for k, v in overrides.items() if v is not None})
    picked["invert"] = set(invert)
    picked["arm"] = arm_button
    picked["trigger"] = trigger_button
    picked["trigger_mode"] = trigger_mode
    return picked


class RcSender(threading.Thread):
    """Streams rc_packet datagrams at a fixed rate from a channels callable."""

    def __init__(self, channels_fn, dest=("127.0.0.1", 9004), rate_hz=50.0, sock=None):
        super().__init__(daemon=True)
        self.channels_fn = channels_fn
        self.dest = dest
        self.period = 1.0 / rate_hz
        self.running = True
        self.t0 = time.monotonic()
        self.sent = 0
        self.sock = sock if sock is not None else socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        next_t = time.monotonic()
        while self.running:
            pkt = struct.pack("<d16H", time.monotonic() - self.t0, *self.channels_fn())
            try:
                self.sock.sendto(pkt, self.dest)
                self.sent += 1
            except OSError:
                pass
            next_t += self.period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # fell behind; resync

    def stop(self):
        self.running = False
        self.join(timeout=2.0)


# --- CLI actions -----------------------------------------------------------


def print_mapping(info, mapping):
    print("device : {} ({})".format(info.get("name", "?"), info.get("axes", 0)),
          end="")
    print(" axes, {} buttons".format(info.get("buttons", 0)))
    axmap = info.get("axmap") or {}
    for role, label, ch in (("roll", "Roll", "ch1"), ("pitch", "Pitch", "ch2"),
                            ("throttle", "Throttle", "ch3"), ("yaw", "Yaw", "ch4"),
                            ("mode6", "6-pos dial", "ch5/AUX1"),
                            ("switch3", "3-pos toggle", "ch6/AUX2")):
        idx = mapping[role]
        nm = axmap.get(idx) or "?"
        print("  {:<12} <- axis {} (kernel name {}) -> {}".format(label, idx, nm, ch))
    print("  {:<12} <- button {} -> ch7/AUX3 (switch level)".format("ARM", mapping["arm"]))
    print("  {:<12} <- button {} -> ch8/AUX4 ({})".format(
        "Trigger", mapping["trigger"], mapping.get("trigger_mode", "hold")))
    if mapping.get("invert"):
        print("  inverted    : {}".format(",".join(sorted(mapping["invert"]))))


def run_probe(args):
    fd, info = open_joystick(args.device)
    try:
        mapping = default_mapping(info, build_overrides(args), args.invert,
                                  args.arm_button, args.trigger_button, args.trigger_mode)
        print_mapping(info, mapping)
        state = JoystickState(mapping)
        print("\nlive raw values for {} s (wiggle sticks to verify directions):".format(args.seconds))
        deadline = time.monotonic() + args.seconds
        rem = b""
        while time.monotonic() < deadline:
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            try:
                data = os.read(fd, JS_EVENT_SIZE * 64)
            except BlockingIOError:
                continue
            events, rem = parse_event_stream(rem + data)
            for ev in events:
                state.feed(ev)
            axes = " ".join("a{}={:>6}".format(i, state.axis_values.get(i, 0))
                            for i in range(info["axes"]))
            btns = " ".join("b{}={}".format(i, state.button_values.get(i, 0))
                            for i in range(info["buttons"]))
            print("\r  {}  {}  {}".format(axes, btns, state.describe()).ljust(100),
                  end="", flush=True)
        print()
    finally:
        os.close(fd)


def run_bridge(args):
    fd, info = open_joystick(args.device)
    try:
        mapping = default_mapping(info, build_overrides(args), args.invert,
                                  args.arm_button, args.trigger_button, args.trigger_mode)
        print_mapping(info, mapping)
        state = JoystickState(mapping)
        sender = RcSender(state.channels, (args.host, args.port), args.rate)
        sender.start()
        print("streaming {} Hz to {}:{} - Ctrl-C to stop".format(
            args.rate, args.host, args.port))
        rem = b""
        last_status = 0.0
        try:
            while True:
                r, _, _ = select.select([fd], [], [], 0.2)
                if r:
                    data = os.read(fd, JS_EVENT_SIZE * 64)
                    if not data:
                        raise OSError("joystick device went away")
                    events, rem = parse_event_stream(rem + data)
                    for ev in events:
                        state.feed(ev)
                for msg in state.drain_changes():
                    print("  " + msg)
                if args.monitor and time.monotonic() - last_status > 0.25:
                    chans = state.channels()
                    print("\r  {} | {}".format(
                        state.describe(),
                        " ".join(str(c) for c in chans[:8])).ljust(110),
                        end="", flush=True)
                    last_status = time.monotonic()
        except KeyboardInterrupt:
            print("\nstopped ({} frames sent); SITL failsafe takes over)".format(sender.sent))
        finally:
            sender.stop()
    finally:
        os.close(fd)


def build_overrides(args):
    return {
        "roll": args.roll_index, "pitch": args.pitch_index,
        "yaw": args.yaw_index, "throttle": args.throttle_index,
        "mode6": args.mode_index, "switch3": args.switch_index,
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--device", default="/dev/input/js0", help="joystick device")
    p.add_argument("--host", default="127.0.0.1", help="SITL RC destination host")
    p.add_argument("--port", type=int, default=9004, help="SITL RC UDP port")
    p.add_argument("--rate", type=float, default=50.0, help="RC frame rate in Hz")
    p.add_argument("--probe", action="store_true", help="inspect device and exit")
    p.add_argument("--print-config", action="store_true",
                   help="print the matching AETR/AUX CLI configuration without opening a joystick")
    p.add_argument("--seconds", type=float, default=5.0, help="probe duration")
    p.add_argument("--monitor", action="store_true",
                   help="print a live channel status line while bridging")
    p.add_argument("--roll-index", type=int, help="override roll axis index")
    p.add_argument("--pitch-index", type=int, help="override pitch axis index")
    p.add_argument("--yaw-index", type=int, help="override yaw axis index")
    p.add_argument("--throttle-index", type=int, help="override throttle axis index")
    p.add_argument("--mode-index", type=int, help="override 6-pos dial axis index")
    p.add_argument("--switch-index", type=int, help="override 3-pos toggle axis index")
    p.add_argument("--invert", default="", help="comma list of roles to invert")
    p.add_argument("--arm-button", type=int, default=0, help="ARM button index")
    p.add_argument("--trigger-button", type=int, default=1, help="Trigger button index")
    p.add_argument("--trigger-mode", choices=["hold", "toggle"], default="hold",
                   help="Trigger button behaviour")
    args = p.parse_args(argv)
    args.invert = {s.strip() for s in args.invert.split(",") if s.strip()}

    if args.print_config:
        print("\n".join(CLI_CONFIG_LINES))
        return 0

    try:
        if args.probe:
            run_probe(args)
        else:
            run_bridge(args)
    except OSError as e:
        print("error: {} (check the device path and input-group access)".format(e),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
