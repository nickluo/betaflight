#!/usr/bin/env python3
"""Unit tests for the SITL joystick bridge mapping logic (no hardware needed).

Run:
    python3 src/test/sitl/sitl_joystick_test.py
"""

import os
import contextlib
import io
import socket
import struct
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sitl_joystick as sj  # noqa: E402


def ev(axis, number, value, init=False):
    return (0, value, (sj.JS_EVENT_AXIS if axis else sj.JS_EVENT_BUTTON) |
            (sj.JS_EVENT_INIT if init else 0), number)


TX16S_MAPPING = {
    "roll": 0, "pitch": 1, "yaw": 2, "throttle": 3,
    "mode6": 4, "switch3": 5, "arm": 0, "trigger": 1,
    "invert": set(), "trigger_mode": "hold",
}


class TestMappingHelpers(unittest.TestCase):
    def test_export_cli_config_without_a_device(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = sj.main(["--print-config", "--device", "/nonexistent/joystick"])
        self.assertEqual(result, 0)
        self.assertEqual(output.getvalue().splitlines(), list(sj.CLI_CONFIG_LINES))

    def test_cli_table_matches_radio_channel_roles(self):
        self.assertEqual(sj.CLI_CONFIG_LINES[0], "map AETR1234")
        bindings = [list(map(int, line.split()[1:])) for line in sj.CLI_CONFIG_LINES[1:]]
        self.assertEqual(bindings, [
            [0, 0, sj.RC_CHANNEL_INDICES["arm"] - 4, 1700, 2100, 0, 0],
            [1, 1, sj.RC_CHANNEL_INDICES["mode6"] - 4, 1100, 1300, 0, 0],
            [2, 2, sj.RC_CHANNEL_INDICES["mode6"] - 4, 1300, 1500, 0, 0],
            [3, 3, sj.RC_CHANNEL_INDICES["mode6"] - 4, 1500, 1900, 0, 0],
            [4, 11, sj.RC_CHANNEL_INDICES["mode6"] - 4, 1700, 1900, 0, 0],
            [5, 58, sj.RC_CHANNEL_INDICES["switch3"] - 4, 1700, 2100, 0, 0],
            [6, 46, sj.RC_CHANNEL_INDICES["mode6"] - 4, 1900, 2100, 0, 0],
        ])

    def test_fifth_detent_activates_position_and_altitude_hold(self):
        def modes_at(value):
            modes = set()
            for line in sj.CLI_CONFIG_LINES[1:]:
                _, mode, aux, start, end, _, _ = map(int, line.split()[1:])
                if aux == 0 and start <= value < end:
                    modes.add(mode)
            return modes
        self.assertEqual(modes_at(1400), {2})
        self.assertEqual(modes_at(1600), {3})
        self.assertEqual(modes_at(1800), {3, 11})
        self.assertEqual(modes_at(2000), {46})
        self.assertEqual(sj.MODE6_NAMES[4], "POSHOLD+ALTHOLD")
        self.assertEqual(sj.MODE6_NAMES[5], "GPSRESCUE")

    def test_axis_to_us_extremes_and_center(self):
        self.assertEqual(sj.axis_to_us(-32767), 1000)
        self.assertEqual(sj.axis_to_us(0), 1500)
        self.assertEqual(sj.axis_to_us(32767), 2000)

    def test_axis_to_us_invert(self):
        self.assertEqual(sj.axis_to_us(-32767, invert=True), 2000)
        self.assertEqual(sj.axis_to_us(32767, invert=True), 1000)

    def test_axis_to_us_clamps(self):
        self.assertEqual(sj.axis_to_us(-40000), 1000)
        self.assertEqual(sj.axis_to_us(40000), 2000)

    def test_parse_event_stream_whole_and_partial(self):
        buf = struct.pack("<IhBB", 1, -100, sj.JS_EVENT_AXIS, 3)
        events, rem = sj.parse_event_stream(buf)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][1], -100)
        self.assertEqual(rem, b"")
        # a read split mid-event leaves the tail as remainder
        events, rem = sj.parse_event_stream(buf[:5])
        self.assertEqual(events, [])
        self.assertEqual(rem, buf[:5])
        events, rem2 = sj.parse_event_stream(rem + buf[5:] + buf)
        self.assertEqual(len(events), 2)
        self.assertEqual(rem2, b"")

    def test_quantize_mode6_detents(self):
        # OpenTX 6-pos switch emits six evenly spaced levels
        levels = [-32767, -19660, -6554, 6553, 19660, 32767]
        for want, raw in enumerate(levels):
            pos = None
            for _ in range(2):  # second pass exercises the hysteresis path
                pos = sj.quantize_mode6(raw, pos)
            self.assertEqual(pos, want, msg="raw {}".format(raw))

    def test_quantize_mode6_hysteresis_holds_near_edge(self):
        pos = sj.quantize_mode6(-32767, None)
        self.assertEqual(pos, 0)
        # band 0/1 edge sits at -32767 + 65536/6 = -21845
        self.assertEqual(sj.quantize_mode6(-21500, 0), 0)   # past edge, inside hysteresis
        self.assertEqual(sj.quantize_mode6(-21000, 0), 1)   # beyond edge + margin
        self.assertEqual(sj.quantize_mode6(-22000, 1), 1)   # above edge - margin
        self.assertEqual(sj.quantize_mode6(-22700, 1), 0)   # back into band 0

    def test_quantize_mode6_far_jump_adopts(self):
        self.assertEqual(sj.quantize_mode6(32767, 0), 5)

    def test_quantize_switch3(self):
        self.assertEqual(sj.quantize_switch3(-32767), 0)
        self.assertEqual(sj.quantize_switch3(0), 1)
        self.assertEqual(sj.quantize_switch3(32767), 2)


class TestJoystickState(unittest.TestCase):
    def make_state(self, **kw):
        mapping = dict(TX16S_MAPPING)
        mapping.update(kw)
        return sj.JoystickState(mapping)

    def feed_init(self, state):
        # power-on state as delivered via JS_EVENT_INIT: sticks centred,
        # throttle idle, dial and toggle at position 0
        for number in (0, 1, 2):
            state.feed(ev(True, number, 0, init=True))
        state.feed(ev(True, 3, -32767, init=True))
        state.feed(ev(True, 4, -32767, init=True))
        state.feed(ev(True, 5, -32767, init=True))

    def test_initial_channels(self):
        st = self.make_state()
        self.feed_init(st)
        self.assertEqual(st.channels()[:8],
                         [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000])

    def test_stick_motion(self):
        st = self.make_state()
        self.feed_init(st)
        st.feed(ev(True, 0, 32767))    # roll right
        st.feed(ev(True, 1, -32767))   # pitch forward (nose down)
        st.feed(ev(True, 2, 32767))    # yaw right
        st.feed(ev(True, 3, 32767))    # throttle full
        self.assertEqual(st.channels()[:4], [2000, 1000, 2000, 2000])

    def test_invert(self):
        st = self.make_state(invert={"pitch"})
        self.feed_init(st)
        st.feed(ev(True, 1, -32767))
        self.assertEqual(st.channels()[1], 2000)

    def test_mode6_positions(self):
        st = self.make_state()
        self.feed_init(st)
        levels = [-32767, -19660, -6554, 6553, 19660, 32767]
        for pos, raw in enumerate(levels):
            st.feed(ev(True, 4, raw))
            self.assertEqual(st.channels()[4], sj.MODE6_VALUES_US[pos])
        self.assertEqual(sj.MODE6_VALUES_US, [1000, 1200, 1400, 1600, 1800, 2000])

    def test_switch3(self):
        st = self.make_state()
        self.feed_init(st)
        for pos, raw in enumerate((-32767, 0, 32767)):
            st.feed(ev(True, 5, raw))
            self.assertEqual(st.channels()[5], [1000, 1500, 2000][pos])

    def test_arm_switch_follows_button_level(self):
        st = self.make_state()
        self.feed_init(st)
        self.assertEqual(st.channels()[6], 1000)
        st.feed(ev(False, 0, 1))       # switch high
        self.assertEqual(st.channels()[6], 2000)
        st.feed(ev(False, 0, 1))       # repeated high does not toggle
        self.assertEqual(st.channels()[6], 2000)
        st.feed(ev(False, 0, 0))
        self.assertEqual(st.channels()[6], 1000)
        st.feed(ev(False, 0, 1))
        self.assertEqual(st.channels()[6], 2000)
        st.feed(ev(False, 0, 0))
        self.assertEqual(st.channels()[6], 1000)
        self.assertEqual(st.drain_changes(),
                         ["ARM switch -> ON", "ARM switch -> OFF",
                          "ARM switch -> ON", "ARM switch -> OFF"])

    def test_arm_init_tracks_switch_without_using_trigger_toggle_mode(self):
        st = self.make_state(trigger_mode="toggle")
        st.feed(ev(False, 0, 1, init=True))
        self.assertEqual(st.channels()[6], 2000)
        st.feed(ev(False, 0, 1, init=True))
        self.assertEqual(st.channels()[6], 2000)
        st.feed(ev(False, 0, 0, init=True))
        self.assertEqual(st.channels()[6], 1000)

    def test_trigger_hold_and_toggle(self):
        st = self.make_state()
        self.feed_init(st)
        st.feed(ev(False, 1, 1))
        self.assertEqual(st.channels()[7], 2000)
        st.feed(ev(False, 1, 0))
        self.assertEqual(st.channels()[7], 1000)

        st = self.make_state(trigger_mode="toggle")
        self.feed_init(st)
        st.feed(ev(False, 1, 1))
        st.feed(ev(False, 1, 0))
        self.assertEqual(st.channels()[7], 2000)
        st.feed(ev(False, 1, 1))
        st.feed(ev(False, 1, 0))
        self.assertEqual(st.channels()[7], 1000)

    def test_unused_axes_and_buttons_ignored(self):
        st = self.make_state()
        self.feed_init(st)
        st.feed(ev(True, 9, 32767))
        st.feed(ev(False, 5, 1))
        ch = st.channels()
        self.assertEqual(ch[:8], [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000])


class TestDefaultMapping(unittest.TestCase):
    TX16S = {"name": "OpenTX Radiomaster TX16S Joystick", "axes": 6,
             "buttons": 2,
             "axmap": {0: "Rx", 1: "Ry", 2: "Rz", 3: "Throttle",
                       4: "Rudder", 5: "Wheel"}}

    def test_matches_tx16s_layout_by_name(self):
        m = sj.default_mapping(self.TX16S)
        self.assertEqual((m["roll"], m["pitch"], m["yaw"], m["throttle"]),
                         (0, 1, 2, 3))
        self.assertEqual((m["mode6"], m["switch3"]), (4, 5))

    def test_fallback_without_axmap_names(self):
        info = dict(self.TX16S, axmap={})
        m = sj.default_mapping(info)
        self.assertEqual((m["roll"], m["pitch"], m["yaw"], m["throttle"],
                          m["mode6"], m["switch3"]), (0, 1, 2, 3, 4, 5))

    def test_overrides(self):
        m = sj.default_mapping(self.TX16S,
                               {"roll": 2, "mode6": 5, "pitch": None},
                               invert={"yaw"})
        self.assertEqual(m["roll"], 2)
        self.assertEqual(m["mode6"], 5)
        self.assertEqual(m["pitch"], 1)   # None means "keep the default"
        self.assertEqual(m["invert"], {"yaw"})

    def test_partially_named_layout_has_no_axis_collisions(self):
        # xpad-style layout: X/Y/Z/Rx/Ry/Rz with no axis named Throttle
        info = {"name": "Generic Gamepad", "axes": 6, "buttons": 11,
                "axmap": {0: "X", 1: "Y", 2: "Z", 3: "Rx", 4: "Ry", 5: "Rz"}}
        m = sj.default_mapping(info)
        sticks = [m["roll"], m["pitch"], m["yaw"], m["throttle"]]
        self.assertEqual(sticks, [3, 4, 5, 0])  # Rx, Ry, Rz named; throttle falls back
        self.assertEqual(len(set(sticks)), 4)
        self.assertEqual((m["mode6"], m["switch3"]), (1, 2))  # leftover axes

    def test_four_axis_layout_yields_distinct_sticks(self):
        info = {"name": "stick", "axes": 4, "buttons": 1,
                "axmap": {0: "X", 1: "Y", 2: "Z", 3: "Rz"}}
        m = sj.default_mapping(info)
        sticks = [m["roll"], m["pitch"], m["yaw"], m["throttle"]]
        self.assertEqual(len(set(sticks)), 4)
        self.assertTrue(all(i is not None for i in sticks))

    def test_aliased_dial_and_toggle_both_track_the_shared_axis(self):
        # fewer than two leftover axes -> dial and toggle share one axis
        info = {"name": "mini", "axes": 4, "buttons": 0, "axmap": {}}
        m = sj.default_mapping(info)
        self.assertEqual(m["mode6"], m["switch3"])
        st = sj.JoystickState(m)
        st.feed(ev(True, m["switch3"], -32767))
        self.assertEqual(st.mode6_position, 0)
        self.assertEqual(st.switch3_position, 0)
        self.assertEqual(st.channels()[4:6], [1000, 1000])
        st.feed(ev(True, m["switch3"], 32767))
        self.assertEqual(st.mode6_position, 5)
        self.assertEqual(st.switch3_position, 2)
        self.assertEqual(st.channels()[4:6], [2000, 2000])


class TestRcSender(unittest.TestCase):
    def test_packet_format_and_rate(self):
        got = []

        class FakeSock:
            def sendto(self, pkt, dest):
                got.append((pkt, dest, time.monotonic()))

        want = [1500, 1510, 1000, 1490] + [1200, 1500, 1000, 1000] + [1000] * 8
        sender = sj.RcSender(lambda: list(want), ("127.0.0.1", 9004), rate_hz=50.0,
                             sock=FakeSock())
        sender.start()
        time.sleep(0.32)
        sender.stop()
        self.assertGreaterEqual(len(got), 10)
        pkt, dest, _ = got[0]
        self.assertEqual(dest, ("127.0.0.1", 9004))
        self.assertEqual(len(pkt), 40)
        ts, *chans = struct.unpack("<d16H", pkt)
        self.assertGreaterEqual(ts, 0.0)
        self.assertLess(ts, 5.0)
        self.assertEqual(chans, want)
        # ~50 Hz within tolerance
        rate = (len(got) - 1) / (got[-1][2] - got[0][2])
        self.assertGreater(rate, 40.0)
        self.assertLess(rate, 60.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
