#!/usr/bin/env python3
"""Offline checks for the real-AirSim control runner; no UE or SITL needed."""

import math
import socket
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import airsim_control_test as control
import sitl_joystick as joystick


class ControlRunnerTest(unittest.TestCase):
    def test_heading_wrap(self):
        self.assertEqual(control.angle_error(2, 358), 4)
        self.assertEqual(control.angle_error(358, 2), -4)

    def test_quaternion_heading(self):
        for heading in (0, 90, 180, 270, 188):
            with self.subTest(heading=heading):
                half = math.radians(heading) / 2
                q = SimpleNamespace(w_val=math.cos(half), x_val=0, y_val=0, z_val=math.sin(half))
                roll, pitch, yaw = control.euler_degrees(q)
                self.assertAlmostEqual(roll, 0)
                self.assertAlmostEqual(pitch, 0)
                self.assertAlmostEqual(control.angle_error(yaw, heading), 0)

    def test_quaternion_pitch_up(self):
        half = math.radians(10) / 2
        q = SimpleNamespace(w_val=math.cos(half), x_val=0, y_val=math.sin(half), z_val=0)
        self.assertAlmostEqual(control.euler_degrees(q)[1], 10)

    def test_status_uses_box_ids_not_flight_mode_bit_numbers(self):
        payload = bytearray(21)
        struct.pack_into("<I", payload, 6, (1 << 0) | (1 << 2))
        struct.pack_into("<I", payload, 17, 1 << 12)
        modes, flags = control.decode_status(payload, [0, 1, 56])
        self.assertEqual(modes, {0, 56})
        self.assertEqual(flags, 1 << 12)

    def test_status_skips_extra_mode_bytes(self):
        payload = bytearray(23)
        payload[15] = 2
        struct.pack_into("<I", payload, 19, 1 << 18)
        self.assertEqual(control.decode_status(payload, [0])[1], 1 << 18)

    def test_truncated_status_is_not_ready(self):
        for payload in (b"", bytes(15), bytes(16)):
            with self.subTest(size=len(payload)), self.assertRaises(RuntimeError):
                control.decode_status(payload, [0])

    def test_provisioning_uses_airsim_gps_and_hover_throttle(self):
        gps = SimpleNamespace(latitude=47, longitude=8, altitude=500)
        config = control.configuration(gps, 1590)
        waypoint = next(line for line in config if line.startswith("waypoint insert"))
        values = waypoint.split()
        self.assertAlmostEqual(float(values[3]), 47 + 20 / 111319.49, places=7)
        self.assertEqual(float(values[4]), 8)
        self.assertEqual(int(values[5]), 50500)
        self.assertIn("set ap_hover_throttle = 1590", config)
        self.assertEqual(config[7:14], list(joystick.CLI_CONFIG_LINES))
        self.assertIn("aux 6 56 5 1700 2100 0 0", config)
        self.assertFalse(any(line.startswith("aux ") and line.split()[2:4] == ["3", "4"]
                             for line in config))

    def test_gps_fix_is_a_nonzero_state_bitmask(self):
        for fix in (0, 2):
            with self.subTest(fix=fix):
                msp = Mock()
                msp.request.side_effect = lambda command: {119: b"\x00\x01", 205: b"",
                                                          106: bytes((fix, 12))}[command]
                runner = control.ControlTest(Mock(), "Copter", msp, Mock(), Mock())
                runner.wait = Mock()
                if fix:
                    runner.sensors()
                else:
                    with self.assertRaises(AssertionError):
                        runner.sensors()

    def test_rc_packet_and_cleanup(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            rc = control.RcStream("127.0.0.1")
            rc.destination = receiver.getsockname()
            rc.set(roll=1600, pitch=1400, arm=2000, autopilot=2000)
            rc.set_mode("POSHOLD+ALTHOLD")
            rc.thread.start()
            try:
                data, _ = receiver.recvfrom(64)
                self.assertEqual(len(data), 40)
                channels = struct.unpack("<d16H", data)[1:]
                self.assertEqual(channels[:4], (1600, 1400, 1000, 1500))
                self.assertEqual(channels[4:8], (1800, 1000, 2000, 1000))
                self.assertEqual(channels[9], 2000)
            finally:
                rc.close()
            self.assertFalse(rc.thread.is_alive())
            self.assertIsNone(rc.error)

    def test_initial_radio_channels_match_joystick(self):
        mapping = joystick.default_mapping({"axes": 6, "buttons": 2})
        state = joystick.JoystickState(mapping)
        rc = control.RcStream("127.0.0.1")
        try:
            self.assertEqual(rc.channels, state.channels())
        finally:
            rc.close()

    def test_mode_dial_does_not_modify_arm_toggle_or_trigger(self):
        rc = control.RcStream("127.0.0.1")
        try:
            rc.set(arm=2000, switch3=2000, trigger=2000)
            for mode, value in zip(joystick.MODE6_NAMES, joystick.MODE6_VALUES_US):
                with self.subTest(mode=mode):
                    rc.set_mode(mode)
                    self.assertEqual(rc.channels[4:8], [value, 2000, 2000, 2000])
        finally:
            rc.close()


if __name__ == "__main__":
    unittest.main()
