#!/usr/bin/env python3
"""Offline checks for the real-AirSim control runner; no UE or SITL needed."""

import math
import socket
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import airsim_control_test as control
import airsim_offboard_test as offboard
import sitl_joystick as joystick


class ControlRunnerTest(unittest.TestCase):
    def test_pilot_mode_takeover_requires_actual_mode_and_arming_not_offboard(self):
        for mode, modes in (("ACRO", {0}), ("ANGLE", {0, 1}), ("HORIZON", {0, 2}),
                            ("ALTHOLD", {0, 1, 3}), ("POSHOLD+ALTHOLD", {0, 1, 3, 11})):
            with self.subTest(mode=mode):
                self.assertTrue(offboard.pilot_mode_active({"modes": modes}, mode))
                self.assertFalse(offboard.pilot_mode_active({"modes": modes | {58}}, mode))
                self.assertFalse(offboard.pilot_mode_active({"modes": modes | {56}}, mode))
                self.assertFalse(offboard.pilot_mode_active({"modes": modes - {0}}, mode))

    def test_offboard_host_controller_uses_flu_pitch_and_vertical_feedback(self):
        rates, throttle = offboard.rate_throttle(5, 5, 2, 0, 2, 0.59)
        self.assertLess(rates[0], 0)
        self.assertGreater(rates[1], 0)
        self.assertGreater(throttle, 0.59)
        _, climb = offboard.rate_throttle(0, 0, 1, 0, 2, 0.59)
        _, descend = offboard.rate_throttle(0, 0, 3, 0, 2, 0.59)
        self.assertGreater(climb, 0.59)
        self.assertLess(descend, 0.59)
        _, braking = offboard.rate_throttle(0, 0, 2, 1, 2, 0.59)
        self.assertGreater(braking, 0.59)

    def test_offboard_rate_override_and_bounds(self):
        rates, throttle = offboard.rate_throttle(30, 30, -10, 10, 2, 0.59, (2, 0.35))
        self.assertEqual(rates, (-0.5, 0.5, 0.35))
        self.assertEqual(throttle, 0.78)

    def test_offboard_horizontal_braking_uses_body_heading_and_preserves_rate_override(self):
        for velocity, heading, axis, sign in (((1, 0), 0, 1, -1), ((0, 1), 0, 0, -1),
                                             ((1, 0), 90, 0, 1), ((0, 1), 90, 1, -1)):
            with self.subTest(velocity=velocity, heading=heading):
                rates, _ = offboard.rate_throttle(0, 0, 2, 0, 2, 0.17,
                                                   horizontal_velocity=velocity, heading_deg=heading)
                self.assertGreater(rates[axis] * sign, 0)
                rates, _ = offboard.rate_throttle(0, 0, 2, 0, 2, 0.17, (axis, 0.2),
                                                   velocity, heading)
                self.assertEqual(rates[axis], 0.2)

    def test_offboard_touchdown_requires_fresh_ground_contact_and_low_velocity(self):
        collision = SimpleNamespace(has_collided=True, time_stamp=200, normal=SimpleNamespace(z_val=-1))
        self.assertTrue(offboard.touchdown_confirmed(collision, 100, (0, 0, 0)))
        self.assertFalse(offboard.touchdown_confirmed(collision, 200, (0, 0, 0)))
        self.assertFalse(offboard.touchdown_confirmed(collision, 100, (0, 0, 1)))
        collision.normal.z_val = 0
        self.assertFalse(offboard.touchdown_confirmed(collision, 100, (0, 0, 0)))
        collision.normal.z_val = -1
        collision.has_collided = False
        self.assertFalse(offboard.touchdown_confirmed(collision, 100, (0, 0, 0)))
    def test_offboard_rate_assertion_checks_all_axis_signs_and_units(self):
        for axis in (0, 1, 2):
            with self.subTest(axis=axis):
                sample = [0.0, 0.0, 0.0]
                sample[axis] = 0.2 if axis == 0 else -0.2
                result = offboard.assert_rate_response([sample] * 4, axis, 0.2)
                self.assertAlmostEqual(result["gain"], 1)
                sample[axis] *= -1
                with self.assertRaises(AssertionError):
                    offboard.assert_rate_response([sample] * 4, axis, 0.2)

    def test_api_cleanup_disarms_and_disables_control_on_rejected_disarm(self):
        test = Mock()
        test.client.isApiControlEnabled.return_value = True
        test.client.armDisarm.return_value = False
        with self.assertRaisesRegex(RuntimeError, "not confirmed"):
            control.disarm_test(test)
        test.client.enableApiControl.assert_called_once_with(False, vehicle_name=test.vehicle)
        test.rc.set.assert_called_once_with(arm=1000, throttle=1000, yaw=1500, autopilot=1000)
        test.wait.assert_called_once()

    def test_offboard_rpc_rejection_is_not_hidden_by_future_join(self):
        test = Mock()
        test.client.moveByAngleRatesThrottleAsync.return_value.get.return_value = False
        scenario = offboard.OffboardTest(test, 0.59)
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            scenario.command((0, 0, 0), 0)
        test.client.moveByAngleRatesThrottleAsync.return_value.get.assert_called_once()

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
        config = control.configuration(gps, 1211)
        waypoint = next(line for line in config if line.startswith("waypoint insert"))
        values = waypoint.split()
        self.assertAlmostEqual(float(values[3]), 47 + 20 / 111319.49, places=7)
        self.assertEqual(float(values[4]), 8)
        self.assertEqual(int(values[5]), 50500)
        self.assertIn("set ap_hover_throttle = 1211", config)
        self.assertIn("set ap_throttle_min = 1050", config)
        self.assertIn("set airmode_start_throttle_percent = 15", config)
        self.assertEqual([line for line in config if line.startswith(("map ", "aux "))][:-1],
                         list(joystick.CLI_CONFIG_LINES))
        self.assertIn("aux 7 56 5 1700 2100 0 0", config)
        self.assertEqual([line for line in config if line.startswith("aux 6 ")],
                         ["aux 6 46 0 1900 2100 0 0"])
        self.assertFalse(any(line.startswith("aux ") and line.split()[2:4] == ["3", "4"]
                             for line in config))

    def test_f70_hover_preserves_headroom_below_hover_and_correct_neutral_stick(self):
        hover = (1211 - 1050) / 950
        rotor_signal = 0.055 + 0.945 * hover
        self.assertAlmostEqual(rotor_signal * 4 * 22.76123465, 2 * 9.80665, delta=0.03)
        self.assertEqual(control.alt_hold_neutral_pwm(1211), 1250)
        self.assertLess(15, (1211 - 1050) * 100 / 950)
        _, thrust = offboard.rate_throttle(0, 0, 2, 0, 2, hover)
        self.assertAlmostEqual(thrust, hover)
        _, descent = offboard.rate_throttle(0, 0, 3, 0, 2, hover)
        self.assertLess(descent, hover)
        self.assertEqual(offboard.rate_throttle(0, 0, 10, -10, 0, hover)[1], 0)

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

    def test_waypoint_dwell_covers_delayed_drift_and_altitude_rollback(self):
        msp = Mock()
        msp.request.return_value = bytes((0, 3, 11, 56))
        runner = control.ControlTest(Mock(), "Copter", msp, Mock(), Mock())
        runner.origin = (0.0, 0.0, 0.0)
        runner.wait = Mock()
        runner.mission()
        predicate = runner.wait.call_args.args[1]
        self.assertEqual(runner.wait.call_args.kwargs["dwell"], 60)
        state = {"position": (20, 0, -5), "velocity": (0, 0, 0), "modes": [0, 3, 11, 56]}
        self.assertTrue(predicate(state))
        self.assertFalse(predicate({**state, "position": (20, 0, -3)}))
        with self.assertRaisesRegex(AssertionError, "drift"):
            predicate({**state, "position": (30, 0, -5)})
        with self.assertRaisesRegex(AssertionError, "lost"):
            predicate({**state, "modes": [0, 3, 11]})

    def test_landing_uses_full_low_throttle_without_disarming_early(self):
        rc = control.RcStream("127.0.0.1")
        msp = Mock()
        msp.request.return_value = bytes((0, 3, 11))
        runner = control.ControlTest(Mock(), "Copter", msp, rc, Mock())
        runner.origin = (0.0, 0.0, 0.0)
        snapshots = []
        runner.wait = Mock(side_effect=lambda *_args, **_kwargs: snapshots.append(list(rc.channels)))
        try:
            rc.set(arm=2000)
            rc.set_mode("POSHOLD+ALTHOLD")
            runner.land()
            self.assertEqual(snapshots[0][2], 1000)
            self.assertEqual(snapshots[0][4], 1800)
            self.assertEqual(snapshots[0][6], 2000)
            self.assertEqual(snapshots[1][6], 2000)
            self.assertEqual(snapshots[2][6], 1000)
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
