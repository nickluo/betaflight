#!/usr/bin/env python3
"""Offline checks for sixth-detent Rescue wiring and return-home test inputs."""

import contextlib
import io
from unittest.mock import patch
import unittest

import airsim_rescue_test as rescue
import sitl_joystick as joystick


class RescueInputsTest(unittest.TestCase):
    def test_canonical_sixth_detent_selects_only_rescue(self):
        rules = [list(map(int, line.split()[1:])) for line in joystick.CLI_CONFIG_LINES[1:]]
        modes = {mode for _, mode, aux, low, high, _, _ in rules
                 if aux == 0 and low <= 2000 < high}
        self.assertEqual(modes, {46})
        self.assertEqual(joystick.MODE6_NAMES[5], "GPSRESCUE")

    def test_rescue_synthesized_controller_and_legacy_are_distinguished(self):
        self.assertEqual(rescue.rescue_controller([0, 46]), "legacy-gps-rescue")
        self.assertEqual(rescue.rescue_controller([0, 56, 3, 11]), "flight-plan-rescue")
        self.assertIsNone(rescue.rescue_controller([0, 3, 11]))
        self.assertIsNone(rescue.rescue_controller([0, 58]))

    def test_simulation_profile_tunes_outer_loop_not_rate_pid_or_rx_loss(self):
        self.assertIn("set gps_rescue_return_alt = 8", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_ground_speed = 300", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_landing_rate = 50", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_allow_arming_without_fix = OFF", rescue.SIM_RESCUE_CONFIG)
        for setting in ("ap_position_p = 6", "ap_velocity_drag_coeff = 0",
                        "ap_yaw_mode = BEARING", "ap_max_angle = 25", "ap_nav_accel = 100"):
            self.assertIn(f"set {setting}", rescue.SIM_RESCUE_CONFIG)
        forbidden = {f"{term}_{axis}" for term in "pidf" for axis in ("roll", "pitch", "yaw")}
        forbidden.update(("failsafe_procedure", "ap_rx_loss_policy", "imu_dcm_kp"))
        self.assertFalse(any(line.split()[1] in forbidden for line in rescue.SIM_RESCUE_CONFIG
                             if line.startswith("set ")))

    def test_distance_uses_airsim_horizontal_ground_truth(self):
        self.assertEqual(rescue.distance_home({"position": (3, 4, -10)}, (0, 0, 0)), 5)

    def test_export_config_without_vehicle_or_binary(self):
        output = io.StringIO()
        with patch("sys.argv", ["airsim_rescue_test.py", "--print-config"]), \
                contextlib.redirect_stdout(output):
            self.assertEqual(rescue.main(), 0)
        self.assertEqual(output.getvalue().splitlines(),
                         [*joystick.CLI_CONFIG_LINES, *rescue.SIM_RESCUE_CONFIG])


class ReturnFlightMetricsTest(unittest.TestCase):
    def metrics(self, velocity=(-3, 0, 0), attitude=(0, 3, 0)):
        metrics = rescue.ReturnFlightMetrics()
        metrics.update(0, (80, 0), (10, 0, 5), (0, 40, 0))
        for i in range(30):
            metrics.update(6 + i * .1, (70 - i * .3, 0), velocity, attitude)
        return metrics

    def test_entry_and_landing_are_separate_from_cruise(self):
        metrics = self.metrics()
        metrics.update(50, (1, 0), (0, 0, .5), (0, 0, 0))
        summary = metrics.summary()
        self.assertEqual(summary["cruise_samples"], 30)
        self.assertEqual(summary["peak_horizontal_speed_m_s"], 10)
        self.assertAlmostEqual(summary["peak_tilt_deg"], 40)
        self.assertAlmostEqual(summary["cruise_speed_mean_m_s"], 3)
        self.assertAlmostEqual(summary["cruise_speed_std_m_s"], 0)
        self.assertAlmostEqual(summary["cruise_speed_error_rms_m_s"], 0)
        self.assertAlmostEqual(summary["cruise_peak_tilt_deg"], 3)
        metrics.require_stable_cruise()

    def test_no_cruise_cannot_pass_stability(self):
        metrics = rescue.ReturnFlightMetrics()
        metrics.update(0, (3, 0), (0, 0, 0), (0, 0, 0))
        self.assertEqual(metrics.summary()["cruise_samples"], 0)
        with self.assertRaisesRegex(AssertionError, "too few"):
            metrics.require_stable_cruise()

    def test_reproduced_speed_oscillation_is_rejected(self):
        metrics = self.metrics()
        for i in range(30):
            metrics.update(10 + i * .1, (55 - i * .1, 0),
                           ((-6 if i % 2 else 2), 0, 0), (0, 35, 0))
        with self.assertRaisesRegex(AssertionError, "unstable Rescue"):
            metrics.require_stable_cruise()
        self.assertGreater(metrics.summary()["cruise_reverse_fraction"], .05)

    def test_reversing_at_requested_speed_is_not_stable_return(self):
        metrics = self.metrics(velocity=(3, 0, 0))
        self.assertEqual(metrics.summary()["cruise_speed_error_rms_m_s"], 0)
        with self.assertRaisesRegex(AssertionError, "unstable Rescue"):
            metrics.require_stable_cruise()

    def test_zero_speed_samples_are_not_hidden(self):
        metrics = self.metrics(velocity=(0, 0, 0))
        self.assertEqual(metrics.summary()["cruise_samples"], 30)
        self.assertEqual(metrics.summary()["cruise_speed_error_rms_m_s"], 3)
        with self.assertRaisesRegex(AssertionError, "unstable Rescue"):
            metrics.require_stable_cruise()

    def test_excessive_tilt_is_rejected_even_at_cruise_speed(self):
        metrics = self.metrics(attitude=(20, 20, 0))
        with self.assertRaisesRegex(AssertionError, "unstable Rescue"):
            metrics.require_stable_cruise()


if __name__ == "__main__":
    unittest.main()
