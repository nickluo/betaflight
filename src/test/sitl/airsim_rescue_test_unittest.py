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

    def test_low_simulation_profile_does_not_modify_pid_or_rx_loss_policy(self):
        self.assertIn("set gps_rescue_return_alt = 8", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_ground_speed = 250", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_descend_rate = 50", rescue.SIM_RESCUE_CONFIG)
        self.assertIn("set gps_rescue_allow_arming_without_fix = OFF", rescue.SIM_RESCUE_CONFIG)
        self.assertFalse(any("yaw =" in line or "failsafe_procedure" in line
                             for line in rescue.SIM_RESCUE_CONFIG))

    def test_distance_uses_airsim_horizontal_ground_truth(self):
        self.assertEqual(rescue.distance_home({"position": (3, 4, -10)}, (0, 0, 0)), 5)

    def test_export_config_without_vehicle_or_binary(self):
        output = io.StringIO()
        with patch("sys.argv", ["airsim_rescue_test.py", "--print-config"]), \
                contextlib.redirect_stdout(output):
            self.assertEqual(rescue.main(), 0)
        self.assertEqual(output.getvalue().splitlines(),
                         [*joystick.CLI_CONFIG_LINES, *rescue.SIM_RESCUE_CONFIG])


if __name__ == "__main__":
    unittest.main()
