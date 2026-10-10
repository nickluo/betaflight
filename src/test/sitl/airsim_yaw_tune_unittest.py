#!/usr/bin/env python3
"""Offline regressions for Yaw response scoring and safe parameter application."""

import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import airsim_yaw_tune as tune


def response(direction=1, ringing=False, gain=1.0):
    step = [tune.Sample(i / 20, direction * 38 * gain, direction * 38,
                        (350 + direction * 38 * gain * i / 20) % 360) for i in range(25)]
    tail = []
    heading = step[-1].heading_deg
    for i in range(161):
        t = i / 20
        rate = direction * (10 * math.sin(3 * t) if ringing else 38 * math.exp(-10 * t))
        heading = (heading + rate / 20) % 360
        tail.append(tune.Sample(t, rate, 0, heading))
    return step, tail


def delayed_response(tau, direction=1):
    step, tail = response(direction)
    step = [tune.replace(s, rate_dps=direction * 38 * (1 - math.exp(-s.time_s / tau)))
            for s in step]
    return step, tail


class YawMetricsTest(unittest.TestCase):
    def test_clock_must_advance_in_real_time(self):
        tune.verify_clock(1_000_000_000, 2_000_000_000, 1)
        for second in (1_000_000_000, 3_000_000_000):
            with self.subTest(second=second), self.assertRaises(RuntimeError):
                tune.verify_clock(1_000_000_000, second, 1)
        with self.assertRaises(ValueError):
            tune.verify_clock(1_000_000_000, 2_000_000_000, 0)

    def test_stable_response_and_heading_wrap_both_directions(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                step, tail = response(direction)
                metrics = tune.measure(step, tail, direction)
                self.assertTrue(metrics.stable)
                self.assertAlmostEqual(metrics.response_gain, 1)
                self.assertAlmostEqual(metrics.travel_deg, 45.6)
                self.assertEqual(metrics.reverse_deg, 0)
                self.assertLessEqual(metrics.settling_s, 0.35)

    def test_ringing_is_not_accepted(self):
        step, tail = response(ringing=True)
        metrics = tune.measure(step, tail, 1)
        self.assertFalse(metrics.stable)
        self.assertGreater(metrics.reverse_deg, 1.5)
        self.assertGreater(metrics.settling_s, 2)

    def test_nonresponsive_low_gain_is_not_good_tuning(self):
        step, tail = response(gain=0.2)
        metrics = tune.measure(step, tail, 1)
        self.assertFalse(metrics.responsive)
        self.assertFalse(metrics.stable)

    def test_neutral_target_must_really_be_zero(self):
        step, tail = response()
        tail = [tune.replace(s, target_dps=3) for s in tail]
        with self.assertRaisesRegex(ValueError, "not neutral"):
            tune.measure(step, tail, 1)

    def test_invalid_or_sparse_samples_are_errors(self):
        step, tail = response()
        for invalid in (step[:2], [tune.replace(step[0], rate_dps=math.nan), *step[1:]],
                        [step[0], step[0], *step[1:]], step[::10]):
            with self.subTest(samples=len(invalid)), self.assertRaises(ValueError):
                tune.measure(invalid, tail, 1)

    def test_independent_repeat_requires_stability_and_ten_percent_gain(self):
        step, tail = response()
        good = tune.aggregate([tune.measure(step, tail, 1)] * 2)
        baseline: tune.TrialResult = {**good, "score": good["score"] * 2}
        too_close: tune.TrialResult = {**good, "score": good["score"] * 1.05}
        unstable: tune.TrialResult = {**good, "stable": False}
        self.assertTrue(tune.validated_improvement(baseline, good))
        self.assertFalse(tune.validated_improvement(too_close, good))
        self.assertFalse(tune.validated_improvement(baseline, unstable))

    def test_stable_candidate_beats_lower_score_that_misses_absolute_limits(self):
        step, tail = response()
        stable = tune.aggregate([tune.measure(step, tail, 1)] * 2)
        unstable: tune.TrialResult = {**stable, "stable": False, "score": 0}
        weak: tune.TrialResult = {**stable, "responsive": False, "score": 0}
        self.assertTrue(tune.better(stable, unstable))
        self.assertFalse(tune.better(unstable, stable))
        self.assertFalse(tune.better(weak, stable))

    def test_response_objective_penalizes_slow_start_even_if_tail_is_stable(self):
        fast_step, tail = delayed_response(0.12)
        slow_step, _ = delayed_response(0.45)
        fast = tune.measure(fast_step, tail, 1)
        slow = tune.measure(slow_step, tail, 1)
        self.assertTrue(fast.stable)
        self.assertTrue(slow.stable)
        self.assertTrue(fast.response_ready)
        self.assertFalse(slow.response_ready)
        self.assertGreater(slow.rise80_s, fast.rise80_s)
        self.assertGreater(slow.pulse_tracking_rmse, fast.pulse_tracking_rmse)
        fast_result = tune.aggregate([fast] * 2)
        slow_result = tune.aggregate([slow] * 2)
        self.assertTrue(tune.better(fast_result, slow_result, "response"))
        self.assertTrue(tune.validated_improvement(slow_result, fast_result, "response"))

    def test_brief_peak_is_not_a_sustained_rise(self):
        for direction in (1, -1):
            step, _ = delayed_response(0.45, direction)
            step[2] = tune.replace(step[2], rate_dps=direction * 38)
            self.assertGreater(tune.sustained_rise80(step, direction, 38), 0.5)

    def test_response_objective_rejects_overshoot_and_stability_regression(self):
        step, tail = delayed_response(0.12)
        fast = tune.measure(step, tail, 1)
        overshoot = tune.replace(fast, overshoot_fraction=0.2)
        self.assertFalse(overshoot.response_ready)
        baseline = tune.aggregate([fast] * 2)
        invalid: tune.TrialResult = {**baseline, "response_score": 0, "rise80_s": 0,
                                    "stable": False, "response_ready": False}
        self.assertFalse(tune.validated_improvement(baseline, invalid, "response"))
        self.assertFalse(tune.better(invalid, baseline, "response"))

    def test_response_improvement_must_include_faster_rise(self):
        step, tail = delayed_response(0.12)
        baseline = tune.aggregate([tune.measure(step, tail, 1)] * 2)
        candidate: tune.TrialResult = {**baseline, "response_score": 0}
        self.assertFalse(tune.validated_improvement(baseline, candidate, "response"))

    def test_response_search_preserves_integral_and_does_not_lower_proportional_gain(self):
        baseline = tune.Gains(75, 0, 30, 15)
        stages = tune.search_stages("response", baseline)
        self.assertEqual(stages, (("p", (90, 110, 130, 150)), ("f", (60, 120)),
                                  ("hold", (30,))))
        self.assertNotIn("i", [field for field, _ in stages])
        self.assertEqual(tune.search_stages("response", tune.Gains(150, 0, 120, 30)),
                         (("p", ()), ("f", ()), ("hold", ())))


class YawPersistenceTest(unittest.TestCase):
    def test_parse_profile_and_all_required_settings(self):
        text = "profile 0\np_yaw = 45\ni_yaw = 60\nf_yaw = 60\n" \
               "feedforward_yaw_hold_gain = 15\nap_hover_throttle = 1590\n"
        self.assertEqual(tune.parse_settings(text), (0, tune.Gains(45, 60, 60, 15), 1590))
        with self.assertRaises(ValueError):
            tune.parse_settings(text.replace("f_yaw = 60\n", ""))
        self.assertEqual(tune.parse_settings(text.replace("p_yaw = 45", "p_yaw = 150"))[1].p, 150)
        with self.assertRaises(ValueError):
            tune.parse_settings(text.replace("p_yaw = 45", "p_yaw = 151"))

    def test_gain_commands_do_not_touch_other_axes_or_debug_settings(self):
        lines = tune.Gains(55, 20, 30, 0).lines()
        self.assertEqual(lines, ["set p_yaw = 55", "set i_yaw = 20",
                                 "set f_yaw = 30", "set feedforward_yaw_hold_gain = 0"])

    def test_external_eeprom_change_blocks_apply(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "eeprom.bin"
            source.write_bytes(b"changed")
            with patch.object(tune.control, "require_free_ports"), \
                    self.assertRaisesRegex(RuntimeError, "changed during"):
                tune.apply_best(Path("binary"), source, "old-digest", root / "apply",
                                0, tune.Gains(55, 20, 30, 0))
            self.assertEqual(source.read_bytes(), b"changed")

    def test_verification_failure_does_not_overwrite_original(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "eeprom.bin"
            source.write_bytes(b"original")
            desired = tune.Gains(55, 20, 30, 0)
            with patch.object(tune.control, "require_free_ports"), \
                    patch.object(tune.control, "provision"), \
                    patch.object(tune, "inspect", return_value=(0, tune.Gains(45, 60, 60, 15), 1590)), \
                    self.assertRaisesRegex(RuntimeError, "verification"):
                tune.apply_best(Path("binary"), source, tune.digest(source), root / "apply", 0, desired)
            self.assertEqual(source.read_bytes(), b"original")

    def test_successful_apply_replaces_only_verified_trial_eeprom(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "eeprom.bin"
            source.write_bytes(b"original")
            desired = tune.Gains(55, 20, 30, 0)
            def write_trial(_binary, trial, _lines, _name):
                (trial / "eeprom.bin").write_bytes(b"verified")
            with patch.object(tune.control, "require_free_ports"), \
                    patch.object(tune.control, "provision", side_effect=write_trial), \
                    patch.object(tune, "inspect", return_value=(0, desired, 1590)):
                tune.apply_best(Path("binary"), source, tune.digest(source), root / "apply", 0, desired)
            self.assertEqual(source.read_bytes(), b"verified")
            self.assertFalse(list(root.glob(".yaw-eeprom-*")))


if __name__ == "__main__":
    unittest.main()
