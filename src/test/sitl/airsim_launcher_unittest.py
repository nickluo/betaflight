#!/usr/bin/env python3
"""Offline Camera8 launcher and monotonic-vision regression checks."""

import math
import itertools
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

import cosysairsim as airsim
import numpy as np
from builtin_interfaces.msg import Time

from airsim_launcher import AirsimLauncher, launcher_angles, move_actor, quaternion_rpy, rotate
from airsim_tracker.airsim_tracker_node import AirSimTrackerNode
from airsim_tracker.target_los import TargetLosEstimator


class LauncherTest(unittest.TestCase):
    def test_minus_ten_aim_aligns_camera_through_platform_pitch(self):
        position, target = (8, 0, -2), (78, 0, -47)
        yaw, pitch, elevation = launcher_angles(position, target, -10)
        camera_axis = rotate(quaternion_rpy(0, pitch, yaw),
                             rotate(quaternion_rpy(0, math.radians(80), 0), (1, 0, 0)))
        delta = tuple(t-p for p, t in zip(position, target))
        expected = tuple(v/math.hypot(*delta) for v in delta)
        np.testing.assert_allclose(camera_axis, expected, atol=1e-8)
        self.assertAlmostEqual(pitch, elevation-math.radians(80))

    def test_unreachable_launcher_tilt_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "beyond 65"):
            launcher_angles((0, 0, 0), (70, 0, 0), -10)

    def test_verified_unchanged_ue_transform_is_not_an_actuator_failure(self):
        client = Mock()
        pose = airsim.Pose(airsim.Vector3r(1, 2, 3), quaternion_rpy(0, .5, 0))
        client.simSetObjectPose.return_value = False
        client.simGetObjectPose.return_value = pose
        move_actor(client, "BFLauncher_Tray", pose)

    def test_failed_ue_transform_cannot_be_hidden_as_a_noop(self):
        client = Mock()
        client.simSetObjectPose.return_value = False
        client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(20, 0, 0))
        with self.assertRaisesRegex(RuntimeError, "did not reach"):
            move_actor(client, "BFLauncher_Tray", airsim.Pose(airsim.Vector3r()))

    def make_launcher(self):
        client = Mock()
        client.simListSceneObjects.return_value = []
        client.simListAssets.return_value = ["EditorCube"]
        client.simGetObjectPose.return_value = airsim.Pose(
            airsim.Vector3r(0, 0, 0), quaternion_rpy(0, 0, math.pi/2))
        client.simGetVehiclePose.return_value = airsim.Pose(airsim.Vector3r())
        client.simSpawnObject.side_effect = lambda name, *args: name
        client.simRunConsoleCommand.return_value = True
        return AirsimLauncher(client, "127.0.0.1", 41451), client

    def test_spawn_is_relative_to_camera_heading_and_mesh_size_is_normalized(self):
        launcher, client = self.make_launcher()
        self.assertAlmostEqual(launcher.ground_xy[0], 0)
        self.assertAlmostEqual(launcher.ground_xy[1], 8)
        launcher.build(0)
        call = client.simSpawnObject.call_args_list[0]
        self.assertEqual(call.args[1], "EditorCube")
        self.assertAlmostEqual(call.args[3].x_val, 2.2/2.56)
        self.assertTrue(launcher.pinned)
        client.simRunConsoleCommand.assert_called_with("airsim.LauncherLock.Copter 1")

    def test_release_restores_physics_before_reporting_released(self):
        launcher, client = self.make_launcher()
        launcher.build(0)
        launcher.release()
        client.simRunConsoleCommand.assert_called_with("airsim.LauncherLock.Copter 0")
        self.assertFalse(launcher.pinned)
        self.assertTrue(launcher.released)

    def test_missing_physical_constraint_prevents_mounted_operation(self):
        launcher, client = self.make_launcher()
        client.simRunConsoleCommand.return_value = False
        with self.assertRaisesRegex(RuntimeError, "no virtual launcher constraint"):
            launcher.build(0)
        client.simSetKinematics.assert_not_called()

    def test_stowed_landing_saddle_is_centered_under_launch_home(self):
        launcher, client = self.make_launcher()
        launcher.build(0)
        launcher.position = (.4, 8.2, -2)
        launcher.stow()
        name, pose = client.simSetObjectPose.call_args.args[:2]
        self.assertEqual(name, "BFLauncher_Saddle")
        self.assertEqual((pose.position.x_val, pose.position.y_val), launcher.position[:2])
        self.assertEqual(pose.orientation.x_val, 0)
        self.assertEqual(pose.orientation.y_val, 0)
        self.assertTrue(launcher.stowed)

    def test_spawn_frame_is_remeasured_after_reset_fall_settles(self):
        launcher, client = self.make_launcher()
        client.simGetVehiclePose.return_value = airsim.Pose(airsim.Vector3r(8, 4, -.2))
        client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(8, 4, 0))
        launcher.refresh_spawn_offset()
        self.assertAlmostEqual(launcher.spawn_offset[2], .2)
        client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(8, 4, -.2))
        launcher.refresh_spawn_offset()
        self.assertEqual(launcher.spawn_offset, (0, 0, 0))

    @patch("airsim_launcher.time.monotonic", side_effect=itertools.count(1, .05))
    def test_launcher_tracks_moving_target_without_changing_fixed_camera_aim(self, _clock):
        launcher, client = self.make_launcher()
        launcher.build(0)
        for y in range(120):
            launcher.aim_at((70, 8 + y * .02, -45))
        first_yaw, first_pitch = launcher.yaw, launcher.pitch
        for y in range(120):
            launcher.aim_at((70, 12 - y * .02, -43))
        self.assertNotEqual(launcher.yaw, first_yaw)
        self.assertNotEqual(launcher.pitch, first_pitch)
        desired, *_ = launcher_angles(launcher.position, (70, 9.62, -43), -10)
        self.assertLess(abs(launcher.yaw - desired), .05)
        self.assertEqual(launcher.aim_pitch_deg, -10)
        launcher.release()
        client.simSetKinematics.reset_mock()
        with self.assertRaisesRegex(RuntimeError, "after release"):
            launcher.aim_at((70, 20, -45))
        client.simSetKinematics.assert_not_called()


class MonotonicDetectionTest(unittest.TestCase):
    def test_capture_time_publication_does_not_use_latest_gimbal_prediction(self):
        tracker = SimpleNamespace(PUBLISH_PREDICTION=False)
        rotation = np.eye(3)
        stamp, actual = AirSimTrackerNode.prediction_reference(tracker, 10, rotation)
        self.assertEqual(stamp, 10)
        self.assertIs(actual, rotation)

    def test_prediction_mode_keeps_existing_fresh_gimbal_reference(self):
        rotation = np.eye(3)
        tracker = SimpleNamespace(
            PUBLISH_PREDICTION=True, camera_lock=MagicMock(), camera_history=[(9.9, rotation)],
            get_clock=Mock(return_value=SimpleNamespace(
                now=Mock(return_value=SimpleNamespace(nanoseconds=10_000_000_000)))))
        stamp, actual = AirSimTrackerNode.prediction_reference(tracker, 9.8, np.eye(3))
        self.assertEqual(stamp, 9.9)
        self.assertIs(actual, rotation)

    def make_tracker(self):
        return SimpleNamespace(
            last_detection_stamp_ns=0, detection_publisher=Mock(), los_publisher=Mock(),
            get_logger=Mock(return_value=Mock()),
            los_estimator=TargetLosEstimator(1000, (768, 480)))

    def test_duplicate_or_older_predictions_do_not_reset_valid_los(self):
        tracker = self.make_tracker()
        publish = AirSimTrackerNode.publish_detections
        publish(tracker, (760, 472, 16, 16), Time(sec=10), np.eye(3))
        first = tracker.los_estimator.previous
        for sec in (10, 9):
            publish(tracker, (760, 472, 16, 16), Time(sec=sec), np.eye(3))
        self.assertEqual(tracker.detection_publisher.publish.call_count, 1)
        self.assertIs(tracker.los_estimator.previous, first)
        publish(tracker, (760, 472, 16, 16), Time(sec=11), np.eye(3))
        self.assertEqual(tracker.detection_publisher.publish.call_count, 2)
        self.assertEqual(tracker.los_publisher.publish.call_args.args[0].header.frame_id, "camera_flu")

    def test_newer_real_loss_still_invalidates_los(self):
        tracker = self.make_tracker()
        publish = AirSimTrackerNode.publish_detections
        publish(tracker, (760, 472, 16, 16), Time(sec=10), np.eye(3))
        publish(tracker, None, Time(sec=11), None)
        self.assertIsNone(tracker.los_estimator.previous)
        self.assertEqual(tracker.los_publisher.publish.call_args.args[0].header.frame_id, "")


if __name__ == "__main__":
    unittest.main()
