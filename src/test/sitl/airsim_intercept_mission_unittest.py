#!/usr/bin/env python3
"""Offline mission regressions; run in the sourced ROS 2/AirSim environment."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

import cosysairsim as airsim
import airsim_balloon_target as balloon
import airsim_intercept_mission as mission


class InterceptMissionTest(unittest.TestCase):
    def make_mission(self):
        return mission.InterceptMission(SimpleNamespace())

    def test_mission_provisions_the_verified_sitl_rate_profile(self):
        self.assertIn("profile 0", mission.SITL_CONFIG_BASE)
        self.assertIn("set simplified_pids_mode = OFF", mission.SITL_CONFIG_BASE)
        for axis, gains in (("roll", (37, 62, 22, 95)),
                            ("pitch", (32, 57, 22, 84)),
                            ("yaw", (10, 0, 0, 8))):
            for term, gain in zip("pidf", gains):
                self.assertIn(f"set {term}_{axis} = {gain}", mission.SITL_CONFIG_BASE)

    def test_rate_profile_is_verified_from_firmware_not_only_cli_text(self):
        test = self.make_mission()
        test.msp_request = Mock(return_value=bytes([37, 62, 22, 32, 57, 22, 10, 0, 0]))
        test.verify_rate_profile()
        self.assertEqual(test.report["flight_controller_rate_pid"]["yaw"], {"p": 10, "i": 0, "d": 0})
        test.msp_request.return_value = bytes([45, 80, 30, 47, 84, 34, 45, 80, 0])
        with self.assertRaisesRegex(RuntimeError, "rate profile did not load"):
            test.verify_rate_profile()

    @patch.object(mission.time, "sleep")
    @patch.object(mission.socket, "create_connection")
    @patch.object(mission.airsim, "MultirotorClient")
    def test_rpc_startup_retries_with_fresh_client(self, client_class, connect, _sleep):
        first, ready = Mock(), Mock()
        client_class.side_effect = [first, ready]
        connect.side_effect = [ConnectionRefusedError("not listening"), MagicMock()]
        ready.listVehicles.return_value = [mission.VEHICLE, mission.TARGET]
        client = mission.connect_airsim("127.0.0.1", 41451, 5)
        self.assertIs(client, ready)
        first.client.close.assert_called_once()
        client_class.assert_called_with(ip="127.0.0.1", port=41451, timeout_value=5)

    @patch.object(mission.time, "sleep")
    @patch.object(mission.time, "monotonic", side_effect=[0, 0, 2])
    @patch.object(mission.socket, "create_connection", side_effect=ConnectionRefusedError("not listening"))
    @patch.object(mission.airsim, "MultirotorClient")
    def test_rpc_startup_timeout_names_endpoint_and_reason(self, client_class, *_mocks):
        with self.assertRaisesRegex(RuntimeError, "41451 not ready.*not listening"):
            mission.connect_airsim("127.0.0.1", 41451, 1)
        client_class.return_value.client.close.assert_called_once()

    @patch.object(mission.socket, "create_connection")
    @patch.object(mission.airsim, "MultirotorClient")
    def test_wrong_airSim_world_is_not_retried_as_a_connection_error(self, client_class, _connect):
        client_class.return_value.listVehicles.return_value = ["Copter"]
        with self.assertRaisesRegex(RuntimeError, "missing required vehicles"):
            mission.connect_airsim("127.0.0.1", 41451, 1)
        self.assertEqual(client_class.call_count, 1)

    def test_child_failure_surfaces_balloon_error_and_log_path(self):
        process = mission.ManagedProcess("balloon", [])
        process.process = Mock(returncode=1)
        process.process.poll.return_value = 1
        process.log_path = Mock()
        process.log_path.exists.return_value = True
        process.log_path.read_text.return_value = "Traceback\nRuntimeError: balloon collision with Tree"
        with self.assertRaisesRegex(RuntimeError, "balloon collision with Tree"):
            process.wait_for_log("standoff reached", 5)

    def test_stale_ros_graph_cannot_hide_a_crashed_node(self):
        process = mission.ManagedProcess("interceptor", [])
        process.process = Mock()
        process.process.poll.return_value = None
        process.log_path = Mock()
        process.log_path.exists.return_value = True
        process.log_path.read_text.return_value = "[ERROR] process has died: invalid parameter type"
        with self.assertRaisesRegex(RuntimeError, "ROS node died"):
            process.wait_for_log("Relative IMM-UKF node ready", 5)

    def test_camera_view_guard_converts_enu_to_ned(self):
        # ENU heading 90 deg points north; pitch -30 deg points 30 deg up.
        q = mission.quaternion_rpy(0, math.radians(-30), math.pi / 2)
        orientation = (q.x_val, q.y_val, q.z_val, q.w_val)
        target = (math.cos(math.pi / 6), 0, -math.sin(math.pi / 6))
        self.assertAlmostEqual(mission.camera_target_error_degrees(orientation, target), 0, places=5)
        self.assertAlmostEqual(mission.camera_target_error_degrees(orientation, (-1, 0, 0)), 150)

    @patch.object(mission.rclpy, "shutdown")
    @patch.object(mission.time, "sleep")
    @patch.object(mission, "park_target", side_effect=RuntimeError("cannot release target"))
    def test_cleanup_failure_makes_report_fail(self, _park, _sleep, _shutdown):
        test = self.make_mission()
        test.stage = "balloon"
        test.report["passed"] = True
        test.world_ready = True
        test.balloon = Mock()
        test.client = Mock()
        test.teardown()
        self.assertFalse(test.report["passed"])
        self.assertIn("cannot release target", test.report["error"])
        test.client.client.close.assert_called_once()

    def test_enu_heading_is_converted_to_clockwise_north_heading(self):
        test = self.make_mission()
        for heading in (0, 31, -31, 90, 179, -179):
            with self.subTest(heading=heading):
                a = math.radians(90 - heading) / 2
                test.ros_iface = SimpleNamespace(imu_quat=(0, 0, math.sin(a), math.cos(a)))
                self.assertAlmostEqual(test.imu_yaw_math(), heading)

    def test_distance_uses_common_world_not_vehicle_spawn_coordinates(self):
        test = self.make_mission()
        test.client = Mock()
        test.client.simGetGroundTruthKinematics.return_value = SimpleNamespace(
            position=airsim.Vector3r(52, 36, -43.5), linear_velocity=airsim.Vector3r(),
            orientation=airsim.Quaternionr(0, 0, 0, 1))
        test.client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(60, 36, -45))
        self.assertEqual(test.gt(mission.TARGET)["pos"], (60, 36, -45))

    def test_invalid_object_pose_is_not_a_successful_distance_sample(self):
        test = self.make_mission()
        test.client = Mock()
        test.client.simGetGroundTruthKinematics.return_value = SimpleNamespace(
            linear_velocity=airsim.Vector3r(), orientation=airsim.Quaternionr(0, 0, 0, 1))
        test.client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(math.nan, 0, 0))
        with self.assertRaisesRegex(RuntimeError, "invalid AirSim ground truth"):
            test.gt()

    def test_return_flythrough_does_not_relabel_mission_closest_approach(self):
        test = self.make_mission()
        test.stage = "rth"
        test.min_distance = 2.0
        test.t0 = 0
        test.gt = Mock(side_effect=[
            {"pos": (0, 0, 0), "vel": (0, 0, 0), "rpy": (0, 0, 0)},
            {"pos": (0.5, 0, 0), "vel": (0, 0, 0), "rpy": (0, 0, 0)}])
        test.modes = Mock(return_value=([0, 3, 11, 56], 0))
        self.assertEqual(test.sample()["distance"], 0.5)
        self.assertEqual(test.min_distance, 2.0)

    def test_trajectory_records_fc_attitude_without_changing_world_scoring(self):
        test = self.make_mission()
        test.stage = "mission"
        test.t0 = 0
        test.ros_iface = SimpleNamespace(
            target_los=None, target_bbox=None, tracker_state=2, mission_state=True,
            imu_quat=(0, 0, math.sqrt(.5), math.sqrt(.5)))
        test.gt = Mock(side_effect=[
            {"pos": (0, 0, 0), "vel": (0, 0, 0), "rpy": (0, 0, 0)},
            {"pos": (.5, 0, 0), "vel": (0, 0, 0), "rpy": (0, 0, 0)}])
        test.modes = Mock(return_value=([0], 0))
        test.trajectory = Mock()
        self.assertEqual(test.sample()["distance"], .5)
        self.assertEqual(test.trajectory.writerow.call_args.args[0][-4:],
                         test.ros_iface.imu_quat)

    def test_hover_handover_brakes_vertical_motion(self):
        test = self.make_mission()
        test.origin = (0, 0, 0)
        test.publish_hover_command = Mock()
        for vz, expected in ((0, 9.81), (-3, 4), (3, 14)):
            with self.subTest(vz=vz):
                test.hold_launch_hover({"pos": (0, 0, -4), "vel": (0, 0, vz)})
                self.assertAlmostEqual(test.publish_hover_command.call_args.args[0], expected)

    def test_success_requires_hit_return_and_no_error(self):
        passed = {"intercept": {"outcome": "hit"}, "rth": {"outcome": "returned"}}
        self.assertTrue(mission.mission_passed(passed, None))
        self.assertFalse(mission.mission_passed(passed, RuntimeError("cleanup failed")))
        self.assertFalse(mission.mission_passed({"intercept": {"outcome": "hit"}}, None))
        self.assertFalse(mission.mission_passed(
            {"intercept": {"outcome": "lost"}, "rth": {"outcome": "returned"}}, None))
        self.assertFalse(mission.mission_passed(
            {"intercept": {"outcome": "hit"}, "rth": {"outcome": "failed"}}, None))

    def prepare_intercept(self, samples, mission_active=True):
        test = self.make_mission()
        test.origin = (0, 0, 0)
        test.ros_iface = Mock(tracker_state=2, mission_state=mission_active, control_sequence=0)
        test.rc = Mock()
        test.client = Mock()
        test.client.getMultirotorState.return_value = SimpleNamespace(timestamp=1)
        test.client.simGetCollisionInfo.return_value = SimpleNamespace(has_collided=False)
        samples = iter(samples)

        def sample():
            state = {"pos": (0, 0, -3), "rpy": (0, 0, 0), **next(samples)}
            test.last_sample = state
            test.min_distance = min(test.min_distance or math.inf, state["distance"])
            return state

        test.sample = sample
        return test

    @patch.object(mission.time, "sleep")
    def test_intercept_report_separates_measured_speed_and_climb(self, _sleep):
        test = self.prepare_intercept([
            {"vel": (12, 5, -20), "distance": 10},
            {"vel": (3, 4, 7), "distance": .75}])
        result = test.run_intercept()
        self.assertEqual(result["outcome"], "hit")
        self.assertEqual(result["closest_distance_m"], .75)
        self.assertAlmostEqual(result["peak_speed_m_s"], math.sqrt(569))
        self.assertEqual(result["peak_horizontal_speed_m_s"], 13)
        self.assertEqual(result["peak_climb_speed_m_s"], 20)

    @patch.object(mission.time, "sleep")
    def test_launcher_stows_only_after_five_meter_separation(self, _sleep):
        test = self.prepare_intercept([
            {"pos": (1, 0, -2), "vel": (0, 0, 0), "distance": 10},
            {"pos": (4, 0, -4), "vel": (0, 0, 0), "distance": .75}])

        def stow():
            self.assertGreater(math.dist(test.last_sample["pos"], test.origin), 5)
            test.launcher.stowed = True

        test.launcher = SimpleNamespace(released=True, stowed=False, stow=Mock(side_effect=stow))
        test.run_intercept()
        test.launcher.stow.assert_called_once()

    def test_close_miss_is_not_rounded_to_a_hit(self):
        test = self.prepare_intercept(
            [{"vel": (0, 0, 0), "distance": 1.04}], mission_active=False)
        result = test.run_intercept()
        self.assertEqual(result["outcome"], "lost")
        self.assertEqual(result["closest_distance_m"], 1.04)

    def test_non_target_collision_emergency_is_preserved(self):
        test = self.prepare_intercept([{"vel": (0, 0, 0), "distance": 10}])
        test.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=True, time_stamp=2, object_name="Landscape_1")
        with self.assertRaisesRegex(RuntimeError, "non-target collision"):
            test.run_intercept()
        self.assertTrue(test.hard_collision)
        test.ros_iface.publish_offboard.assert_called_once_with(False)

    def test_target_contact_alone_does_not_override_distance_threshold(self):
        test = self.prepare_intercept(
            [{"vel": (0, 0, 0), "distance": 2}], mission_active=False)
        test.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=True, time_stamp=2, object_name=mission.TARGET)
        self.assertEqual(test.run_intercept()["outcome"], "lost")
        self.assertFalse(test.hard_collision)

    @patch.object(mission.time, "sleep")
    def test_range_reopening_ends_failed_pass_before_blind_turn(self, _sleep):
        test = self.prepare_intercept([
            {"vel": (0, 0, 0), "distance": 3},
            {"vel": (0, 0, 0), "distance": 12}])
        result = test.run_intercept()
        self.assertEqual(result["outcome"], "opening_range")
        self.assertEqual(result["closest_distance_m"], 3)
        self.assertFalse(test.hard_collision)

    def prepare_return(self, state, contact_time):
        test = self.make_mission()
        test.origin = (100, -30, -3)
        test.airborne_timestamp = 100
        test.ros_iface = Mock()
        test.rc = Mock()
        test.client = Mock()
        test.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=True, time_stamp=contact_time, normal=SimpleNamespace(z_val=-1),
            position=airsim.Vector3r(100, -30, -3), object_name="Landscape_1")
        test.wait = Mock(return_value={"modes": [0, 3, 11, 56]})
        test.sample = Mock(return_value=state)
        return test

    @patch.object(mission.time, "sleep")
    def test_return_rejects_disarm_aloft_even_at_home(self, _sleep):
        test = self.prepare_return(
            {"pos": (100, -30, -43), "vel": (0, 0, 0), "modes": []}, 200)
        with self.assertRaisesRegex(RuntimeError, "disarmed before home landing"):
            test.return_home()

    @patch.object(mission.time, "sleep")
    def test_return_requires_new_ground_contact(self, _sleep):
        test = self.prepare_return(
            {"pos": (100, -30, -3), "vel": (0, 0, 0), "modes": []}, 99)
        with self.assertRaisesRegex(RuntimeError, "disarmed before home landing"):
            test.return_home()

    @patch.object(mission.time, "sleep")
    def test_return_accepts_settled_ground_landing(self, _sleep):
        test = self.prepare_return(
            {"pos": (100.1, -30.1, -3), "vel": (0, 0, 0), "modes": []}, 200)
        test.return_home()
        self.assertEqual(test.report["phases"]["rth"]["outcome"], "returned")
        self.assertTrue(test.report["phases"]["rth"]["ground_contact"])
        test.ros_iface.publish_offboard.assert_called_once_with(False)

    @patch.object(mission.time, "sleep")
    def test_target_contact_aloft_cannot_count_as_home_ground_contact(self, _sleep):
        test = self.prepare_return(
            {"pos": (100, -30, -3), "vel": (0, 0, 0), "modes": []}, 200)
        test.client.simGetCollisionInfo.return_value.position = airsim.Vector3r(170, -30, -43)
        with self.assertRaisesRegex(RuntimeError, "disarmed before home landing"):
            test.return_home()

    @patch.object(balloon.airsim, "MultirotorClient")
    def test_balloon_world_standoff_is_converted_to_vehicle_local_coordinates(self, client_class):
        client = client_class.return_value
        client.simGetVehiclePose.return_value = airsim.Pose(airsim.Vector3r())
        client.simGetObjectPose.return_value = airsim.Pose(airsim.Vector3r(8, 0, -1.5))
        client.getSettingsString.return_value = "{}"
        client.isApiControlEnabled.side_effect = [True, False]
        client.simGetGroundTruthKinematics.return_value = airsim.KinematicsState()
        target = balloon.BalloonTarget(SimpleNamespace(
            host="127.0.0.1", rpc_port=41451, x=60, y=36, z=-45))
        self.assertEqual(target.local_standoff, (52, 36, -43.5))

    def make_balloon(self):
        target = balloon.BalloonTarget.__new__(balloon.BalloonTarget)
        target.args = SimpleNamespace(scale=1, cruise=8, reset=True, motion="hover")
        target.client = Mock()
        target.stop = Mock()
        target.stop.is_set.return_value = False
        target.standoff = (60, 36, -45)
        target.local_standoff = (52, 36, -43.5)
        target.spawn_offset = (8, 0, -1.5)
        target.flight_timestamp = 100
        target.geo_origin = SimpleNamespace(latitude=47.641468, longitude=-122.140165, altitude=122)
        target.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=False, time_stamp=0)
        target.client.isApiControlEnabled.side_effect = [True, False]
        target.client.simGetGroundTruthKinematics.return_value = airsim.KinematicsState()
        return target

    def test_balloon_rpc_task_errors_are_not_discarded_by_join(self):
        target = self.make_balloon()
        target.client.moveByVelocityAsync.return_value.get.side_effect = mission.RpcTimeoutError("timeout")
        with self.assertRaises(mission.RpcTimeoutError):
            target.command_velocity((0, 0, -1))
        target.client.moveByVelocityAsync.return_value.join.assert_not_called()

    def test_balloon_false_task_result_is_not_success(self):
        target = self.make_balloon()
        target.client.moveByVelocityAsync.return_value.get.return_value = False
        with self.assertRaisesRegex(RuntimeError, "failed/cancelled"):
            target.command_velocity((0, 0, -1))

    @patch.object(balloon.time, "monotonic", side_effect=[0, 0, 0, 10])
    def test_balloon_stall_ends_without_repeated_blind_moves(self, _clock):
        target = self.make_balloon()
        target.vehicle_position = Mock(return_value=(8, 0, -1.5))
        target.client.simGetGroundTruthKinematics.return_value = airsim.KinematicsState()
        with self.assertRaisesRegex(RuntimeError, "stalled en route"):
            target.fly_leg(target.standoff)
        target.client.moveByVelocityAsync.assert_not_called()

    @patch.object(balloon.time, "monotonic", side_effect=range(0, 100))
    def test_balloon_near_goal_must_be_stationary_before_ready(self, _clock):
        target = self.make_balloon()
        target.client.simGetGroundTruthKinematics.return_value = SimpleNamespace(
            linear_velocity=airsim.Vector3r(3, 0, 0))
        target.vehicle_position = Mock(return_value=target.standoff)
        with self.assertRaisesRegex(RuntimeError, "did not settle at waypoint"):
            target.fly_leg(target.standoff)

    def test_balloon_collision_reports_actor_and_stops_commands(self):
        target = self.make_balloon()
        target.vehicle_position = Mock(return_value=(8, 0, -1.5))
        target.client.simGetGroundTruthKinematics.return_value = airsim.KinematicsState()
        target.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=True, time_stamp=101, object_name="Tree")
        with self.assertRaisesRegex(RuntimeError, "collision with Tree"):
            target.fly_leg(target.standoff)
        target.client.moveByVelocityAsync.assert_not_called()

    def test_obstructed_balloon_staging_chooses_checked_alternative(self):
        target = self.make_balloon()
        target.corridor_clear = Mock(side_effect=[False, True])
        route = target.plan_route((0, 5, -5))
        self.assertEqual(route[0], (0, -3, -5))
        self.assertEqual(route[1], (0, -3, -45))
        self.assertEqual(route[-1], target.standoff)

    def test_fully_obstructed_scene_does_not_arm_balloon(self):
        target = self.make_balloon()
        target.corridor_clear = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "no obstacle-clear"):
            target.plan_route((0, 5, -5))
        target.client.armDisarm.assert_not_called()

    def test_balloon_reset_zeros_kinematics_and_always_applies_scale(self):
        target = self.make_balloon()
        target.args.spawn_x, target.args.spawn_y, target.args.spawn_z = (20, 30, -5)
        target.route = [(20, 30, -5), (20, 30, -45), target.standoff]
        target.plan_route = Mock(return_value=target.route)
        target.vehicle_position = Mock(return_value=target.route[0])
        with patch.object(balloon, "park_target"), patch.object(balloon.time, "sleep"):
            target.reset_to_spawn()
        state = target.client.simSetKinematics.call_args.args[0]
        self.assertEqual((state.position.x_val, state.position.y_val, state.position.z_val),
                         (12, 30, -3.5))
        self.assertEqual(state.linear_velocity.get_length(), 0)
        target.client.simSetVehiclePose.assert_not_called()
        target.client.simSetObjectScale.assert_called_once()

    def test_balloon_deployment_failure_still_cancels_native_task(self):
        target = self.make_balloon()
        target.reset_to_spawn = Mock()
        target.fly_to_standoff = Mock(side_effect=RuntimeError("blocked"))
        with self.assertRaisesRegex(RuntimeError, "blocked"):
            target.run()
        target.client.cancelLastTask.assert_called_once_with(vehicle_name=balloon.TARGET_NAME)
        target.client.enableApiControl.assert_called_with(False, vehicle_name=balloon.TARGET_NAME)
        target.client.moveToPositionAsync.assert_not_called()
        target.client.client.close.assert_called_once()

    def test_visibility_conversion_preserves_wgs84_local_frame(self):
        origin = SimpleNamespace(latitude=47.641468, longitude=-122.140165, altitude=122)
        point = balloon.world_to_geopoint((0, 0, 0), origin)
        self.assertAlmostEqual(point.latitude, origin.latitude, places=8)
        self.assertAlmostEqual(point.longitude, origin.longitude, places=8)
        self.assertAlmostEqual(point.altitude, origin.altitude, places=5)
        point = balloon.world_to_geopoint((100, 0, -50), origin)
        self.assertGreater(point.latitude, origin.latitude)
        self.assertAlmostEqual(point.longitude, origin.longitude, places=8)
        self.assertAlmostEqual(point.altitude, 172, delta=.01)


if __name__ == "__main__":
    unittest.main()
