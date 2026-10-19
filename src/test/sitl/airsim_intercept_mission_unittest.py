#!/usr/bin/env python3
"""Offline mission regressions; run in the sourced ROS 2/AirSim environment."""

import math
import itertools
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import ANY, MagicMock, Mock, patch

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

    def test_controller_profile_is_independent_of_balloon_motion(self):
        base = dict(guidance_speed=20.0, climb_speed=20.0, turn_cap_factor=.8,
                    rate_control_attenuation=False)
        hover = {"balloon_motion": "hover", **base}
        orbit = {"balloon_motion": "orbit", **base}
        def parameters():
            return {"controller": {"v_pn_ref": 12.0, "terminal_speed_floor": 11.0,
                                   "visual_speed_floor": 2.6, "lateral_decay_floor": .4},
                    "estimation": {}, "control": {}, "physics": {}}
        hover_params, orbit_params = parameters(), parameters()
        mission.apply_interceptor_profile(hover_params, SimpleNamespace(**hover))
        mission.apply_interceptor_profile(orbit_params, SimpleNamespace(**orbit))
        self.assertEqual(hover_params, orbit_params)
        self.assertEqual(hover_params["controller"]["terminal_speed_floor"], 11.0)
        self.assertEqual(hover_params["controller"]["visual_speed_floor"], 2.6)
        self.assertEqual(hover_params["controller"]["v_des"], 20.0)

    def test_rate_profile_is_verified_from_firmware_not_only_cli_text(self):
        test = self.make_mission()
        test.msp_request = Mock(return_value=bytes([37, 62, 22, 32, 57, 22, 10, 0, 0]))
        test.verify_rate_profile()
        self.assertEqual(test.report["flight_controller_rate_pid"]["yaw"], {"p": 10, "i": 0, "d": 0})
        test.msp_request.return_value = bytes([45, 80, 30, 47, 84, 34, 45, 80, 0])
        with self.assertRaisesRegex(RuntimeError, "rate profile did not load"):
            test.verify_rate_profile()

    def test_arm_switch_stays_low_until_firmware_checks_clear(self):
        test = self.make_mission()
        test.rc = Mock()
        test.ros_iface = Mock()
        calls = []

        def wait(description, predicate, **kwargs):
            calls.append(description)
            if len(calls) == 1:
                self.assertEqual(test.rc.set.call_args.kwargs["arm"], mission.joystick.RC_MIN_US)
                self.assertFalse(predicate({"flags": 0x200}))
                self.assertFalse(predicate({"flags": 1 << 29}))
                self.assertTrue(predicate({"flags": 0}))
            else:
                self.assertEqual(test.rc.set.call_args.kwargs["arm"], mission.joystick.RC_MAX_US)
                self.assertTrue(predicate({"modes": [0]}))
            return {"modes": [0]}

        test.wait = wait
        test.arm_fc()
        self.assertEqual(len(calls), 2)
        self.assertTrue(test.armed_once)
        test.ros_iface.trigger_seen.assert_called_once_with("ON", ANY)

    @patch.object(mission.time, "sleep")
    @patch.object(mission.time, "monotonic", side_effect=[0, 10, 5, 20])
    def test_arm_without_bridge_auto_on_trigger_fails(self, _clock, _sleep):
        test = self.make_mission()
        test.rc = Mock()
        test.ros_iface = Mock()
        test.ros_iface.trigger_seen.return_value = False
        test.wait = Mock(return_value={"modes": [0]})
        with self.assertRaisesRegex(RuntimeError, "auto-publish /fpv/tracker_trigger ON"):
            test.arm_fc()
        test.ros_iface.trigger_seen.assert_called_with("ON", ANY)

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

    def test_target_process_exit_cannot_be_mistaken_for_static_target(self):
        test = self.make_mission()
        test.balloon = Mock()
        test.balloon.check_running.side_effect = RuntimeError("balloon motion collision")
        test.gt = Mock()
        with self.assertRaisesRegex(RuntimeError, "balloon motion collision"):
            test.sample()
        test.gt.assert_not_called()

    @patch.object(mission, "ManagedProcess")
    def test_runner_passes_motion_parameters_and_rotates_center_with_camera(self, process_class):
        test = self.make_mission()
        test.args = SimpleNamespace(
            host="127.0.0.1", rpc_port=41451, balloon_script=Path("airsim_balloon_target.py"),
            balloon_x=70, balloon_y=0, balloon_z=-45, balloon_motion="orbit",
            balloon_orbit_radius=5, balloon_orbit_speed=1.5)
        test.origin = (100, 200, -3)
        test.workdir = Path("/tmp/not-created-by-unit-test")
        test.launcher = SimpleNamespace(heading=math.pi / 2)
        test.gt = Mock(return_value={"pos": (100, 270, -48), "vel": (1.5, 0, 0)})
        test.launch_balloon()
        argv = process_class.call_args.args[1]
        for flag, value in (("--motion", "orbit"), ("--orbit-radius", "5"),
                            ("--orbit-speed", "1.5")):
            self.assertEqual(argv[argv.index(flag) + 1], value)
        self.assertAlmostEqual(float(argv[argv.index("--x") + 1]), 100)
        self.assertAlmostEqual(float(argv[argv.index("--y") + 1]), 270)
        process_class.return_value.wait_for_log.assert_called_once_with("target ready", 120)
        self.assertEqual(test.report["phases"]["balloon"]["ready_speed_m_s"], 1.5)

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

    # ------------------------------------------------------------ interactive

    def make_interactive(self, samples, dials, timeout=100, **ros):
        test = self.make_mission()
        test.args = SimpleNamespace(interactive_timeout=timeout)
        test.t0 = 0
        test.origin = (0, 0, 0)
        test.launcher = None
        test.rc = None
        ros.setdefault("tracker_state", 2)
        ros.setdefault("mission_state", False)
        ros.setdefault("rc_in", [1000] * 16)
        ros.setdefault("triggers", [])
        test.ros_iface = Mock(**ros)
        test.ros_iface.trigger_seen.return_value = False
        test.sample = Mock(side_effect=samples)
        test.rth_dial_active = Mock(side_effect=dials)
        test.envelope_exceeded = Mock(return_value=None)
        return test

    def test_interactive_pass_criteria(self):
        self.assertTrue(mission.interactive_passed({}, None))
        self.assertTrue(mission.interactive_passed(
            {"interactive_missions": [{"outcome": "hit"}]}, None))
        self.assertTrue(mission.interactive_passed(
            {"interactive_missions": [{"outcome": "user_rth"}],
             "rth": {"outcome": "returned"}}, None))
        self.assertTrue(mission.interactive_passed(
            {"interactive_missions": [{"outcome": "dismissed"}]}, None))
        self.assertTrue(mission.interactive_passed(
            {"rth": {"outcome": "not-launched-home"}}, None))
        self.assertFalse(mission.interactive_passed(
            {"interactive_missions": [{"outcome": "lost"}]}, None))
        self.assertFalse(mission.interactive_passed(
            {"interactive_missions": [{"outcome": "hit"}, {"outcome": "hit"}],
             "rth": {"outcome": "failed"}}, None))
        self.assertFalse(mission.interactive_passed({}, RuntimeError("session crashed")))

    def test_rc_channel_reads_bridge_mirror_with_staleness(self):
        test = self.make_mission()
        channels = [1000] * 16
        channels[mission.joystick.RC_CHANNEL_INDICES["mode6"]] = 1900
        test.ros_iface = SimpleNamespace(rc_in=channels, rc_in_stamp=time.monotonic())
        self.assertEqual(test.rc_channel("mode6"), 1900)
        self.assertTrue(test.rth_dial_active())
        test.ros_iface.rc_in_stamp = time.monotonic() - 5
        self.assertIsNone(test.rc_channel("mode6"))
        self.assertFalse(test.rth_dial_active())

    @patch.object(mission.time, "sleep")
    def test_interactive_session_flies_arm_mission_rth(self, _sleep):
        test = self.make_interactive(
            samples=[{"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": mode}
                     for mode in ([], [0], [0], [0])],
            dials=[False, False, False, True])
        test.ros_iface.trigger_seen.side_effect = [False, True]
        test.interactive_armed = Mock()
        test.supervise_interactive_mission = Mock()
        test.interactive_return_home = Mock()
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            test.interactive_mission()
        test.interactive_armed.assert_called_once()
        # arm-edge anchor: the time of the last DISarmed sample
        anchor = test.interactive_armed.call_args.args[1]
        self.assertIsInstance(anchor, (int, float))
        test.supervise_interactive_mission.assert_called_once()
        test.interactive_return_home.assert_called_once()

    @patch.object(mission.time, "sleep")
    def test_interactive_disarm_on_ground_ends_the_session(self, _sleep):
        test = self.make_interactive(
            samples=[{"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": []},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": []}],
            dials=[False, False, False])
        test.interactive_armed = Mock()
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            test.interactive_mission()
        self.assertIn("interactive", test.report["phases"])

    @patch.object(mission.time, "sleep")
    def test_interactive_disarm_aloft_fails_the_session(self, _sleep):
        test = self.make_interactive(
            samples=[{"pos": (0, 0, -10), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, -10), "rpy": (0, 0, 0), "modes": []}],
            dials=[False, False])
        test.interactive_armed = Mock()
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            with self.assertRaisesRegex(RuntimeError, "disarmed in flight at 10"):
                test.interactive_mission()

    @patch.object(mission.time, "sleep")
    def test_interactive_pre_session_arming_requires_a_rearm_cycle(self, _sleep):
        # Armed during setup (e.g. while the balloon flies out): the arm-edge
        # ON predates the loop and cannot be verified - only a disarm->arm
        # cycle inside the session runs the verification.
        test = self.make_interactive(
            samples=[{"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": []},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": []}],
            dials=[False, False, False, False])
        test.interactive_armed = Mock()
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            test.interactive_mission()
        test.interactive_armed.assert_called_once()  # the fresh edge only

    @patch.object(mission.time, "sleep")
    def test_interactive_session_times_out(self, _sleep):
        test = self.make_interactive(samples=[], dials=[], timeout=5)
        with patch.object(mission.time, "monotonic", side_effect=[10, 10, 20, 30, 40]):
            with self.assertRaisesRegex(TimeoutError, "timed out"):
                test.interactive_mission()

    @patch.object(mission.time, "sleep")
    def test_interactive_session_ctrl_c_ends_cleanly(self, _sleep):
        test = self.make_interactive(samples=None, dials=[])
        test.sample = Mock(side_effect=KeyboardInterrupt)
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            test.interactive_mission()  # must not raise

    @patch.object(mission.time, "sleep")
    def test_interactive_stop_pulse_restarts_the_tracker_once(self, _sleep):
        off_time = time.monotonic()
        test = self.make_interactive(
            samples=[{"pos": (0, 0, -5), "rpy": (0, 0, 0), "modes": []},
                     {"pos": (0, 0, -5), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, -5), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, -5), "rpy": (0, 0, 0), "modes": [0]},
                     {"pos": (0, 0, 0), "rpy": (0, 0, 0), "modes": []}],
            dials=[False, False, False, False, False], triggers=[(off_time, "OFF")])
        test.interactive_armed = Mock()
        with patch.object(mission.time, "monotonic", side_effect=itertools.count(1)):
            test.interactive_mission()
        test.ros_iface.publish_trigger.assert_called_once_with("ON")

    def test_interactive_armed_requires_bridge_auto_on(self):
        test = self.make_mission()
        test.origin = (0, 0, 0)
        test.launcher = None
        test.wait_for_trigger = Mock(return_value=False)
        with patch.object(mission.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "auto-publish /fpv/tracker_trigger ON"):
                test.interactive_armed({"pos": (0, 0, 0)})
        test.wait_for_trigger = Mock(return_value=True)
        with patch.object(mission.time, "sleep"):
            test.interactive_armed({"pos": (0, 0, 0)})
        self.assertTrue(test.armed_once)

    def test_interactive_armed_anchors_wait_before_the_detection_sample(self):
        # The bridge publishes ON at the physical arm edge, which can precede
        # this loop's slower detection; the anchor must be the last DISarmed
        # sample time, not the detection time.
        test = self.make_mission()
        test.origin = (0, 0, 0)
        test.launcher = None
        test.wait_for_trigger = Mock(return_value=True)
        with patch.object(mission.time, "sleep"):
            test.interactive_armed({"pos": (0, 0, 0)}, since=123.0)
        test.wait_for_trigger.assert_called_once_with("ON", 123.0, 10.0)
        self.assertTrue(test.armed_once)

    @patch.object(mission.time, "sleep")
    def test_interactive_supervise_warns_on_ground_launch_and_releases(self, _sleep):
        test = self.make_interactive(samples=[{"pos": (0, 0, 0), "rpy": (0, 0, 0)}],
                                     dials=[False])
        test.run_intercept = Mock(return_value={"outcome": "hit", "closest_distance_m": .9})
        test.supervise_interactive_mission()
        test.run_intercept.assert_called_once_with(
            publish_mission=False, abort=test.rth_dial_active)
        test.ros_iface.publish_trigger.assert_not_called()
        test.ros_iface.publish_offboard.assert_called_once_with(False)
        self.assertEqual(test.report["phases"]["interactive_missions"],
                         [{"outcome": "hit", "closest_distance_m": .9}])

    @patch.object(mission.time, "sleep")
    def test_interactive_supervise_user_rth_defers_release_to_return_home(self, _sleep):
        test = self.make_interactive(samples=[{"pos": (0, 0, -5), "rpy": (0, 0, 0)}],
                                     dials=[False])
        test.run_intercept = Mock(return_value={"outcome": "user_rth",
                                                "closest_distance_m": 5})
        test.supervise_interactive_mission()
        test.run_intercept.assert_called_once()
        test.ros_iface.publish_offboard.assert_not_called()

    @patch.object(mission.time, "sleep")
    def test_run_intercept_returns_user_rth_when_pilot_dials_home(self, _sleep):
        test = self.prepare_intercept([{"vel": (0, 0, 0), "distance": 10}])
        result = test.run_intercept(abort=Mock(return_value=True))
        self.assertEqual(result["outcome"], "user_rth")
        self.assertEqual(result["closest_distance_m"], 10)

    @patch.object(mission.time, "sleep")
    def test_run_intercept_suppresses_mission_publish_for_native_trigger(self, _sleep):
        test = self.prepare_intercept([{"vel": (0, 0, 0), "distance": .75}])
        result = test.run_intercept(publish_mission=False)
        self.assertEqual(result["outcome"], "hit")
        test.ros_iface.publish_trigger.assert_not_called()

    def test_run_intercept_dismiss_pulse_is_not_scored_as_a_miss(self):
        test = self.prepare_intercept([{"vel": (0, 0, 0), "distance": 10}],
                                      mission_active=False)
        test.ros_iface.trigger_seen = Mock(side_effect=lambda f, since: f == "DISMISS")
        result = test.run_intercept()
        self.assertEqual(result["outcome"], "dismissed")
        self.assertTrue(result["terminated_by_node"])

    @patch.object(mission.time, "sleep")
    def test_return_home_with_pilot_dial_leaves_rc_stream_alone(self, _sleep):
        test = self.prepare_return(
            {"pos": (100.1, -30.1, -3), "vel": (0, 0, 0), "modes": []}, 200)
        test.return_home(dial_engaged=True)
        test.rc.set.assert_not_called()
        test.rc.set_mode.assert_not_called()
        self.assertEqual(test.report["phases"]["rth"]["outcome"], "returned")
        test.ros_iface.publish_trigger.assert_called_once_with("OFF")
        test.ros_iface.publish_offboard.assert_called_once_with(False)

    def test_non_target_collision_emergency_works_without_script_rc(self):
        test = self.prepare_intercept([{"vel": (0, 0, 0), "distance": 10}])
        test.rc = None
        test.client.simGetCollisionInfo.return_value = SimpleNamespace(
            has_collided=True, time_stamp=2, object_name="Landscape_1")
        with self.assertRaisesRegex(RuntimeError, "non-target collision"):
            test.run_intercept()
        test.ros_iface.publish_offboard.assert_called_once_with(False)

    def test_interactive_return_home_skips_unreleased_launcher(self):
        test = self.make_mission()
        test.launcher = SimpleNamespace(released=False, pivot=(0, 0, 0))
        test.return_home = Mock()
        test.interactive_return_home()
        test.return_home.assert_not_called()
        self.assertEqual(test.report["phases"]["rth"]["outcome"], "not-launched-home")

    @patch.object(mission, "ManagedProcess")
    def test_interactive_launches_joystick_bridge_with_passthrough_args(self, process_class):
        test = self.make_mission()
        test.args = SimpleNamespace(
            joystick_device="/dev/input/js1", joystick_args="--invert pitch --rate 100")
        test.workdir = Path("/tmp/not-created-by-unit-test")
        test.external_rc = False
        test.start_joystick_bridge()
        argv = process_class.call_args.args[1]
        self.assertEqual(argv[argv.index("--device") + 1], "/dev/input/js1")
        self.assertEqual(argv[0], sys.executable)
        self.assertIn("--invert", argv)  # shlex-split user passthrough args
        self.assertNotIn("--arm-mode", argv)  # latch is a user choice, not forced
        process_class.return_value.wait_for_log.assert_called_once_with("streaming", 10)

    def test_interactive_adopts_an_already_streaming_rc_sender(self):
        test = self.make_mission()
        test.args = SimpleNamespace(joystick_device="/dev/input/js0", joystick_args="")
        test.external_rc = True
        test.start_joystick_bridge()  # early return: no workdir/process needed

    def prepare_intercept(self, samples, mission_active=True):
        test = self.make_mission()
        test.origin = (0, 0, 0)
        test.ros_iface = Mock(tracker_state=2, mission_state=mission_active, control_sequence=0)
        test.ros_iface.trigger_seen.return_value = False
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
    def test_intercept_report_measures_target_motion_independently(self, _sleep):
        test = self.prepare_intercept([
            {"vel": (0, 0, 0), "distance": 10,
             "target_pos": (70, 0, -45), "target_vel": (0, 1.5, 0)},
            {"vel": (0, 0, 0), "distance": .75,
             "target_pos": (70, 2, -45), "target_vel": (0, 1.2, 0)}])
        result = test.run_intercept()
        self.assertEqual(result["target_peak_speed_m_s"], 1.5)
        self.assertEqual(result["target_path_length_m"], 2)
        self.assertEqual(result["target_displacement_m"], 2)

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
        self.assertTrue(test.report["phases"]["rth"]["disarm_trigger_off"])
        test.ros_iface.publish_offboard.assert_called_once_with(False)

    @patch.object(mission.time, "sleep")
    def test_return_fails_without_disarm_edge_off_trigger(self, _sleep):
        test = self.prepare_return(
            {"pos": (100.1, -30.1, -3), "vel": (0, 0, 0), "modes": []}, 200)
        test.wait_for_trigger = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "auto-publish /fpv/tracker_trigger OFF"):
            test.return_home()

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
        target.args = SimpleNamespace(scale=1, cruise=8, reset=True, motion="hover",
                                      orbit_radius=5, orbit_speed=1.5, duration=0)
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

    def test_orbit_reference_stays_centered_with_exact_tangential_speed(self):
        center = (-328, 2, -48)
        for radius, speed in ((2, .5), (5, 1.5), (10, 3)):
            period = 2 * math.pi * radius / speed
            for t in (0, .35, period / 4, period / 2, period, 10 * period):
                with self.subTest(radius=radius, speed=speed, t=t):
                    pos, vel = balloon.orbit_reference(center, radius, speed, t)
                    offset = tuple(p-c for p, c in zip(pos, center))
                    self.assertAlmostEqual(math.hypot(*offset), radius)
                    self.assertEqual(pos[2], center[2])
                    self.assertAlmostEqual(math.hypot(*vel), speed)
                    self.assertAlmostEqual(sum(p*v for p, v in zip(offset, vel)), 0)

    def test_motion_parameters_reject_invalid_values_before_starting(self):
        for radius, speed in ((0, 1), (-1, 1), (5, 0), (math.nan, 1), (5, math.inf),
                              (11, 1), (5, 6)):
            with self.subTest(radius=radius, speed=speed), self.assertRaises(ValueError):
                balloon.validate_motion("orbit", radius, speed)
        with self.assertRaises(ValueError):
            balloon.orbit_reference((0, 0, 0), 5, 1, -1)
        with self.assertRaises(ValueError):
            balloon.validate_motion("invalid", 5, 1)

    def test_orbit_clearance_covers_entry_connector_and_entire_circle(self):
        target = self.make_balloon()
        target.args.motion = "orbit"
        target.corridor_clear = Mock(return_value=True)
        target.check_motion_clearance()
        points = target.corridor_clear.call_args.args[0]
        self.assertEqual(points[0], target.standoff)
        self.assertEqual(points[1], (65, 36, -45))
        self.assertGreaterEqual(len(points), 34)
        self.assertLess(math.dist(points[1], points[-1]), 1e-8)
        for pos in points[1:]:
            self.assertAlmostEqual(math.dist(pos, target.standoff), 5)
        target.corridor_clear.return_value = False
        with self.assertRaisesRegex(RuntimeError, "orbit corridor is obstructed"):
            target.check_motion_clearance()
        target.client.armDisarm.assert_not_called()

    @patch.object(balloon.time, "sleep")
    def test_dynamic_loop_uses_simulation_time_without_log_control_pauses(self, sleep):
        target = self.make_balloon()
        target.args.motion = "orbit"
        target.fly_leg = Mock()
        target.command_velocity = Mock()
        times = [0, .5, 1, 1.5, 2]
        samples = [balloon.orbit_reference(target.standoff, 5, 1.5, t) for t in times]
        states = [SimpleNamespace(
            timestamp=int((100+t) * 1e9),
            kinematics_estimated=SimpleNamespace(linear_velocity=airsim.Vector3r(*vel)))
            for t, (_, vel) in zip(times, samples)]
        target.client.getMultirotorState.side_effect = [states[0], *states]
        positions = iter([samples[0][0], *(p for p, _ in samples)])
        target.vehicle_position = lambda name: next(positions) if name == balloon.TARGET_NAME else (0, 0, 0)
        target.stop.is_set.side_effect = [False] * len(times) + [True]
        with patch("builtins.print") as output:
            target.hold_orbit()
        target.fly_leg.assert_called_once_with(samples[0][0])
        for call, (_, vel) in zip(target.command_velocity.call_args_list, samples):
            for actual, expected in zip(call.args[0], vel):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(call.kwargs["duration"], .1)
        self.assertTrue(any("target ready: motion=orbit" in c.args[0] for c in output.call_args_list))
        sleep.assert_not_called()

    def test_motion_envelope_deviation_stops_before_another_command(self):
        target = self.make_balloon()
        target.args.motion = "orbit"
        target.fly_leg = Mock()
        target.command_velocity = Mock()
        target.vehicle_position = Mock(return_value=(68, 36, -45))
        target.client.getMultirotorState.return_value = SimpleNamespace(
            timestamp=100_000_000_000,
            kinematics_estimated=SimpleNamespace(linear_velocity=airsim.Vector3r()))
        with self.assertRaisesRegex(RuntimeError, "left motion envelope"):
            target.hold_orbit()
        target.command_velocity.assert_not_called()

    @patch.object(mission.time, "sleep")
    @patch.object(mission.time, "monotonic", side_effect=itertools.count(1, .25))
    def test_prelaunch_tracks_moving_target_before_mission(self, _clock, _sleep):
        test = self.make_mission()
        test.args = SimpleNamespace(balloon_motion="orbit", balloon_orbit_speed=1.5)
        test.ros_iface = Mock(tracker_state=2)
        test.capture_png = Mock()
        test.launcher = SimpleNamespace(yaw=0, pitch=-.8)
        positions = itertools.count(0, .3)

        def sample():
            y = next(positions)
            test.launcher.yaw = y / 70
            return {"target_pos": (70, y, -45), "target_vel": (0, 1.5, 0), "camera_error_deg": 1}

        test.sample = sample
        test.start_tracking()
        evidence = test.report["phases"]["prelaunch_tracking"]
        self.assertGreater(evidence["target_displacement_m"], .5)
        self.assertGreater(evidence["launcher_yaw_change_deg"], .1)
        # 起桨路径的 ON 由桥在 arm 边沿自动发布（arm_fc 已校验），此处不再手动发。
        test.ros_iface.publish_trigger.assert_not_called()

    @patch.object(mission.time, "sleep")
    @patch.object(mission.time, "monotonic", side_effect=itertools.count(1, .25))
    def test_aim_only_tracking_still_publishes_manual_on(self, _clock, _sleep):
        test = self.make_mission()
        test.args = SimpleNamespace(balloon_motion="hover")
        test.ros_iface = Mock(tracker_state=2)
        test.capture_png = Mock()
        test.sample = Mock(return_value={
            "target_pos": (70, 0, -45), "target_vel": (0, 0, 0), "camera_error_deg": None})
        test.start_tracking(manual_trigger=True)
        test.ros_iface.publish_trigger.assert_called_once_with("ON")
        self.assertIn("prelaunch_tracking", test.report["phases"])

    @patch.object(mission.time, "sleep")
    @patch.object(mission.time, "monotonic", side_effect=itertools.count(1))
    def test_dynamic_readiness_rejects_stationary_target_even_if_tracker_locked(self, _clock, _sleep):
        test = self.make_mission()
        test.args = SimpleNamespace(balloon_motion="orbit", balloon_orbit_speed=1.5)
        test.ros_iface = Mock(tracker_state=2)
        test.capture_png = Mock()
        test.sample = Mock(return_value={
            "target_pos": (70, 0, -45), "target_vel": (0, 0, 0), "camera_error_deg": 0})
        with self.assertRaisesRegex(RuntimeError, "tracking/aim readiness failed"):
            test.start_tracking()
        self.assertNotIn("prelaunch_tracking", test.report["phases"])

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
