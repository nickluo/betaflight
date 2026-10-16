#!/usr/bin/env python3
"""Closed-loop balloon interception mission: SITL + apm_bridge + tracker + interceptor.

Orchestrates the full engagement against a running Cosys-AirSim environment:

  1. fresh SITL (provisioned: GPS virtual, aux table, GPS-Rescue RTH)
  2. apm_bridge custom_link relay (AirSim 5764 <-> bridge <-> SITL 5763)
  3. airsim_tracker (YOLO+nanotrack) + drone_interceptor (relative IMM-UKF)
  4. balloon flies to the standoff point (~70 m out, >40 m alt; separate script)
  5. ground yaw alignment toward the balloon, ARM (props spin up)
  6. tracker_trigger ON -> wait for visual lock -> tracker_trigger MISSION
  7. interceptor flies the engagement; success = ground-truth closest approach
  8. one-key RTH: mode dial sixth detent -> GPS Rescue home + land + disarm

Run under the ROS 2 environment (see airsim_control_test.py for the SITL
mechanics this builds on):

    source /opt/ros/jazzy/setup.bash
    source <fpv_ws>/install/setup.bash
    /usr/bin/python3 airsim_intercept_mission.py --binary <SITL.elf> \
        --balloon-script airsim_balloon_target.py
"""

import argparse
import contextlib
import hashlib
import json
import math
import os
import shutil
import signal
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
import yaml
from msgpackrpc.error import TimeoutError as RpcTimeoutError, TransportError

sys.path.insert(0, str(Path(__file__).resolve().parent))
import airsim_control_test as control  # noqa: E402  (MspClient, RcStream, provision, ...)
import sitl_joystick as joystick  # noqa: E402
from airsim_launcher import AirsimLauncher, quaternion_rpy, rotate  # noqa: E402
from airsim_balloon_target import park_target, validate_motion  # noqa: E402

# Must precede the rclpy import: rmw can latch ROS_DOMAIN_ID when the library
# loads, so setting it later would leave the orchestrator on a different domain
# than the bridge/tracker/interceptor it launches.
ROS_DOMAIN = os.environ.get("ROS_DOMAIN_ID", "78")
os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN

import cosysairsim as airsim  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from quadrotor_msgs.msg import ControlCommand  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Bool, Header, Int8  # noqa: E402
from vision_msgs.msg import Detection2D  # noqa: E402

VEHICLE = "Copter"
TARGET = "TargetDrone"
LINK_WAIT = 30.0
TRACK_LOCK_WAIT = 45.0
MISSION_WAIT = 100.0
RTH_WAIT = 240.0
HIT_DISTANCE = 1.0  # m, common-world ground-truth intercept threshold


def apply_interceptor_profile(params, args):
    """Keep shared YAML tuning; apply only CLI limits and the SITL attitude source."""
    params["estimation"]["gyro_attitude_during_mission"] = False
    controller = params["controller"]
    controller["v_des"] = args.guidance_speed
    controller["v_pn_ref"] = min(args.guidance_speed, controller["v_pn_ref"])
    controller["turn_radius_cap_factor"] = args.turn_cap_factor
    params["control"]["rate_control_attenuation"] = args.rate_control_attenuation
    params["physics"]["v_climb"] = args.climb_speed
    params["physics"]["v_descent"] = args.climb_speed
    return params


SITL_CONFIG_BASE = (
    "profile 0",
    "set simplified_pids_mode = OFF",
    "set p_roll = 37",
    "set i_roll = 62",
    "set d_roll = 22",
    "set f_roll = 95",
    "set p_pitch = 32",
    "set i_pitch = 57",
    "set d_pitch = 22",
    "set f_pitch = 84",
    "set p_yaw = 10",
    "set i_yaw = 0",
    "set d_yaw = 0",
    "set f_yaw = 8",
    "feature GPS",
    "set gps_provider = VIRTUAL",
    "set trust_mag = ON",
    "set imu_dcm_kp = 2500",
    "set small_angle = 45",
    "set pwr_on_arm_grace = 5",
    "set failsafe_procedure = AUTO-LAND",
    "set custom_link_motors_stream = ON",
    "set airmode_start_throttle_percent = 15",
    "set mag_declination = 20",
    "set gps_rescue_min_start_dist = 15",
    "set gps_rescue_alt_mode = FIXED_ALT",
    "set gps_rescue_return_alt = 8",
    "set gps_rescue_initial_climb = 3",
    "set gps_rescue_ascend_rate = 200",
    "set gps_rescue_descend_rate = 200",
    "set gps_rescue_landing_rate = 50",
    "set gps_rescue_ground_speed = 300",
    "set gps_rescue_descent_dist = 8",
    "set gps_rescue_min_sats = 8",
    "set gps_rescue_allow_arming_without_fix = OFF",
    *joystick.CLI_CONFIG_LINES,
    "aux 7 56 5 1700 2100 0 0",  # flight-plan box on AUX6 (rescue plan executor)
)


def rescue_controller(modes):
    """Either the legacy GPS-Rescue box or the synthesized rescue flight plan."""
    if 46 in modes:
        return "legacy-gps-rescue"
    if {56, 3, 11} <= set(modes):
        return "flight-plan-rescue"
    return None


def mission_passed(phases, error):
    return (error is None and phases.get("intercept", {}).get("outcome") == "hit"
            and phases.get("rth", {}).get("outcome") == "returned")


def log(message):
    print(f"[mission {time.strftime('%H:%M:%S')}] {message}", flush=True)


def camera_target_error_degrees(camera_flu_to_enu, relative_ned):
    forward_enu = rotate(airsim.Quaternionr(*camera_flu_to_enu), (1, 0, 0))
    forward_ned = (forward_enu[1], forward_enu[0], -forward_enu[2])
    length = math.hypot(*relative_ned) * math.hypot(*forward_ned)
    if not math.isfinite(length) or length <= 1e-6:
        raise RuntimeError("invalid camera boresight or target displacement")
    return math.degrees(math.acos(max(-1, min(1, sum(
        a*b for a, b in zip(forward_ned, relative_ned)) / length))))


def connect_airsim(host, port, timeout):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        client = airsim.MultirotorClient(ip=host, port=port, timeout_value=5)
        try:
            with socket.create_connection((host, port), timeout=1):
                pass
            if not client.ping():
                raise ConnectionError("AirSim RPC ping returned false")
            missing = {VEHICLE, TARGET} - set(client.listVehicles())
            if missing:
                raise RuntimeError(f"AirSim missing required vehicles: {sorted(missing)}")
        except (OSError, TransportError, RpcTimeoutError) as exc:
            client.client.close()
            last_error = exc
            log(f"waiting for AirSim RPC {host}:{port}: {exc}")
            time.sleep(.5)
        except Exception:
            client.client.close()
            raise
        else:
            return client
    raise RuntimeError(f"AirSim RPC {host}:{port} not ready after {timeout:.0f}s; "
                       f"ensure UE game/Play and its RPC server are running: {last_error}") from last_error


class RosInterface(Node):
    def __init__(self):
        super().__init__("intercept_mission")
        self.trigger_pub = self.create_publisher(Header, "/fpv/tracker_trigger", 10)
        self.arm_pub = self.create_publisher(Bool, "/fpv/arm", 10)
        self.offboard_pub = self.create_publisher(Bool, "/fpv/offboard", 10)
        self.control_pub = self.create_publisher(ControlCommand, "/fpv/control_command", 10)
        self.tracker_state = -1
        self.mission_state = None
        self.imu_count = 0
        self.gimbal_quat = None
        self.gimbal_stamp = 0.0
        self.imu_quat = None
        self.imu_acceleration = None
        self.target_los = None
        self.target_bbox = None
        self.control_sequence = 0
        self.create_subscription(ControlCommand, "/fpv/control_command", self.control_received, 10)
        self.create_subscription(TwistStamped, "/fpv/target_los",
                                 lambda m: setattr(self, "target_los", m), 10)
        self.create_subscription(Detection2D, "/fpv/target_bbox",
                                 lambda m: setattr(self, "target_bbox", m), 10)
        self.create_subscription(Int8, "/fpv/tracker_state",
                                 lambda m: setattr(self, "tracker_state", m.data), 10)
        self.create_subscription(Bool, "/fpv/mission_state",
                                 lambda m: setattr(self, "mission_state", m.data), 10)
        self.create_subscription(Imu, "/mavros/imu/data",
                                 lambda m: (setattr(self, "imu_count", self.imu_count + 1),
                                            setattr(self, "imu_quat", (m.orientation.x, m.orientation.y,
                                                                       m.orientation.z, m.orientation.w)),
                                            setattr(self, "imu_acceleration", (m.linear_acceleration.x,
                                                                                m.linear_acceleration.y,
                                                                                m.linear_acceleration.z))),
                                 qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/fpv/gimbal",
            lambda m: (setattr(self, "gimbal_quat", (m.orientation.x, m.orientation.y,
                                                     m.orientation.z, m.orientation.w)),
                       setattr(self, "gimbal_stamp", time.monotonic())), 10)

    def control_received(self, message):
        if message.armed and message.control_mode == ControlCommand.BODY_RATES:
            self.control_sequence += 1

    def publish_trigger(self, frame_id):
        msg = Header()
        msg.frame_id = frame_id
        msg.stamp = self.get_clock().now().to_msg()
        self.trigger_pub.publish(msg)
        log(f"published /fpv/tracker_trigger frame_id={frame_id}")

    def publish_arm(self, armed):
        self.arm_pub.publish(Bool(data=armed))
        log(f"published /fpv/arm={armed}")

    def publish_offboard(self, enabled):
        self.offboard_pub.publish(Bool(data=enabled))
        log(f"published /fpv/offboard={enabled}")


class ManagedProcess:
    def __init__(self, name, argv, cwd=None, log_path=None, shell_env=None, new_session=True):
        self.name = name
        self.argv = argv
        self.cwd = cwd
        self.log_path = log_path
        self.shell_env = shell_env
        self.new_session = new_session
        self.process = None
        self.log_file = None

    def start(self):
        if self.log_path:
            self.log_file = open(self.log_path, "w")
            stream = self.log_file
        else:
            stream = subprocess.DEVNULL
        self.process = subprocess.Popen(
            self.argv, cwd=self.cwd, stdout=stream, stderr=subprocess.STDOUT,
            env={**os.environ, **(self.shell_env or {})},
            start_new_session=self.new_session)
        log(f"started {self.name} (pid {self.process.pid})")
        return self.process

    def wait_for_log(self, marker, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_running()
            if self.log_path and self.log_path.exists():
                text = self.log_path.read_text(errors="replace")
                if "process has died" in text:
                    raise RuntimeError(f"{self.name} ROS node died; log={self.log_path}\n"
                                       + "\n".join(text.splitlines()[-12:]))
                if marker in text:
                    return text
            time.sleep(0.2)
        raise TimeoutError(f"{self.name} log never showed {marker!r} within {timeout:.0f}s")

    def check_running(self):
        if self.process is not None and self.process.poll() is not None:
            tail = ""
            if self.log_path and self.log_path.exists():
                tail = "\n" + "\n".join(self.log_path.read_text(errors="replace").splitlines()[-12:])
            raise RuntimeError(f"{self.name} exited early (code {self.process.returncode}); "
                               f"log={self.log_path}{tail}")

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None and self.new_session:
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                with contextlib.suppress(ProcessLookupError, PermissionError):
                    os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        elif self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.log_file:
            self.log_file.close()
        log(f"stopped {self.name}")


class InterceptMission:
    def __init__(self, args):
        self.args = args
        self.report = {"phases": {}, "passed": False}
        self.armed_once = False
        self.ros_iface = None
        self.spin_thread = None
        self.client = None
        self.msp = None
        self.msp_lock = threading.Lock()
        self.rc = None
        self.sitl = None
        self.processes = []
        self.balloon = None
        self.box_ids = []
        self.origin = None
        self.geo_origin = None
        self.min_distance = None
        self.trajectory = None
        self.trajectory_file = None
        self.workdir = None
        self.stage = "preflight"
        self.bridge = None
        self.pitch_aim = None  # None = yaml default (-15); set by auto-calibration
        self.airborne_timestamp = None
        self.launcher = None
        self.hard_collision = False
        self.world_ready = False

    # ---------------------------------------------------------------- helpers

    def msp_request(self, command):
        with self.msp_lock:
            return self.msp.request(command)

    def modes(self):
        payload = self.msp_request(101)
        modes, flags = control.decode_status(payload, self.box_ids)
        return sorted(modes), flags

    def gt(self, name=VEHICLE):
        k = self.client.simGetGroundTruthKinematics(vehicle_name=name)
        # Kinematics/vehicle APIs use each vehicle's own spawn origin.
        # Object poses use the common world NED origin for inter-vehicle distance.
        p, v = self.client.simGetObjectPose(name).position, k.linear_velocity
        q = k.orientation
        if not all(math.isfinite(value) for value in
                   (p.x_val, p.y_val, p.z_val, v.x_val, v.y_val, v.z_val,
                    q.x_val, q.y_val, q.z_val, q.w_val)):
            raise RuntimeError(f"invalid AirSim ground truth for {name}")
        roll = math.degrees(math.atan2(2 * (q.w_val * q.x_val + q.y_val * q.z_val),
                                       1 - 2 * (q.x_val ** 2 + q.y_val ** 2)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w_val * q.y_val - q.z_val * q.x_val)))))
        yaw = math.degrees(math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val),
                                      1 - 2 * (q.y_val ** 2 + q.z_val ** 2)))
        return {"pos": (p.x_val, p.y_val, p.z_val),
                "vel": (v.x_val, v.y_val, v.z_val),
                "rpy": (roll, pitch, yaw)}

    def sample(self):
        if self.balloon is not None:
            self.balloon.check_running()
        if self.launcher is not None and self.launcher.error is not None:
            raise RuntimeError(f"launcher actuator failed: {self.launcher.error}")
        if self.rc is not None and self.rc.error is not None:
            raise RuntimeError(f"RC stream failed: {self.rc.error}")
        state = self.gt()
        target = self.gt(TARGET)
        distance = math.dist(state["pos"], target["pos"])
        if self.stage == "mission" and (self.min_distance is None or distance < self.min_distance):
            self.min_distance = distance
        modes, flags = self.modes()
        los = self.ros_iface.target_los if self.ros_iface else None
        bbox = self.ros_iface.target_bbox if self.ros_iface else None
        los_age = None if los is None else (
            self.ros_iface.get_clock().now().nanoseconds / 1e9 -
            (los.header.stamp.sec + los.header.stamp.nanosec / 1e9))
        camera = getattr(self.ros_iface, "gimbal_quat", None)
        camera_error = None if camera is None else camera_target_error_degrees(
            camera, tuple(t-p for t, p in zip(target["pos"], state["pos"])))
        row = (self.stage, round(time.monotonic() - self.t0, 2),
               *state["pos"], *state["vel"], *state["rpy"],
               ";".join(map(str, modes)), flags,
               distance, self.min_distance,
               self.ros_iface.tracker_state if self.ros_iface else -1,
               self.ros_iface.mission_state if self.ros_iface else None,
               *target["pos"], bbox.bbox.size_x if bbox else None,
               los.header.frame_id if los else "", los_age, camera_error,
               *target["vel"],
               math.degrees(self.launcher.yaw) if self.launcher else None,
               math.degrees(self.launcher.pitch) if self.launcher else None,
               *(self.ros_iface.imu_quat if self.ros_iface and self.ros_iface.imu_quat
                 else (None,) * 4))
        if self.trajectory:
            self.trajectory.writerow(row)
        state["modes"] = modes
        state["flags"] = flags
        state["distance"] = distance
        state["target_pos"] = target["pos"]
        state["target_vel"] = target["vel"]
        state["camera_error_deg"] = camera_error
        return state

    def wait(self, description, predicate, timeout=30, dwell=0.0, period=0.1):
        deadline, since = time.monotonic() + timeout, None
        last = None
        while time.monotonic() < deadline:
            last = self.sample()
            if predicate(last):
                if since is None:
                    since = time.monotonic()
                if time.monotonic() - since >= dwell:
                    log(f"PASS: {description}")
                    return last
            else:
                since = None
            time.sleep(period)
        raise AssertionError(f"timeout: {description}; last modes={last['modes'] if last else None} "
                             f"dist={last['distance'] if last else None:.1f}" if last else description)

    def ros_launch_argv(self, package, launch_file, extra=()):
        script = (f"source /opt/ros/jazzy/setup.bash && "
                  f"source {shlex.quote(str(self.args.ros_ws / 'install/setup.bash'))} && "
                  f"export ROS_DOMAIN_ID={ROS_DOMAIN} && "
                  f"exec ros2 launch {package} {launch_file} " + shlex.join(extra))
        return ["/bin/bash", "-c", script]

    def wait_for_ros_topic(self, topic, timeout=30, publishers=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            names_types = self.ros_iface.get_topic_names_and_types()
            found = any(name == topic for name, _ in names_types)
            if found and (not publishers or self.ros_iface.count_publishers(topic) > 0):
                return
            time.sleep(0.3)
        raise TimeoutError(f"ROS topic {topic} never appeared")

    # ----------------------------------------------------------------- phases

    def preflight(self):
        self.stage = "preflight"
        self.workdir = Path(self.args.workdir or f"/tmp/bf_intercept_{os.getpid()}")
        self.workdir.mkdir(parents=True, exist_ok=False)
        log(f"artifacts: {self.workdir}")
        self.report["inputs"] = {
            "firmware_sha256": hashlib.sha256(self.args.binary.read_bytes()).hexdigest()}
        if self.args.eeprom:
            self.report["inputs"]["source_eeprom_sha256"] = hashlib.sha256(
                self.args.eeprom.read_bytes()).hexdigest()
        control.require_free_ports()
        control.require_quiet_rc()
        self.client = connect_airsim(self.args.host, self.args.rpc_port, self.args.rpc_wait)
        self.client.confirmConnection()
        self.client.cancelLastTask(vehicle_name=TARGET)
        self.client.reset()
        if self.client.simIsPause():
            self.client.simPause(False)
        time.sleep(1.0)
        self.world_ready = True
        if self.client.isApiControlEnabled(vehicle_name=TARGET):
            raise RuntimeError("AirSim reset did not release target API control")
        # Reset the interceptor to a known pose: earlier runs or manual
        # debugging may have left it rotated/moved on the ground.
        pose = self.client.simGetVehiclePose(vehicle_name=VEHICLE)
        # Approach the launch surface from above; its height differs between
        # editor Play and standalone game origins.
        if self.args.launcher:
            settings = json.loads(self.client.getSettingsString())
            if "LauncherOverview" not in settings.get("Vehicles", {}).get(VEHICLE, {}).get("Cameras", {}):
                raise RuntimeError("LauncherOverview camera is not configured; apply airsim_settings.json and restart UE")
            self.launcher = AirsimLauncher(self.client, self.args.host, self.args.rpc_port,
                                           self.args.reference_camera, self.args.launcher_forward)
            overview = airsim.Pose(airsim.Vector3r(),
                                   quaternion_rpy(0, math.radians(10), self.launcher.heading))
            self.client.simSetCameraPose("LauncherOverview", overview, vehicle_name=VEHICLE)
            measured = self.client.simGetCameraInfo("LauncherOverview", vehicle_name=VEHICLE).pose.position
            anchor = self.launcher.anchor.position
            o = self.launcher.spawn_offset
            overview.position = airsim.Vector3r(anchor.x_val-measured.x_val-o[0],
                                                anchor.y_val-measured.y_val-o[1],
                                                anchor.z_val-measured.z_val-o[2])
            self.client.simSetCameraPose("LauncherOverview", overview, vehicle_name=VEHICLE)
            x, y = self.launcher.ground_xy
            o = self.launcher.spawn_offset
            pose.position = airsim.Vector3r(x-o[0], y-o[1],
                                            self.launcher.anchor.position.z_val-20-o[2])
            pose.orientation = quaternion_rpy(0, 0, self.launcher.heading)
        else:
            pose.position = airsim.Vector3r(self.args.launch_x, self.args.launch_y, -30.0)
            pose.orientation = airsim.Quaternionr(0.0, 0.0, 0.0, 1.0)
        self.client.simSetVehiclePose(pose, True, vehicle_name=VEHICLE)
        if self.client.simIsPause():
            self.client.simPause(False)
        deadline = time.monotonic() + 10
        contact = False
        since = None
        while time.monotonic() < deadline:
            drone = self.gt()
            collision = self.client.simGetCollisionInfo(vehicle_name=VEHICLE)
            contact = contact or (collision.has_collided and collision.normal.z_val < -0.5)
            if contact and math.hypot(*drone["vel"]) < 0.1:
                since = since or time.monotonic()
                if time.monotonic() - since >= 1.0:
                    break
            else:
                since = None
            time.sleep(0.1)
        else:
            raise RuntimeError(f"launch surface not reached safely: {drone}")
        if self.launcher is not None:
            # The constructor's first pose pair can straddle a falling reset frame.
            # Re-measure only after physical contact and stationary render/physics poses.
            self.launcher.refresh_spawn_offset()
            ground_z = collision.impact_point.z_val + self.launcher.spawn_offset[2]
            self.launcher.build(ground_z)
            time.sleep(1.0)
            self.report["phases"]["launcher"] = self.launcher.description()
        try:
            self.geo_origin = self.client.getGpsData(vehicle_name=VEHICLE).gnss.geo_point
        except Exception:  # noqa: BLE001 - waypoint line is optional
            self.geo_origin = None
            log("GPS geo point unavailable; provisioning without waypoint")
        drone = self.gt()
        self.origin = drone["pos"]
        self.report["origin_ned_m"] = self.origin
        log(f"drone origin {self.origin}")
        self.trajectory_file = open(self.workdir / "trajectory.csv", "w", newline="")
        import csv
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow(("stage", "t", "px", "py", "pz", "vx", "vy", "vz",
                                  "roll", "pitch", "yaw", "modes", "flags",
                                  "dist_target", "min_dist", "tracker_state", "mission_state",
                                  "target_px", "target_py", "target_pz", "bbox_width",
                                  "los_frame", "los_age", "camera_error_deg",
                                  "target_vx", "target_vy", "target_vz",
                                  "launcher_yaw_deg", "launcher_pitch_deg",
                                  "fc_qx", "fc_qy", "fc_qz", "fc_qw"))

    def start_sitl(self):
        self.stage = "sitl"
        config = list(SITL_CONFIG_BASE)
        if self.launcher is not None:
            config.append("set small_angle = 65")  # Mounted launcher tilt, simulation only.
        geo = self.geo_origin
        if geo is not None:
            latitude = geo.latitude + 20.0 / 111319.49
            config.append(f"waypoint insert 0 {latitude:.7f} {geo.longitude:.7f} "
                          f"{round((geo.altitude + 5) * 100)} 250 flyover 0 none")
        if self.args.eeprom:
            shutil.copy2(self.args.eeprom.resolve(), self.workdir / "eeprom.bin")
        control.provision(self.args.binary, self.workdir, config, log_name="provision")
        log_file = open(self.workdir / "sitl.log", "w")
        self.sitl = subprocess.Popen([str(self.args.binary)], cwd=self.workdir,
                                     stdout=log_file, stderr=subprocess.STDOUT)
        import socket
        deadline = time.monotonic() + 20
        sock = None
        while time.monotonic() < deadline:
            if self.sitl.poll() is not None:
                raise RuntimeError("SITL exited during startup; see sitl.log")
            try:
                sock = socket.create_connection(("127.0.0.1", 5761), timeout=0.5)
                break
            except (ConnectionRefusedError, OSError):
                time.sleep(0.2)
        if sock is None:
            raise TimeoutError("SITL MSP did not become responsive")
        self.msp = control.MspClient(sock)
        self.box_ids = list(self.msp_request(119))
        self.verify_rate_profile()
        self.rc = control.RcStream("127.0.0.1")
        self.rc.thread.start()
        log("SITL up (MSP 5761), RC streaming on UDP 9004")

    def verify_rate_profile(self):
        actual = list(self.msp_request(112)[:9])  # MSP_PID: roll/pitch/yaw P,I,D.
        expected = [37, 62, 22, 32, 57, 22, 10, 0, 0]
        if actual != expected:
            raise RuntimeError(f"SITL rate profile did not load: expected {expected}, got {actual}")
        self.report["flight_controller_rate_pid"] = {
            axis: dict(zip(("p", "i", "d"), actual[i*3:i*3+3]))
            for i, axis in enumerate(("roll", "pitch", "yaw"))}
        log(f"verified SITL rate PID profile over MSP: {actual}")

    def start_ros(self):
        self.stage = "ros"
        rclpy.init()
        self.ros_iface = RosInterface()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_iface,), daemon=True)
        self.spin_thread.start()

        bridge_parameters = self.args.ros_ws / "install/apm_bridge/share/apm_bridge/parameters/sitl_parameters.yaml"
        bridge_args = [] if self.args.pitch_aim is None else [f"pitch_aim:={self.args.pitch_aim}"]
        if self.launcher is not None:
            profile = yaml.safe_load(bridge_parameters.read_text())
            gimbal = profile["custom_link_bridge"]["ros__parameters"]["gimbal"]
            gimbal["pitch_aim"], gimbal["pitch_max"] = -10.0, 85.0
            gimbal["mission_on_ground"] = True
            gimbal.update(tracking_gain=2.0, tracking_max_rate=60.0,
                          tracking_max_accel=180.0, tracking_filter_tau=.12,
                          tracking_rate_feedforward=0.0,
                          max_angle_speed=120.0)
            bridge_parameters = self.workdir / "bridge_parameters.yaml"
            bridge_parameters.write_text(yaml.safe_dump(profile))
            bridge_args.append(f"parameters:={bridge_parameters}")
        self.bridge = ManagedProcess(
            "apm_bridge", self.ros_launch_argv("apm_bridge", "sitl.launch.py",
                bridge_args),
            log_path=self.workdir / "bridge.log")
        self.bridge.start()
        self.processes.append(self.bridge)
        self.bridge.wait_for_log("Custom link up", LINK_WAIT)
        log("bridge custom link up")

        # AirSim's BetaFlight adapter reconnects to the relay (~1 s) and DDS
        # discovery needs its own moment — poll for a sustained telemetry rate.
        deadline = time.monotonic() + 15
        rate = 0
        while time.monotonic() < deadline:
            base = self.ros_iface.imu_count
            time.sleep(1.0)
            rate = self.ros_iface.imu_count - base
            if rate >= 50:
                break
        if rate < 50:
            raise RuntimeError(f"FC telemetry too slow ({rate} imu/s); "
                               "is the AirSim<->bridge<->SITL chain alive?")
        log(f"imu telemetry {rate} msg/s")

        def gps_ready():
            gps = self.msp_request(106)
            return len(gps) >= 2 and gps[0] and gps[1] >= 8
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not gps_ready():
            time.sleep(0.5)
        if not gps_ready():
            raise RuntimeError("FC never got a GPS fix from the sim (virtual GPS)")
        log("FC GPS fix acquired (>=8 sats)")

        tracker = ManagedProcess(
            "airsim_tracker", self.ros_launch_argv(
                "airsim_tracker", "launch.py", ["show_video:=True", "publish_prediction:=False"]),
            log_path=self.workdir / "tracker.log")
        tracker.start()
        self.processes.append(tracker)
        self.wait_for_ros_topic("/fpv/target_bbox", 30)
        log("tracker up (/fpv/target_bbox present)")

        config_path = self.args.ros_ws / "install/drone_interceptor/share/drone_interceptor/config/relative_imm_ukf_params.yaml"
        profile = yaml.safe_load(config_path.read_text())
        params = profile["interceptor_node_relative_imm_ukf"]["ros__parameters"]
        apply_interceptor_profile(params, self.args)
        profile_path = self.workdir / "interceptor_parameters.yaml"
        profile_path.write_text(yaml.safe_dump(profile))
        self.report["configuration"] = {
            "guidance_speed_m_s": self.args.guidance_speed,
            "climb_speed_limit_m_s": self.args.climb_speed,
            "turn_cap_factor": self.args.turn_cap_factor,
            "sitl_imu_dcm_kp": 2500,
            "rate_control_attenuation": self.args.rate_control_attenuation,
            "balloon_motion": self.args.balloon_motion,
            "balloon_orbit_radius_m": self.args.balloon_orbit_radius,
            "balloon_orbit_speed_m_s": self.args.balloon_orbit_speed,
            "visual_speed_gain": params["controller"]["visual_speed_gain"],
            "visual_speed_floor_m_s": params["controller"]["visual_speed_floor"],
            "terminal_speed_floor_m_s": params["controller"]["terminal_speed_floor"],
            "terminal_speed_bbox_threshold_px": params["controller"]["terminal_speed_bbox_threshold"],
            "terminal_speed_los_rate_threshold_rad_s": params["controller"]["terminal_speed_los_rate_threshold"],
            "vertical_brake_accel_m_s2": params["controller"]["vertical_brake_accel"],
            "gyro_attitude_during_mission": params["estimation"]["gyro_attitude_during_mission"],
            "target_axial_feedforward_limit_m_s": params["controller"]["target_axial_feedforward_limit"],
            "ros_domain_id": ROS_DOMAIN,
            "hit_distance_m": HIT_DISTANCE}
        interceptor = ManagedProcess(
            "interceptor", self.ros_launch_argv(
                "drone_interceptor", "launch_relative_imm_ukf.py",
                ["use_simulation:=False", "use_radar:=False",
                 f"parameters:={profile_path}"]),
            log_path=self.workdir / "interceptor.log")
        interceptor.start()
        self.processes.append(interceptor)
        interceptor.wait_for_log("Relative IMM-UKF node ready", 30)
        self.wait_for_ros_topic("/fpv/control_command", 30)
        log("interceptor up (/fpv/control_command present)")

    def launch_balloon(self):
        self.stage = "balloon"
        dx, dy = self.args.balloon_x, self.args.balloon_y
        if self.launcher is not None:
            c, s = math.cos(self.launcher.heading), math.sin(self.launcher.heading)
            dx, dy = c*dx-s*dy, s*dx+c*dy
        argv = [sys.executable, str(self.args.balloon_script),
                "--host", str(self.args.host), "--rpc-port", str(self.args.rpc_port),
                "--x", str(self.origin[0] + dx),
                "--y", str(self.origin[1] + dy),
                "--z", str(self.origin[2] + self.args.balloon_z),
                "--motion", self.args.balloon_motion,
                "--orbit-radius", str(self.args.balloon_orbit_radius),
                "--orbit-speed", str(self.args.balloon_orbit_speed), "--reset"]
        if self.launcher is not None:
            argv += ["--spawn-x", str(self.origin[0]-5*math.sin(self.launcher.heading)),
                     "--spawn-y", str(self.origin[1]+5*math.cos(self.launcher.heading)),
                     "--spawn-z", str(self.origin[2]-3)]
        self.balloon = ManagedProcess("balloon", argv, new_session=False,
                                      log_path=self.workdir / "balloon.log")
        self.balloon.start()
        self.processes.append(self.balloon)
        self.balloon.wait_for_log("target ready", 120)
        target = self.gt(TARGET)
        horizontal = math.hypot(target["pos"][0] - self.origin[0],
                                target["pos"][1] - self.origin[1])
        altitude = self.origin[2] - target["pos"][2]
        log(f"balloon at {target['pos']}: horizontal {horizontal:.1f} m, alt {altitude:.1f} m")
        self.report["phases"]["balloon"] = {
            "horizontal_distance_m": horizontal, "height_m": altitude,
            "motion": self.args.balloon_motion,
            "center_world_ned_m": (self.origin[0]+dx, self.origin[1]+dy,
                                    self.origin[2]+self.args.balloon_z),
            "orbit_radius_m": self.args.balloon_orbit_radius,
            "orbit_speed_m_s": self.args.balloon_orbit_speed,
            "ready_speed_m_s": math.hypot(*target["vel"])}
        if not 60 <= horizontal <= 80 or altitude <= 40:
            raise RuntimeError("balloon standoff geometry out of bounds")

    def fc_yaw(self):
        return struct.unpack("<hhh", self.msp_request(108))[2] / 10.0

    def imu_yaw_math(self):
        """FC yaw estimate from /mavros/imu/data, converted from the bridge's
        ENU angle (0=east, CCW+) to NED compass heading (0=north, CW+)."""
        quat = self.ros_iface.imu_quat
        if quat is None:
            return None
        x, y, z, w = quat
        enu = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        return (90.0 - enu + 180.0) % 360.0 - 180.0

    def align_ground_yaw(self):
        """Point the grounded drone at the balloon before the props spin up.

        The adapter's FRD/NED -> FLU/NWU conversion must agree with the
        synthetic magnetic field and the bridge's ENU orientation.
        Physical stick yaw on the ground is only a fallback: the yaw rate loop
        against ground friction oscillates and can trip the FC's crash
        protection (motors stop mid-spin), so it uses gentle deflections.
        """
        self.stage = "align"
        if self.launcher is not None:
            self.launcher.aim_at(self.gt(TARGET)["pos"])
            self.launcher.start_aiming()
            def aligned(state):
                quat = self.ros_iface.imu_quat
                acceleration = self.ros_iface.imu_acceleration
                if quat is None or acceleration is None:
                    return False
                imu = control.euler_degrees(airsim.Quaternionr(*quat))
                r, p, y = state["rpy"]
                self.report["phases"]["mounted_alignment"] = {
                    "truth_rpy_deg": state["rpy"], "imu_rpy_enu_deg": imu,
                    "specific_force_m_s2": math.hypot(*acceleration),
                    "render_position_error_m": math.dist(state["pos"], self.launcher.position),
                    "launcher_pitch_deg": math.degrees(self.launcher.pitch)}
                return (abs(imu[0]-r) < 3 and abs(imu[1]+p) < 3
                        and abs(control.angle_error(imu[2], 90-y)) < 3
                        and 8 < math.hypot(*acceleration) < 12
                        and math.dist(state["pos"], self.launcher.position) < .05
                        and abs(math.degrees(self.launcher.pitch)-p) < 2
                        and abs(self.launcher.pitch) > math.radians(30))
            self.wait("mounted body pose, FC IMU and gravity agree", aligned,
                      timeout=20, dwell=1)
            self.origin = self.gt()["pos"]
            self.report["origin_ned_m"] = self.origin
            self.report["phases"]["launcher"] = self.launcher.description()
            log(f"Camera8 launcher aimed: {self.report['phases']['launcher']}")
            return
        target = self.gt(TARGET)["pos"]
        drone = self.gt()["pos"]
        bearing = (math.degrees(math.atan2(target[1] - drone[1], target[0] - drone[0])) + 180) % 360 - 180
        elevation = math.degrees(math.atan2(-(target[2] - drone[2]),
                                            math.hypot(target[0] - drone[0], target[1] - drone[1])))
        log(f"aligning ground yaw to balloon bearing {bearing:.1f} deg "
            f"(elevation {elevation:.1f} deg)")

        pose = self.client.simGetVehiclePose(vehicle_name=VEHICLE)
        half = math.radians(bearing) / 2.0
        pose.orientation = airsim.Quaternionr(0.0, 0.0, math.sin(half), math.cos(half))
        self.client.simSetVehiclePose(pose, True, vehicle_name=VEHICLE)
        time.sleep(2.0)  # let the mag fusion settle onto the new heading
        sim_yaw = self.gt()["rpy"][2]
        imu_yaw = self.imu_yaw_math()
        imu_yaw_s = f"{imu_yaw:.1f}" if imu_yaw is not None else "None"
        sim_err = (sim_yaw - bearing + 180) % 360 - 180
        imu_err = None if imu_yaw is None else (imu_yaw - bearing + 180) % 360 - 180
        log(f"after teleport: sim {sim_yaw:.1f} deg (err {sim_err:.1f}), "
            f"imu {imu_yaw_s} deg (err {imu_err if imu_err is None else round(imu_err, 1)})")
        if abs(sim_err) < 5.0 and imu_err is not None and abs(imu_err) < 8.0:
            log("yaw aligned via teleport (camera + estimator agree)")
            return
        raise RuntimeError("camera and FC heading disagree after ground alignment")

    def post_arm_yaw_check(self):
        """After arming, gently nudge the estimator onto the balloon bearing
        if the teleport alignment did not take (needs spinning motors)."""
        imu_yaw = self.imu_yaw_math()
        if imu_yaw is None:
            return
        target = self.gt(TARGET)["pos"]
        drone = self.gt()["pos"]
        bearing = (math.degrees(math.atan2(target[1] - drone[1], target[0] - drone[0]))
                   + 180) % 360 - 180
        error = (bearing - imu_yaw + 180) % 360 - 180
        if abs(error) < 6.0:
            log(f"post-arm yaw ok (imu {imu_yaw:.1f} vs bearing {bearing:.1f})")
            return
        log(f"post-arm yaw correction: imu {imu_yaw:.1f} -> {bearing:.1f}")
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            imu_yaw = self.imu_yaw_math()
            if imu_yaw is None:
                time.sleep(0.1)
                continue
            error = (bearing - imu_yaw + 180) % 360 - 180
            if abs(error) < 3.0:
                self.rc.set(yaw=1500, throttle=1000)
                time.sleep(0.5)
                log(f"post-arm yaw corrected to {imu_yaw:.1f}")
                return
            deflection = max(-90.0, min(90.0, error * 4.0))
            self.rc.set(yaw=int(round(1500 - deflection)), throttle=1120)
            time.sleep(0.05)
        self.rc.set(yaw=1500, throttle=1000)
        raise RuntimeError(f"post-arm yaw correction failed at imu {imu_yaw:.1f} "
                           f"(wanted {bearing:.1f})")

    def gimbal_boresight_elevation(self):
        """World elevation (deg) of the camera boresight, from /fpv/gimbal."""
        if self.ros_iface.gimbal_quat is None or \
                time.monotonic() - self.ros_iface.gimbal_stamp > 1.0:
            return None
        x, y, z, w = self.ros_iface.gimbal_quat
        return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (x * z - w * y)))))

    def restart_bridge(self):
        marker = time.monotonic()
        self.ros_iface.gimbal_stamp = 0.0
        self.bridge.stop()
        self.processes.remove(self.bridge)
        argv = self.ros_launch_argv("apm_bridge", "sitl.launch.py",
                                    [f"pitch_aim:={self.pitch_aim:.1f}"])
        self.bridge = ManagedProcess("apm_bridge", argv,
                                     log_path=self.workdir / "bridge.log")
        self.bridge.start()
        self.processes.append(self.bridge)
        self.bridge.wait_for_log("Custom link up", LINK_WAIT)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.ros_iface.gimbal_stamp > marker:
                return True
            time.sleep(0.2)
        return False

    def calibrate_gimbal_aim(self):
        """Point the AIM boresight at the balloon elevation before detection.

        AIM requests gimbal pitch 90+pitch_aim (deg toward body +Z). The yaml
        default (-15 -> 75 deg up) overshoots a ~33 deg-elevation target, so
        measure the actual boresight via /fpv/gimbal and correct pitch_aim,
        estimating the response slope empirically across bridge restarts.
        """
        if self.args.pitch_aim is not None:
            self.pitch_aim = self.args.pitch_aim
            log(f"gimbal pitch_aim fixed at {self.pitch_aim} (manual)")
            return
        target = self.gt(TARGET)["pos"]
        drone = self.gt()["pos"]
        wanted = math.degrees(math.atan2(
            -(target[2] - drone[2]),
            math.hypot(target[0] - drone[0], target[1] - drone[1])))
        deadline = time.monotonic() + 8
        boresight = None
        while time.monotonic() < deadline:
            boresight = self.gimbal_boresight_elevation()
            if boresight is not None:
                break
            time.sleep(0.2)
        if boresight is None:
            raise RuntimeError("no fresh /fpv/gimbal AIM feedback")
        log(f"gimbal boresight {boresight:.1f} deg, balloon elevation {wanted:.1f} deg")
        if abs(wanted - boresight) <= 2.0:
            log("boresight within 2 deg of balloon; no pitch_aim correction")
            return
        pairs = [(self.pitch_aim if self.pitch_aim is not None else -15.0, boresight)]
        slope = 1.0
        for iteration in range(3):
            p0, b0 = pairs[-1]
            if abs(slope) < 1e-3:
                raise RuntimeError("AIM pitch is at a joint limit; calibration cannot converge")
            p1 = (wanted - 90.0 if iteration == 0 else p0 + (wanted - b0) / slope)
            p1 = max(-120.0, min(60.0, p1))
            self.pitch_aim = p1
            log(f"gimbal calibration iter {iteration + 1}: pitch_aim={p1:.1f}")
            if not self.restart_bridge():
                raise RuntimeError("bridge did not come back after pitch_aim change")
            time.sleep(1.0)
            b1 = self.gimbal_boresight_elevation()
            if b1 is None:
                raise RuntimeError("no /fpv/gimbal feedback after bridge restart")
            pairs.append((p1, b1))
            log(f"  boresight now {b1:.1f} deg (wanted {wanted:.1f})")
            if abs(wanted - b1) <= 2.0:
                log(f"gimbal calibration converged: pitch_aim={p1:.1f}, boresight {b1:.1f}")
                return
            if abs(p1 - p0) > 1e-3:
                slope = (b1 - b0) / (p1 - p0)
        raise RuntimeError(f"gimbal calibration did not converge: {pairs}")

    def arm_fc(self):
        """Arm via the pilot RC ARM switch (AUX3), NOT the /fpv/arm host request.

        A held host arm request makes the bridge stream 0x20 frames (arm level
        semantics), which engages OFFBOARD authority immediately — the FC then
        ignores RC sticks and the ground yaw alignment cannot rotate the
        airframe. Pilot-switch arming keeps stick authority until the
        interceptor's first ControlCommand arrives and OFFBOARD takes over.
        """
        self.stage = "arm"
        self.rc.set(roll=1500, pitch=1500, yaw=1500, throttle=1000,
                    mode6=joystick.MODE6_VALUES_US[0 if self.launcher is not None else 1],
                    switch3=joystick.RC_MAX_US,
                    arm=joystick.RC_MIN_US, trigger=joystick.RC_MIN_US)
        self.wait("FC arming checks clear with ARM low",
                  lambda s: s["flags"] == 0, timeout=60, dwell=.5)
        self.rc.set(arm=joystick.RC_MAX_US)
        try:
            state = self.wait("pilot-switch arm accepted (props spinning)",
                              lambda s: 0 in s["modes"], timeout=12)
        except AssertionError:
            _, flags = self.modes()
            raise RuntimeError(f"arming failed; arming-disable flags=0x{flags:08x}")
        self.armed_once = True
        log(f"armed, modes={state['modes']} (props up)")

    def start_tracking(self):
        self.stage = "track"
        self.capture_png("0", "ground_camera.png")
        if self.launcher is not None:
            self.capture_png("LauncherOverview", "launcher_overview.png")
        self.ros_iface.publish_trigger("ON")
        first = self.sample()
        yaw_start = self.launcher.yaw if self.launcher else None
        pitch_start = self.launcher.pitch if self.launcher else None
        dynamic = getattr(self.args, "balloon_motion", "hover") == "orbit"
        evidence = {"target_displacement_m": 0.0, "target_peak_speed_m_s": 0.0,
                    "launcher_yaw_change_deg": 0.0, "launcher_pitch_change_deg": 0.0,
                    "max_locked_camera_error_deg": 0.0}
        deadline, since = time.monotonic() + TRACK_LOCK_WAIT, None
        while time.monotonic() < deadline:
            state = self.sample()
            speed = math.hypot(*state["target_vel"])
            displacement = math.dist(state["target_pos"], first["target_pos"])
            evidence["target_displacement_m"] = max(evidence["target_displacement_m"], displacement)
            evidence["target_peak_speed_m_s"] = max(evidence["target_peak_speed_m_s"], speed)
            if self.launcher:
                evidence["launcher_yaw_change_deg"] = max(evidence["launcher_yaw_change_deg"],
                    abs(math.degrees(self.launcher.yaw-yaw_start)))
                evidence["launcher_pitch_change_deg"] = max(evidence["launcher_pitch_change_deg"],
                    abs(math.degrees(self.launcher.pitch-pitch_start)))
            camera_ok = self.launcher is None or (
                state["camera_error_deg"] is not None and state["camera_error_deg"] < 5)
            moving = not dynamic or speed > .5 * self.args.balloon_orbit_speed
            if self.ros_iface.tracker_state == 2 and camera_ok and moving:
                if state["camera_error_deg"] is not None:
                    evidence["max_locked_camera_error_deg"] = max(
                        evidence["max_locked_camera_error_deg"], state["camera_error_deg"])
                if since is None:
                    since = time.monotonic()
                if time.monotonic() - since >= (3.0 if dynamic else 1.0) and (
                        not dynamic or evidence["target_displacement_m"] > .5):
                    self.report["phases"]["prelaunch_tracking"] = evidence
                    log(f"tracker locked with {'moving' if dynamic else 'static'} target; "
                        f"platform tracking={evidence}")
                    return
            else:
                since = None
            time.sleep(0.2)
        raise RuntimeError(f"tracking/aim readiness failed; tracker_state={self.ros_iface.tracker_state}, "
                           f"motion={self.args.balloon_motion}; "
                           f"see {self.workdir}/tracker.log")

    def capture_png(self, camera, filename):
        for _ in range(3):
            image = self.client.simGetImages(
                [airsim.ImageRequest(camera, airsim.ImageType.Scene, False, True)],
                vehicle_name=VEHICLE)[0]
            data = bytes(image.image_data_uint8)
            if image.width > 0 and image.height > 0 and data.startswith(b"\x89PNG\r\n\x1a\n"):
                if camera == "0" and (image.width, image.height) != (self.args.img_w, self.args.img_h):
                    raise RuntimeError(f"vision image shape {image.width}x{image.height} differs from configured intrinsics")
                (self.workdir / filename).write_bytes(data)
                return
            time.sleep(.05)
        raise RuntimeError(f"camera {camera} did not provide a valid RGB PNG")

    def check_launcher_clearance(self):
        self.client.simSetCameraFov("0", self.args.hfov, vehicle_name=VEHICLE)
        self.capture_png("0", "aim_camera.png")
        ranges = {name: self.client.getDistanceSensorData(
            "launcher_range_" + name, vehicle_name=VEHICLE).distance for name in ("center", "left", "right")}
        if not all(math.isfinite(d) and d > 0 for d in ranges.values()):
            raise RuntimeError("invalid launcher collision-ray distances")
        clearance = min(ranges.values())
        self.report["phases"]["clearance"] = {"collision_rays_m": ranges, "half_width_m": 1.0}
        if clearance < 20:
            raise RuntimeError(f"near obstacle in launcher sight cone: {clearance:.1f} m")
        log(f"launcher three-ray collision corridor clear for {clearance:.1f} m")

    def publish_hover_command(self, thrust):
        msg = ControlCommand()
        msg.header.stamp = self.ros_iface.get_clock().now().to_msg()
        msg.control_mode = ControlCommand.BODY_RATES
        msg.armed = True
        msg.bodyrates.x = 0.0
        msg.bodyrates.y = 0.0
        msg.bodyrates.z = 0.0
        msg.collective_thrust = thrust
        self.ros_iface.control_pub.publish(msg)

    def hold_launch_hover(self, state):
        height = self.origin[2] - state["pos"][2]
        thrust = max(4.0, min(14.0, 9.81 + 1.5 * (4.0 - height) + 2.5 * state["vel"][2]))
        self.publish_hover_command(thrust)

    def hover_handover(self):
        """Lift the armed drone to a low hover over the same OFFBOARD 0x20
        stream the interceptor will use, then hand over at MISSION.

        Launching the interceptor's guidance from the ground trips its gyro
        collision detector within the first tenths of a second: the initial
        yaw-rate commands jerk the airframe against ground friction (observed
        |gyro-cmd| = 4.4 rad/s -> false COMPLETED, run 2026-10-13). A short
        rate-zero, thrust-managed climb to ~4 m gives the guidance a clean
        airborne start; the bridge's command cache makes the interceptor's
        first frame take over seamlessly.
        """
        self.stage = "takeoff"
        log("hover takeover: climbing to 4 m via OFFBOARD bodyrate/thrust")
        start = time.monotonic()
        deadline = start + 25
        since = None
        while time.monotonic() < deadline:
            state = self.sample()
            height = self.origin[2] - state["pos"][2]
            self.hold_launch_hover(state)
            if abs(height - 4.0) < 0.4 and abs(state["vel"][2]) < 0.3:
                since = since or time.monotonic()
                if time.monotonic() - since >= 1.0:
                    break
            else:
                since = None
            if max(abs(state["rpy"][0]), abs(state["rpy"][1])) > 45:
                raise RuntimeError(f"takeover attitude upset: {state['rpy'][:2]}")
            time.sleep(0.05)
        else:
            raise RuntimeError("hover takeover did not settle at 4 m")
        log(f"hover at {self.origin[2] - self.gt()['pos'][2]:.1f} m; "
            f"tracker={self.ros_iface.tracker_state}")
        self.airborne_timestamp = self.client.getMultirotorState(vehicle_name=VEHICLE).timestamp

    def run_intercept(self):
        self.stage = "mission"
        self.min_distance = None
        # Hold the hover while the vision lock re-establishes (spin-up shake
        # can drop it for a moment); the mission starts from a solid track.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and self.ros_iface.tracker_state != 2:
            if self.launcher is None:
                self.hold_launch_hover(self.sample())
            time.sleep(0.05)
        if self.ros_iface.tracker_state != 2:
            raise RuntimeError(f"vision lock did not recover at handover "
                               f"(tracker_state={self.ros_iface.tracker_state})")
        control_before_launch = self.ros_iface.control_sequence
        self.ros_iface.publish_trigger("MISSION")
        start = time.monotonic()
        airborne_deadline = start + 20
        result = {"took_off": False, "terminated_by_node": False}
        progress_mark = (start, self.min_distance)
        lost_since = None
        bad_view_since = None
        next_log = 0.0
        peak_speed = 0.0
        peak_horizontal_speed, peak_climb_speed = 0.0, 0.0
        target_start, target_previous = None, None
        target_path = 0.0
        launch_timestamp = self.client.getMultirotorState(vehicle_name=VEHICLE).timestamp
        while time.monotonic() - start < MISSION_WAIT:
            state = self.sample()
            if "target_vel" in state:
                result["target_peak_speed_m_s"] = max(
                    result.get("target_peak_speed_m_s", 0), math.hypot(*state["target_vel"]))
                target_start = state["target_pos"] if target_start is None else target_start
                if target_previous is not None:
                    target_path += math.dist(state["target_pos"], target_previous)
                target_previous = state["target_pos"]
                result["target_path_length_m"] = target_path
                result["target_displacement_m"] = math.dist(state["target_pos"], target_start)
            peak_speed = max(peak_speed, math.hypot(*state["vel"]))
            result["peak_speed_m_s"] = peak_speed
            peak_horizontal_speed = max(peak_horizontal_speed, math.hypot(*state["vel"][:2]))
            peak_climb_speed = max(peak_climb_speed, -state["vel"][2])
            result["peak_horizontal_speed_m_s"] = peak_horizontal_speed
            result["peak_climb_speed_m_s"] = peak_climb_speed
            if self.launcher is not None:
                if not self.launcher.released and self.ros_iface.control_sequence > control_before_launch:
                    self.launcher.release()
                    result["launcher_release_s"] = time.monotonic() - start
                    log("launcher released on first interceptor body-rate command")
                if self.launcher.released and not self.launcher.stowed and \
                        math.dist(state["pos"], self.origin) > 5.0:
                    self.launcher.stow()
                    result["launcher_stow_s"] = time.monotonic() - start
                    result["launcher_stow_distance_m"] = math.dist(state["pos"], self.origin)
                    log(f"launcher stowed after {result['launcher_stow_distance_m']:.1f} m separation")
            height = self.origin[2] - state["pos"][2]
            distance = state["distance"]
            elapsed = time.monotonic() - start
            collision = self.client.simGetCollisionInfo(vehicle_name=VEHICLE)
            if (collision.has_collided and collision.time_stamp > launch_timestamp
                    and collision.object_name != TARGET
                    and not (collision.object_name.startswith("BFLauncher_") and height < 1)):
                self.hard_collision = True
                self.report["phases"]["collision"] = {
                    "object": collision.object_name, "time_s": elapsed, "position": state["pos"]}
                self.ros_iface.publish_trigger("OFF")
                self.rc.set(arm=joystick.RC_MIN_US, throttle=1000, switch3=joystick.RC_MIN_US)
                self.ros_iface.publish_offboard(False)
                raise RuntimeError(f"non-target collision with {collision.object_name}; emergency stop")
            if getattr(self.args, "launcher", False) and self.ros_iface.gimbal_quat is not None:
                error_deg = camera_target_error_degrees(
                    self.ros_iface.gimbal_quat,
                    tuple(t-p for t, p in zip(state["target_pos"], state["pos"])))
                result["peak_camera_error_deg"] = max(result.get("peak_camera_error_deg", 0), error_deg)
                if distance > 12:
                    result["peak_midcourse_camera_error_deg"] = max(
                        result.get("peak_midcourse_camera_error_deg", 0), error_deg)
                vertical_half_fov = math.degrees(math.atan(
                    math.tan(math.radians(self.args.hfov) / 2) * self.args.img_h / self.args.img_w))
                if distance > 12 and error_deg > vertical_half_fov + 3:
                    bad_view_since = bad_view_since or time.monotonic()
                    if time.monotonic() - bad_view_since > .5:
                        raise RuntimeError(f"MISSION camera points away from target: {error_deg:.1f} deg; "
                                           "ending engagement before blind flight")
                else:
                    bad_view_since = None
            if self.ros_iface.tracker_state != 2:
                lost_since = lost_since or time.monotonic()
                if time.monotonic() - lost_since >= 2.0:
                    log(f"target lost for 2 s; closest approach {self.min_distance:.2f} m")
                    result["outcome"] = "lost"
                    result["closest_distance_m"] = self.min_distance
                    result["duration_s"] = elapsed
                    return result
            else:
                lost_since = None
            if result["took_off"] and distance > self.min_distance + 8.0:
                log(f"missed pass: range reopened to {distance:.1f} m "
                    f"from {self.min_distance:.2f} m; ending engagement")
                result["outcome"] = "opening_range"
                result["closest_distance_m"] = self.min_distance
                result["duration_s"] = elapsed
                return result
            # Stagnation watchdog: no closing progress for 20 s while tracking is
            # up means the guidance is circling/blind — end it and go home.
            if result["took_off"] and elapsed > 25:
                mark_time, mark_dist = progress_mark
                if self.min_distance is not None and mark_dist is not None \
                        and mark_dist - self.min_distance < 1.0 \
                        and time.monotonic() - mark_time >= 20:
                    raise RuntimeError(f"intercept stagnated at min distance "
                                       f"{self.min_distance:.1f} m for 20 s")
                if time.monotonic() - mark_time >= 20 or \
                        (self.min_distance is not None and mark_dist is not None
                         and mark_dist - self.min_distance >= 1.0):
                    progress_mark = (time.monotonic(), self.min_distance)
            if self.min_distance is not None and self.min_distance <= HIT_DISTANCE:
                log(f"INTERCEPT: closest approach {self.min_distance:.2f} m "
                    f"(t+{elapsed:.1f}s, height {height:.1f} m)")
                result["outcome"] = "hit"
                result["closest_distance_m"] = self.min_distance
                result["duration_s"] = elapsed
                return result
            if self.ros_iface.mission_state is False:
                result["terminated_by_node"] = True
                outcome = ("hit" if self.min_distance <= HIT_DISTANCE
                           else "lost")
                log(f"mission terminated by interceptor: {outcome} "
                    f"(min distance {self.min_distance:.2f} m)")
                result["outcome"] = outcome
                result["closest_distance_m"] = self.min_distance
                result["duration_s"] = elapsed
                return result
            if not result["took_off"]:
                if height > 2.0:
                    result["took_off"] = True
                    self.airborne_timestamp = self.client.getMultirotorState(vehicle_name=VEHICLE).timestamp
                    log(f"airborne at t+{elapsed:.1f}s (height {height:.1f} m)")
                elif time.monotonic() > airborne_deadline:
                    raise RuntimeError("interceptor never took off after MISSION trigger")
            horizontal = math.hypot(state["pos"][0] - self.origin[0],
                                    state["pos"][1] - self.origin[1])
            if horizontal > 300 or height > 150 or max(abs(state["rpy"][0]),
                                                       abs(state["rpy"][1])) > 85:
                raise RuntimeError(f"safety envelope exceeded: horiz={horizontal:.0f} m "
                                   f"height={height:.0f} m attitude={state['rpy'][:2]}")
            if elapsed >= next_log:
                log(f"t+{elapsed:5.1f}s alt {height:5.1f} m dist {distance:5.1f} m "
                    f"min {self.min_distance:5.1f} m tracker={self.ros_iface.tracker_state} "
                    f"mission={self.ros_iface.mission_state}")
                next_log += 5.0
            time.sleep(0.05)
        raise RuntimeError("mission timeout: intercept did not conclude")

    def return_home(self):
        self.stage = "rth"
        if self.launcher is not None and not self.launcher.released and self.launcher.pivot is not None:
            self.rc.set(arm=joystick.RC_MIN_US, throttle=1000)
            self.report["phases"]["rth"] = {"outcome": "not-launched-home"}
            return
        self.ros_iface.publish_trigger("OFF")
        self.rc.set(roll=1500, pitch=1500, yaw=1500, switch3=joystick.RC_MIN_US)
        self.rc.set_mode("GPSRESCUE")
        time.sleep(0.2)
        self.ros_iface.publish_offboard(False)
        log("one-key RTH engaged (mode dial sixth detent, OFFBOARD released)")
        try:
            state = self.wait("GPS Rescue controller engaged",
                              lambda s: rescue_controller(s["modes"]) is not None, timeout=20)
        except AssertionError:
            state = self.sample()
            if 0 not in state["modes"]:
                log("drone already disarmed before RTH engaged; nothing to return")
                raise RuntimeError("disarmed before RTH could engage") from None
            raise
        start = time.monotonic()
        closest = None
        ground_contact = False
        ground_contact_info = None
        next_log = 0.0
        lost_controller_since = None
        while time.monotonic() - start < RTH_WAIT:
            state = self.sample()
            if rescue_controller(state["modes"]) is None and 0 in state["modes"]:
                lost_controller_since = lost_controller_since or time.monotonic()
                if time.monotonic() - lost_controller_since > 2:
                    raise RuntimeError("GPS Rescue aborted; stopping instead of continuing blind flight")
            else:
                lost_controller_since = None
            distance = math.hypot(state["pos"][0] - self.origin[0],
                                  state["pos"][1] - self.origin[1])
            closest = distance if closest is None else min(closest, distance)
            height = self.origin[2] - state["pos"][2]
            velocity = math.hypot(*state["vel"])
            collision = self.client.simGetCollisionInfo(vehicle_name=VEHICLE)
            contact_at_home = (
                self.airborne_timestamp is not None
                and collision.has_collided
                and collision.time_stamp > self.airborne_timestamp
                and collision.normal.z_val < -0.5
                and math.hypot(collision.position.x_val - self.origin[0],
                               collision.position.y_val - self.origin[1]) < 8
                and abs(collision.position.z_val - self.origin[2]) < 1)
            if contact_at_home:
                ground_contact = True
                ground_contact_info = {
                    "object": collision.object_name, "timestamp_ns": collision.time_stamp,
                    "position_ned_m": (collision.position.x_val, collision.position.y_val,
                                       collision.position.z_val)}
            if time.monotonic() - start >= next_log:
                log(f"RTH t+{time.monotonic() - start:5.1f}s home {distance:5.1f} m "
                    f"alt {height:5.1f} m |v| {velocity:4.1f} m/s modes={state['modes']}")
                next_log += 5.0
            if 0 not in state["modes"]:
                if distance < 8 and abs(height) < 1 and velocity < 0.3 and ground_contact:
                    log(f"RTH complete: home {distance:.1f} m, |v| {velocity:.2f} m/s, disarmed")
                    self.report["phases"]["rth"] = {
                        "outcome": "returned", "home_distance_m": distance,
                        "height_m": height, "speed_m_s": velocity,
                        "auto_disarmed": True, "ground_contact": True,
                        "ground_contact_info": ground_contact_info,
                        "duration_s": time.monotonic() - start}
                    return
                raise RuntimeError(f"disarmed before home landing: home={distance:.1f} m "
                                   f"height={height:.1f} m speed={velocity:.2f} m/s")
            time.sleep(0.2)
        raise RuntimeError(f"RTH did not conclude (closest home distance {closest:.1f} m)")

    def teardown(self, error=None):
        stage_reached = self.stage
        self.stage = "cleanup"
        with contextlib.suppress(Exception):
            if self.rc is not None:
                self.rc.set(arm=joystick.RC_MIN_US, throttle=1000, yaw=1500)
                self.rc.set_mode("ACRO")
        with contextlib.suppress(Exception):
            self.ros_iface.publish_arm(False)
        with contextlib.suppress(Exception):
            self.ros_iface.publish_offboard(False)
        if self.launcher is not None and not self.launcher.released and self.launcher.pivot is not None:
            try:
                self.launcher.release()
                self.launcher.stow()
                position = (*self.launcher.pivot[:2], self.launcher.pivot[2]-self.launcher.MOUNT_OFFSET)
                self.launcher.set_vehicle_pose(position, quaternion_rpy(0, 0, self.launcher.heading))
            except Exception as exc:
                log(f"launcher cleanup failed: {exc}")
                error = error or exc
        if self.armed_once:
            with contextlib.suppress(Exception):
                self.client.enableApiControl(False, vehicle_name=VEHICLE)
        time.sleep(0.5)
        for process in reversed(self.processes):
            try:
                process.stop()
            except Exception as exc:
                log(f"{process.name} cleanup failed: {exc}")
                error = error or exc
        if self.world_ready and self.balloon is not None:
            try:
                park_target(self.client)
                self.report["phases"]["target_cleanup"] = {"parked": True, "api_control": False}
            except Exception as exc:
                log(f"target cleanup failed: {exc}")
                self.report["phases"]["target_cleanup"] = {"parked": False, "error": str(exc)}
                error = error or exc
        if self.rc is not None:
            with contextlib.suppress(Exception):
                self.rc.close()
        if self.sitl is not None:
            control.stop_process(self.sitl)
        with contextlib.suppress(Exception):
            rclpy.shutdown()
        if self.trajectory_file:
            self.trajectory_file.close()
        if self.client is not None:
            try:
                self.client.client.close()
            except Exception as exc:
                log(f"AirSim client cleanup failed: {exc}")
                error = error or exc
        if error is not None:
            self.report["passed"] = False
        self.report["error"] = str(error) if error else None
        self.report["min_distance"] = self.min_distance
        self.report["stage_reached"] = stage_reached
        if self.workdir is not None and self.workdir.is_dir():
            with open(self.workdir / "report.json", "w") as f:
                json.dump(self.report, f, indent=2)
            log(f"report: {self.workdir}/report.json (error={error})")

    def run(self):
        self.t0 = time.monotonic()
        error = None
        try:
            self.preflight()
            self.start_sitl()
            self.start_ros()
            if self.args.balloon_script:
                self.launch_balloon()
            else:
                log("external balloon; skipping balloon launch")
            self.align_ground_yaw()
            self.calibrate_gimbal_aim()
            if self.launcher is not None:
                self.check_launcher_clearance()
            if self.args.aim_only:
                self.start_tracking()
                self.report["phases"]["aim_check"] = {"passed": True}
            else:
                self.arm_fc()
                self.post_arm_yaw_check()
                self.start_tracking()
                if self.launcher is None:
                    self.hover_handover()
                result = self.run_intercept()
                self.report["phases"]["intercept"] = result
        except Exception as exc:  # noqa: BLE001 - reported and re-raised after teardown
            error = exc
            log(f"FAIL: {exc}")
        finally:
            if self.armed_once and not self.hard_collision:
                try:
                    self.return_home()
                except Exception as exc:
                    log(f"RTH FAIL: {exc}")
                    self.report["phases"]["rth"] = {"outcome": "failed", "error": str(exc)}
                    error = error or exc
            else:
                log("collision emergency stop; RTH inhibited" if self.hard_collision
                    else "never armed; skipping RTH")
            self.report["check_type"] = "aim-only" if self.args.aim_only else "intercept"
            self.report["passed"] = (
                error is None and self.report["phases"].get("aim_check", {}).get("passed", False)
                if self.args.aim_only else mission_passed(self.report["phases"], error))
            self.teardown(error)
        return 0 if self.report.get("passed") else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True,
                        help="betaflight_SITL.elf to run")
    parser.add_argument("--eeprom", type=Path, default=Path("eeprom.bin"),
                        help="clone the tuned simulation EEPROM into the run directory")
    parser.add_argument("--ros-ws", type=Path, default=Path("/media/nick/data/workspace/fpv_ws"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--rpc-wait", type=float, default=30,
                        help="bounded wait for UE/AirSim RPC startup (seconds; never restarts UE)")
    parser.add_argument("--workdir", help="artifact directory (default /tmp/bf_intercept_<pid>)")
    parser.add_argument("--balloon-script", type=lambda value: Path(value) if value else None,
                        default=Path(__file__).resolve().parent / "airsim_balloon_target.py",
                        help="balloon motion script; empty string to skip (external balloon)")
    parser.add_argument("--balloon-x", type=float, default=60.0)
    parser.add_argument("--balloon-y", type=float, default=36.0)
    parser.add_argument("--balloon-z", type=float, default=-45.0)
    parser.add_argument("--balloon-motion", choices=("hover", "orbit"), default="hover")
    parser.add_argument("--balloon-orbit-radius", type=float, default=5.0,
                        help="dynamic target circle radius about balloon-x/y/z (m)")
    parser.add_argument("--balloon-orbit-speed", type=float, default=1.5,
                        help="dynamic target tangential speed (m/s)")
    parser.add_argument("--launch-x", type=float, default=0.0,
                        help="ground launch X in Copter-local NED (m)")
    parser.add_argument("--launch-y", type=float, default=0.0,
                        help="ground launch Y in Copter-local NED (m)")
    parser.add_argument("--launcher", action="store_true",
                        help="use a Camera8-referenced auto-aiming virtual platform")
    parser.add_argument("--aim-only", action="store_true",
                        help="verify mounted pose, clearance and vision lock without arming or releasing")
    parser.add_argument("--reference-camera", default="CineCameraActor_0",
                        help="UE actor name for Camera8 (its editor label is Camera8)")
    parser.add_argument("--launcher-forward", type=float, default=8.0,
                        help="launcher distance ahead of the reference camera (m)")
    parser.add_argument("--guidance-speed", type=float, default=20.0,
                        help="interceptor desired closing speed (m/s)")
    parser.add_argument("--climb-speed", type=float, default=20.0,
                        help="climb/descent speed limit (m/s)")
    parser.add_argument("--turn-cap-factor", type=float, default=.8,
                        help="turn-radius speed governor, [0.3,1.0]; lower brakes earlier")
    parser.add_argument("--rate-control-attenuation", action=argparse.BooleanOptionalAction,
                        default=False, help="body-rate attenuation (off for Betaflight SITL)")
    parser.add_argument("--pitch-aim", type=float, default=None,
                        help="override bridge gimbal.pitch_aim (deg) if detection needs it")
    parser.add_argument("--hfov", type=float, default=72.0)
    parser.add_argument("--img-w", type=int, default=1536)
    parser.add_argument("--img-h", type=int, default=960)
    args = parser.parse_args()
    try:
        validate_motion(args.balloon_motion, args.balloon_orbit_radius, args.balloon_orbit_speed)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.rpc_wait) or args.rpc_wait <= 0:
        parser.error("--rpc-wait must be finite and positive")
    if args.launcher:
        if args.pitch_aim is not None and args.pitch_aim != -10.0:
            parser.error("virtual launcher uses fixed --pitch-aim=-10")
        args.pitch_aim = -10.0
    if not 0 < args.guidance_speed <= 40 or not 0 < args.climb_speed <= 25:
        parser.error("guidance speed must be in (0,40], climb speed in (0,25]")
    if not .3 <= args.turn_cap_factor <= 1:
        parser.error("--turn-cap-factor must be in [0.3,1.0]")
    if not args.binary.is_file() or not args.eeprom.is_file():
        parser.error("--binary and --eeprom must exist")
    if args.workdir and Path(args.workdir).exists():
        parser.error("--workdir must be a new directory")
    args.binary = args.binary.resolve()
    args.eeprom = args.eeprom.resolve()
    args.ros_ws = args.ros_ws.resolve()
    if args.balloon_script is not None:
        args.balloon_script = args.balloon_script.resolve()
    return InterceptMission(args).run()


if __name__ == "__main__":
    sys.exit(main())
