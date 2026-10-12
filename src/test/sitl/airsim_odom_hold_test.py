"""Compare native AirSim POS HOLD/ALT HOLD odometry against ground truth.

Requires a sourced ROS workspace, a freshly started idle custom-link bridge/SITL,
no competing controllers, and the standard AUX1 mode/AUX3 arm mapping.
Control uses AirSim RC -> custom-link 0x23; no MSP or API OFFBOARD is used.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import time
from typing import TypedDict

import cosysairsim as airsim
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import GPSRAW, RCIn, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class Metrics(TypedDict):
    samples: int
    truth_span_xyz: list[float]
    odom_position_error_rms_xyz: list[float]
    roll_pitch_span_deg: list[float]
    yaw_error_span_deg: float
    velocity_rms_xy: list[float]
    yaw_error_mean_deg: float


def euler(q):
    x, y, z, w = q
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1., min(1., 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Observer(Node):
    def __init__(self):
        super().__init__("native_hold_observer")
        self.state: State | None = None
        self.odom: Odometry | None = None
        self.imu: Imu | None = None
        self.rc: RCIn | None = None
        self.gps: GPSRAW | None = None
        self.times = {}
        for cls, topic, key, qos in (
                (State, "/mavros/state", "state", 10),
                (Odometry, "/fpv/odom", "odom", qos_profile_sensor_data),
                (Imu, "/mavros/imu/data", "imu", qos_profile_sensor_data),
                (RCIn, "/mavros/rc/in", "rc", qos_profile_sensor_data),
                (GPSRAW, "/mavros/gpsstatus/gpsraw", "gps", qos_profile_sensor_data)):
            self.create_subscription(cls, topic, lambda m, k=key: self.receive(k, m), qos)

    def receive(self, key, message):
        setattr(self, key, message)
        self.times[key] = time.monotonic()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hover-pwm", type=int, default=1211)
    parser.add_argument("--turn", action="store_true")
    parser.add_argument("--verify", action="store_true", help="enforce measured odometry/hover regression limits")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Observer()
    client = airsim.MultirotorClient(timeout_value=3)
    if client.isApiControlEnabled(vehicle_name="Copter"):
        raise RuntimeError("AirSim API control is already owned")
    rc = airsim.RCData(is_initialized=True, is_valid=True)
    stage = "preflight"
    origin = None
    yaw_origin = None
    start = time.monotonic()
    rows = []
    report: dict[str, object] = {"passed": False}
    metrics: dict[str, Metrics] = {}
    stream = (args.output / "flight.csv").open("w", newline="")
    writer = csv.writer(stream)
    writer.writerow(("time_s", "stage", "mode", "armed",
                     "truth_x", "truth_y", "truth_z", "truth_vx", "truth_vy", "truth_vz",
                     "odom_x", "odom_y", "odom_z", "pos_error_x", "pos_error_y", "pos_error_z",
                     "truth_roll", "truth_pitch_flu", "truth_yaw_cw", "imu_roll", "imu_pitch", "imu_yaw_enu",
                     "yaw_error_deg", "acc_x_flu", "acc_y_flu", "acc_z_flu",
                     "truth_ax_ned", "truth_ay_ned", "truth_az_ned", "gps_lat", "gps_lon", "gps_alt_cm",
                     "pose_var_x", "pose_var_y", "pose_var_z", "velocity_var_x", "velocity_var_y"))

    def pump(duration=.02):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.002)

    def send():
        rc.timestamp = round(time.time() * 1e9)
        client.client.call("moveByRC", {
            "timestamp": rc.timestamp, "pitch": rc.pitch, "roll": rc.roll,
            "throttle": rc.throttle, "yaw": rc.yaw,
            "switches": int(bool(rc.switch1)) | (int(bool(rc.switch3)) << 2),
            "is_initialized": True, "is_valid": True,
        }, "Copter")

    def sample():
        pump()
        send()
        k = client.simGetGroundTruthKinematics(vehicle_name="Copter")
        if not isinstance(k, airsim.KinematicsState):
            raise RuntimeError("AirSim returned invalid ground-truth kinematics")
        r, p, y = euler((k.orientation.x_val, k.orientation.y_val, k.orientation.z_val, k.orientation.w_val))
        if node.state is None or time.monotonic() - node.times.get("state", 0) > .5:
            raise RuntimeError("custom-link state unavailable")
        if node.imu is None or time.monotonic() - node.times.get("imu", 0) > .2:
            raise RuntimeError("custom-link IMU unavailable")
        q = node.imu.orientation
        ir, ip, iy = euler((q.x, q.y, q.z, q.w))
        position, velocity = k.position, k.linear_velocity
        truth = [0., 0., 0.]
        truth_velocity = [0., 0., 0.]
        odom = [float("nan")] * 3
        error = [float("nan")] * 3
        if origin is not None and yaw_origin is not None:
            c, s = math.cos(yaw_origin), math.sin(yaw_origin)
            n, e, d = position.x_val - origin[0], position.y_val - origin[1], position.z_val - origin[2]
            truth = [c * n + s * e, s * n - c * e, -d]
            truth_velocity = [c * velocity.x_val + s * velocity.y_val,
                              s * velocity.x_val - c * velocity.y_val, -velocity.z_val]
            if node.odom is not None:
                o = node.odom.pose.pose.position
                odom = [o.x, o.y, o.z]
                error = [a - b for a, b in zip(odom, truth)]
            if stage != "cleanup" and (truth[2] > 8 or math.hypot(*truth[:2]) > 25 or
                                       max(abs(r), abs(p)) > math.radians(35)):
                raise RuntimeError(f"flight envelope exceeded: truth={truth}, rpy={tuple(map(math.degrees,(r,p,y)))}")
        a = node.imu.linear_acceleration
        g = node.gps
        pcov = node.odom.pose.covariance if node.odom else [float("nan")] * 36
        vcov = node.odom.twist.covariance if node.odom else [float("nan")] * 36
        row = (time.monotonic() - start, stage, node.state.mode, node.state.armed,
               *truth, *truth_velocity, *odom, *error, math.degrees(r), -math.degrees(p),
               math.degrees(y), math.degrees(ir), math.degrees(ip), math.degrees(iy),
               math.degrees(angle_error(iy, math.pi / 2 - y)), a.x, a.y, a.z,
               k.linear_acceleration.x_val, k.linear_acceleration.y_val, k.linear_acceleration.z_val,
               g.lat if g else 0, g.lon if g else 0, g.alt if g else 0,
               pcov[0], pcov[7], pcov[14], vcov[0], vcov[7])
        writer.writerow(row)
        rows.append(row)
        stream.flush()
        return {"k": k, "truth": truth, "velocity": truth_velocity,
                "rpy": (r, p, y), "imu_rpy": (ir, ip, iy), "odom": odom, "error": error}

    def observe(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            state = sample()
        return state

    def wait(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = sample()
            if predicate(state):
                return state
        raise RuntimeError(f"timeout in stage {stage}; last state={state}")

    def summarize(name):
        selected = [r for r in rows if r[1] == name]
        if not selected:
            return
        def span(index):
            values = [float(r[index]) for r in selected]
            return max(values) - min(values)
        def rms(index):
            return math.sqrt(sum(float(r[index]) ** 2 for r in selected) / len(selected))
        result: Metrics = {"samples": len(selected), "truth_span_xyz": [span(i) for i in (4,5,6)],
                  "odom_position_error_rms_xyz": [rms(i) for i in (13,14,15)],
                  "roll_pitch_span_deg": [span(16),span(17)], "yaw_error_span_deg":span(22),
                  "velocity_rms_xy": [rms(7), rms(8)],
                  "yaw_error_mean_deg": sum(float(r[22]) for r in selected) / len(selected)}
        report[name] = result
        metrics[name] = result
        print(name, json.dumps(result), flush=True)

    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and (node.state is None or node.imu is None):
            pump(.1)
        assert node.state is not None and not node.state.armed
        assert node.count_publishers("/fpv/control_command") == 0
        assert node.count_publishers("/fpv/control_command_raw") == 0
        rc.throttle, rc.switch1, rc.switch3 = 0., 0, 0
        state = observe(3)
        if node.state.mode != "ACRO" or node.rc is None or node.rc.channels[4] > 1100:
            raise RuntimeError("RC mode commands are not being accepted; stop other RC feeders and restart idle SITL")
        k = state["k"]
        assert math.sqrt(sum(v*v for v in (k.linear_velocity.x_val,k.linear_velocity.y_val,k.linear_velocity.z_val))) < .2
        print("ground attitude:", {"truth_rpy_deg": list(map(math.degrees,state["rpy"])),
                                    "imu_rpy_deg": list(map(math.degrees,state["imu_rpy"]))}, flush=True)
        origin = (k.position.x_val, k.position.y_val, k.position.z_val)
        yaw_origin = state["rpy"][2]
        rc.switch3 = 1
        stage = "arming"
        wait(lambda _: node.state is not None and node.state.armed, 8)
        rc.switch1 = 1
        stage = "takeoff"
        rc.throttle = .45
        wait(lambda _: node.state is not None and "POS_HOLD" in node.state.mode and "ALT_HOLD" in node.state.mode, 5)
        wait(lambda s: s["truth"][2] >= 3, 15)
        rc.throttle = (1050 + 950 * (args.hover_pwm - 1000) / 1000 - 1000) / 1000
        stage = "hold_entry"
        wait(lambda _: node.state is not None and "POS_HOLD" in node.state.mode and "ALT_HOLD" in node.state.mode, 5)
        observe(5)
        stage = "hover_before"
        observe(15)
        summarize(stage)
        stage = "forward"
        rc.pitch = -.2
        observe(8)
        rc.pitch = 0.
        summarize(stage)
        stage = "brake"
        observe(5)
        stage = "hover_after"
        observe(20)
        summarize(stage)
        if args.turn:
            stage = "yaw_turn"
            rc.yaw = .2
            observe(2)
            rc.yaw = 0.
            stage = "hover_turned"
            observe(10)
            summarize(stage)
        stage = "landing"
        rc.throttle = 0.
        wait(lambda s: s["truth"][2] < .3 and abs(s["velocity"][2]) < .3, 20)
        rc.switch3 = 0
        wait(lambda _: node.state is not None and not node.state.armed, 3)
        if args.verify:
            for name in ("hover_before", "forward", "hover_after"):
                metric = metrics[name]
                assert metric["odom_position_error_rms_xyz"][0] < .12, (name, metric)
                assert metric["odom_position_error_rms_xyz"][1] < .03, (name, metric)
                assert metric["odom_position_error_rms_xyz"][2] < .2, (name, metric)
                assert abs(metric["yaw_error_mean_deg"]) < .5, (name, metric)
            metric = metrics["hover_after"]
            assert metric["truth_span_xyz"][0] < .25 and metric["truth_span_xyz"][1] < .10, metric
            assert metric["truth_span_xyz"][2] < .5, metric
            assert max(metric["roll_pitch_span_deg"]) < 1.0, metric
            assert max(metric["velocity_rms_xy"]) < .05, metric
            print("PASS: odometry accuracy and native hover regression limits", flush=True)
        report["passed"] = True
    finally:
        stage = "cleanup"
        rc.pitch = rc.roll = rc.yaw = 0.
        try:
            if node.state is not None and node.state.armed:
                rc.throttle = 0.
                rc.switch1 = 1
                if origin is not None:
                    wait(lambda s: s["truth"][2] < .3 and abs(s["velocity"][2]) < .3, 20)
        finally:
            rc.switch3 = rc.switch1 = 0
            rc.throttle = 0.
            send()
            pump(.5)
            stream.close()
            (args.output / "report.json").write_text(json.dumps(report, indent=2))
            client.client.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
