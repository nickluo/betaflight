#!/usr/bin/env python3
"""Control a real UE/Cosys-AirSim BetaFlight vehicle, not a harness plant.

AirSim owns physics and sends sensors over TCP 5763. This script starts its
own SITL, sends AETR RC on UDP 9004, and checks AirSim ground truth against
MSP. No FDM packets or synthetic IMU/GPS samples are generated.
"""

import argparse
import csv
from contextlib import contextmanager, ExitStack
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import TypedDict

import sitl_joystick as joystick


class FlightState(TypedDict):
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    rpy: tuple[float, float, float]
    fc_rpy: tuple[float, float, float]
    yaw_rate_dps: float
    modes: list[int]
    arming_flags: int


def log(message):
    print(f"[airsim-test] {message}", flush=True)


def angle_error(actual, expected):
    return (actual - expected + 180.0) % 360.0 - 180.0


def euler_degrees(q):
    """FRD -> NED quaternion: roll right, pitch up, heading clockwise."""
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    return (
        math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
        math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
        math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360,
    )


def decode_status(payload, box_ids):
    if len(payload) < 16:
        raise RuntimeError("truncated MSP_STATUS")
    offset = 16 + payload[15]
    if len(payload) < offset + 5:
        raise RuntimeError("MSP_STATUS has no arming-disable flags")
    flags = struct.unpack_from("<I", payload, 6)[0]
    modes = {box for index, box in enumerate(box_ids[:32]) if flags & (1 << index)}
    return modes, struct.unpack_from("<I", payload, offset + 1)[0]


class MspClient:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b""

    def request(self, command):
        self.sock.sendall(b"$M<" + bytes((0, command, command)))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            start = self.buffer.find(b"$M")
            if start >= 0 and len(self.buffer) >= start + 5:
                size, reply_command = self.buffer[start + 3:start + 5]
                end = start + 6 + size
                if len(self.buffer) >= end:
                    frame = self.buffer[start:end]
                    self.buffer = self.buffer[end:]
                    checksum = 0
                    for value in frame[3:-1]:
                        checksum ^= value
                    if checksum != frame[-1]:
                        raise RuntimeError("MSP reply checksum mismatch")
                    if reply_command == command:
                        if frame[2:3] != b">":
                            raise RuntimeError(f"MSP command {command} rejected")
                        return frame[5:-1]
                    continue
            self.sock.settimeout(max(0.01, deadline - time.monotonic()))
            data = self.sock.recv(4096)
            if not data:
                raise RuntimeError("MSP connection closed")
            self.buffer += data
        raise TimeoutError(f"MSP command {command} timed out")


class RcStream:
    def __init__(self, host):
        self.destination = (host, 9004)
        self.channels = [joystick.RC_MID_US] * joystick.NUM_CHANNELS
        for role in ("throttle", "mode6", "switch3", "arm", "trigger"):
            self.channels[joystick.RC_CHANNEL_INDICES[role]] = joystick.RC_MIN_US
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.thread = threading.Thread(target=self.run, daemon=True)

    def set(self, **channels):
        indices = {**joystick.RC_CHANNEL_INDICES, "autopilot": 9}
        with self.lock:
            for name, value in channels.items():
                self.channels[indices[name]] = value

    def set_mode(self, mode):
        self.set(mode6=joystick.MODE6_VALUES_US[joystick.MODE6_NAMES.index(mode)])

    def run(self):
        start = time.monotonic()
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    packet = struct.pack("<d16H", time.monotonic() - start, *self.channels)
                self.sock.sendto(packet, self.destination)
                self.stop_event.wait(0.02)
        except OSError as exc:
            self.error = exc
            self.stop_event.set()

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.sock.close()


def configuration(gps, hover_pwm):
    latitude = gps.latitude + 20.0 / 111319.49
    return [
        "feature GPS",
        "set gps_provider = VIRTUAL",
        "set trust_mag = ON",
        "set small_angle = 45",
        "set failsafe_procedure = AUTO-LAND",
        "set custom_link_motors_stream = ON",
        f"set ap_hover_throttle = {hover_pwm}",
        *joystick.CLI_CONFIG_LINES,
        # Test-only mission switch: keep the radio's first eight channels unchanged.
        "aux 6 56 5 1700 2100 0 0",
        f"waypoint insert 0 {latitude:.7f} {gps.longitude:.7f} "
        f"{round((gps.altitude + 5) * 100)} 250 flyover 0 none",
    ]


def require_free_ports():
    for port in (5761, 5763):
        with socket.socket() as probe:
            probe.settimeout(0.3)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(f"TCP {port} is occupied; stop the existing SITL explicitly")

def require_quiet_rc():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 9004))
        probe.settimeout(0.3)
        try:
            probe.recvfrom(64)
        except socket.timeout:
            return
        raise RuntimeError("another RC sender is streaming on UDP 9004; stop the joystick bridge first")


def check_vehicle(client, vehicle, airsim):
    if not client.ping() or vehicle not in client.listVehicles():
        raise RuntimeError("AirSim RPC/vehicle unavailable; start UE simulation first")
    if client.isApiControlEnabled(vehicle_name=vehicle):
        raise RuntimeError("AirSim API control is enabled; disable it before this RC test")
    kinematics = client.simGetGroundTruthKinematics(vehicle_name=vehicle)
    if not isinstance(kinematics, airsim.KinematicsState):
        raise RuntimeError("AirSim returned invalid kinematics")
    velocity = kinematics.linear_velocity
    speed = math.sqrt(velocity.x_val ** 2 + velocity.y_val ** 2 + velocity.z_val ** 2)
    if not math.isfinite(speed) or speed > 0.3:
        raise RuntimeError("vehicle is moving before startup; choose a stationary launch "
                           "surface with working UE ground collisions")


class ControlTest:
    FIELDS = ("stage", "elapsed_s", "sim_timestamp", "north_m", "east_m", "down_m",
              "vn_mps", "ve_mps", "vd_mps", "roll_deg", "pitch_up_deg", "heading_deg",
              "fc_roll_deg", "fc_pitch_down_deg", "fc_heading_deg", "arming_flags", "modes",
              "yaw_rate_dps")

    def __init__(self, client, vehicle, msp, rc, writer):
        self.client, self.vehicle, self.msp, self.rc, self.writer = client, vehicle, msp, rc, writer
        self.box_ids = list(msp.request(119))
        self.start = time.monotonic()
        self.origin = None
        self.last_timestamp = None
        self.last_advance = time.monotonic()
        self.stage = "startup"
        self.last_sample = None

    def sample(self) -> FlightState:
        if self.rc.error is not None:
            raise RuntimeError(f"RC stream failed: {self.rc.error}")
        k = self.client.simGetGroundTruthKinematics(vehicle_name=self.vehicle)
        timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle).timestamp
        if timestamp != self.last_timestamp:
            self.last_timestamp, self.last_advance = timestamp, time.monotonic()
        elif time.monotonic() - self.last_advance > 3:
            raise RuntimeError("AirSim simulation clock stopped (is UE paused?)")
        pos = (k.position.x_val, k.position.y_val, k.position.z_val)
        vel = (k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val)
        rpy = euler_degrees(k.orientation)
        yaw_rate = math.degrees(k.angular_velocity.z_val)
        if not all(math.isfinite(value) for value in (*pos, *vel, *rpy, yaw_rate)):
            raise RuntimeError("non-finite AirSim ground truth")
        attitude = struct.unpack("<hhh", self.msp.request(108))
        fc = (attitude[0] / 10, attitude[1] / 10, float(attitude[2]))
        modes, flags = decode_status(self.msp.request(101), self.box_ids)
        self.writer.writerow((self.stage, time.monotonic() - self.start, timestamp,
                              *pos, *vel, *rpy, *fc, flags, ";".join(map(str, sorted(modes))),
                              yaw_rate))
        state: FlightState = {"position": pos, "velocity": vel, "rpy": rpy,
                              "fc_rpy": fc, "modes": sorted(modes), "arming_flags": flags,
                              "yaw_rate_dps": yaw_rate}
        self.last_sample = state
        if self.origin is not None and self.stage != "cleanup":
            height = self.origin[2] - pos[2]
            distance = math.hypot(pos[0] - self.origin[0], pos[1] - self.origin[1])
            if height > 15 or distance > 45 or max(abs(rpy[0]), abs(rpy[1])) > 45:
                raise AssertionError(f"safety envelope exceeded: height={height:.1f}, "
                                     f"distance={distance:.1f}, attitude={rpy}")
        return state

    def wait(self, description, predicate, timeout=30, dwell=0):
        deadline, since = time.monotonic() + timeout, None
        while time.monotonic() < deadline:
            state = self.sample()
            if predicate(state):
                if since is None:
                    since = time.monotonic()
                if time.monotonic() - since >= dwell:
                    log(f"PASS: {description}")
                    return state
            else:
                since = None
            time.sleep(0.1)
        raise AssertionError(f"timeout: {description}; last state={self.last_sample}")

    def observe(self, duration) -> FlightState:
        if duration <= 0:
            raise ValueError("observation duration must be positive")
        deadline = time.monotonic() + duration
        state = self.sample()
        while time.monotonic() < deadline:
            state = self.sample()
            time.sleep(0.1)
        return state

    def sensors(self):
        self.stage = "sensors"
        self.rc.set_mode("ANGLE")
        self.wait("AirSim sensors + RX ready", lambda s: s["arming_flags"] == 0, timeout=45)
        self.msp.request(205)
        self.wait("accelerometer calibration complete", lambda s: s["arming_flags"] == 0,
                  timeout=20, dwell=2)
        def aligned(s):
            r, p, y = s["rpy"]
            fr, fp, fy = s["fc_rpy"]
            return abs(fr - r) < 5 and abs(fp + p) < 5 and abs(angle_error(fy, y)) < 5
        self.wait("FC attitude matches AirSim (pitch inverted, same heading)", aligned,
                  timeout=25, dwell=3)
        gps = self.msp.request(106)
        if len(gps) < 2 or not gps[0] or gps[1] < 6:
            raise AssertionError(f"no GPS fix: {gps[:2]!r}")

    def takeoff(self, hover_pwm):
        self.stage = "takeoff"
        origin = self.sample()["position"]
        self.origin = origin
        self.rc.set(arm=joystick.RC_MAX_US)
        self.wait("armed", lambda s: 0 in s["modes"], timeout=8)
        self.rc.set(throttle=min(1900, hover_pwm + 40))
        self.wait("climbed 3 m in AirSim", lambda s: origin[2] - s["position"][2] > 3,
                  timeout=25)
        self.rc.set(throttle=hover_pwm)
        self.rc.set_mode("POSHOLD+ALTHOLD")
        self.wait("ALTHOLD + POSHOLD active", lambda s: {3, 11} <= set(s["modes"]))
        anchor = self.sample()["position"]
        self.stage = "hover"
        def settled(s):
            p, v = s["position"], s["velocity"]
            return math.dist(p[:2], anchor[:2]) < 3 and abs(p[2] - anchor[2]) < 2 \
                and math.hypot(v[0], v[1]) < 1 and abs(v[2]) < 0.6
        self.wait("stable hover for 5 s", settled, timeout=40, dwell=5)

    def directions(self):
        for axis, index in (("roll", 0), ("pitch", 1), ("yaw", 2)):
            self.stage = axis
            before = self.sample()
            self.rc.set_mode("ALTHOLD")
            self.rc.set(**{axis: 1600})
            try:
                after = self.observe(1)
                delta = (angle_error(after["rpy"][2], before["rpy"][2]) if axis == "yaw"
                         else after["rpy"][index] - before["rpy"][index])
                expected_delta = -delta if axis == "pitch" else delta
                if expected_delta < 3:
                    raise AssertionError(f"{axis} response wrong: delta={delta:.1f} deg")
                log(f"PASS: +{axis} RC response = {delta:+.1f} deg (AirSim)")
            finally:
                self.rc.set(**{axis: 1500})
                self.rc.set_mode("POSHOLD+ALTHOLD")
            self.wait(f"settled after {axis}", lambda s: max(abs(s["rpy"][0]), abs(s["rpy"][1])) < 5
                      and math.hypot(*s["velocity"][:2]) < 1, timeout=30, dwell=2)

    def mission(self):
        origin = self.origin
        if origin is None:
            raise RuntimeError("mission requires a takeoff origin")
        self.stage = "mission"
        self.rc.set(autopilot=joystick.RC_MAX_US)
        self.wait("AUTOPILOT + ALTHOLD + POSHOLD active",
                  lambda s: {56, 3, 11} <= set(s["modes"]))
        target = (origin[0] + 20, origin[1])
        self.wait("20 m north waypoint reached (AirSim ground truth)",
                  lambda s: math.dist(s["position"][:2], target) < 4, timeout=60)
        self.wait("parked near waypoint for 5 s",
                  lambda s: math.dist(s["position"][:2], target) < 6
                  and math.hypot(*s["velocity"][:2]) < 1, timeout=35, dwell=5)

    def land(self):
        origin = self.origin
        if origin is None:
            raise RuntimeError("landing requires a takeoff origin")
        self.stage = "landing"
        self.rc.set(autopilot=1000, throttle=1000)
        self.wait("ALT HOLD accepts full-low throttle while armed",
                  lambda s: {0, 3, 11} <= set(s["modes"]) and 56 not in s["modes"],
                  timeout=5)
        self.wait("landed", lambda s: origin[2] - s["position"][2] < 0.5
                  and abs(s["velocity"][2]) < 0.5, timeout=60, dwell=1)
        self.rc.set(arm=1000, throttle=1000)
        self.rc.set_mode("ACRO")
        self.wait("disarmed", lambda s: 0 not in s["modes"], timeout=5)


def provision(binary, directory, lines, name="config", log_name=None):
    config = directory / f"{name}.txt"
    config.write_text("\n".join(lines) + "\n")
    result = subprocess.run([str(binary), "--config", str(config.resolve())], cwd=directory,
                            capture_output=True, text=True, timeout=60)
    log_path = directory / f"{log_name or name}.log"
    log_path.write_text(result.stdout + result.stderr)
    if result.returncode or not (directory / "eeprom.bin").exists() \
            or "###ERROR" in result.stdout or "###ERROR" in result.stderr:
        raise RuntimeError(f"SITL provisioning failed; see {log_path}")
    return result.stdout


def stop_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def disarm_test(test):
    test.stage = "cleanup"
    test.rc.set(arm=1000, throttle=1000, yaw=1500, autopilot=1000)
    test.rc.set_mode("ACRO")
    try:
        if test.client.isApiControlEnabled(vehicle_name=test.vehicle):
            try:
                if not test.client.armDisarm(False, vehicle_name=test.vehicle):
                    raise RuntimeError("API cleanup disarm was not confirmed")
            finally:
                test.client.enableApiControl(False, vehicle_name=test.vehicle)
    finally:
        test.wait("cleanup disarm confirmed", lambda s: 0 not in s["modes"], timeout=5)
        time.sleep(0.3)


@contextmanager
def flight_session(binary, client, vehicle, directory, lines, stream_rc=True):
    require_free_ports()
    require_quiet_rc()
    with ExitStack() as stack:
        sitl_log = stack.enter_context((directory / "sitl.log").open("w"))
        trajectory = stack.enter_context((directory / "trajectory.csv").open("w", newline=""))
        provision(binary, directory, lines, log_name="provision")
        process = subprocess.Popen([str(binary)], cwd=directory, stdout=sitl_log, stderr=sitl_log)
        stack.callback(stop_process, process)
        deadline = time.monotonic() + 20
        sock = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("SITL exited during startup; see sitl.log")
            try:
                sock = socket.create_connection(("127.0.0.1", 5761), timeout=0.5)
                break
            except (ConnectionRefusedError, socket.timeout):
                time.sleep(0.2)
        if sock is None:
            raise TimeoutError("SITL MSP did not become responsive")
        stack.callback(sock.close)
        rc = RcStream("127.0.0.1")
        stack.callback(rc.close)
        if stream_rc:
            rc.thread.start()
        writer = csv.writer(trajectory)
        writer.writerow(ControlTest.FIELDS)
        test = ControlTest(client, vehicle, MspClient(sock), rc, writer)
        stack.callback(disarm_test, test)
        yield test


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--vehicle", default="Copter")
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host (SITL runs locally)")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--scenario", choices=("sensors", "hover", "mission", "all"), default="all")
    parser.add_argument("--hover-pwm", type=int, default=1590,
                        help="FC hover throttle; ~59%% for BetaFlightParams' 1 kg / 4 x 4.18 N")
    parser.add_argument("--output", type=Path, help="new artifact directory (must not already exist)")
    args = parser.parse_args()
    if not args.binary.is_file() or not 1100 <= args.hover_pwm <= 1700:
        parser.error("binary must exist and --hover-pwm must be between 1100 and 1700")
    try:
        import cosysairsim as airsim
        from msgpackrpc.error import RPCError
    except ImportError as exc:
        parser.error(f"Cosys-AirSim Python client required: {exc}")

    output = args.output or Path(tempfile.mkdtemp(prefix="bf_airsim_"))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    log(f"artifacts: {output}")
    test = None
    report = {"scenario": args.scenario, "vehicle": args.vehicle, "passed": False}
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port, timeout_value=5)
    try:
        require_free_ports()
        check_vehicle(client, args.vehicle, airsim)
        gps_data = client.getGpsData(vehicle_name=args.vehicle)
        if not isinstance(gps_data, airsim.GpsData):
            raise RuntimeError("AirSim returned invalid GPS data")
        lines = configuration(gps_data.gnss.geo_point, args.hover_pwm)
        with flight_session(args.binary.resolve(), client, args.vehicle, output, lines) as test:
            test.sensors()
            if args.scenario != "sensors":
                test.takeoff(args.hover_pwm)
                if args.scenario == "all":
                    test.directions()
                if args.scenario in ("mission", "all"):
                    test.mission()
                test.land()
        report["passed"] = True
    except (AssertionError, RuntimeError, OSError, TimeoutError, ValueError, RPCError) as exc:
        report["error"] = str(exc)
        if exc.__context__ is not None:
            report["caused_by"] = str(exc.__context__)
        log(f"FAIL: {exc}")
    finally:
        client.client.close()
        if test is not None:
            report["last_state"] = test.last_sample
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    log("PASS" if report["passed"] else "FAIL")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
