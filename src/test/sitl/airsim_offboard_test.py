#!/usr/bin/env python3
"""Exercise native AirSim API -> custom-link OFFBOARD with real UE physics.

No UDP RC or FDM packets are sent. A temporary EEPROM clones current tuning.
The small host-side attitude/altitude loop below sends only the implemented
body-rate/throttle API; takeoff/hover/position APIs are not used.
"""

import argparse
import json
import math
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time

import airsim_control_test as control
from sitl_joystick import CLI_CONFIG_LINES


def rate_throttle(roll_deg, pitch_up_deg, height, down_velocity, target_height,
                  hover, override=None):
    rates = [-math.radians(roll_deg) * 4, math.radians(pitch_up_deg) * 4, 0.0]
    if override is not None:
        axis, rate = override
        rates[axis] = rate
    rates = tuple(max(-0.5, min(0.5, rate)) for rate in rates)
    tilt = max(0.7, math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_up_deg)))
    throttle = hover / tilt + 0.05 * (target_height - height) + 0.10 * down_velocity
    return rates, max(0.30, min(0.78, throttle))


def assert_rate_response(samples, axis, command):
    if len(samples) < 3:
        raise AssertionError("too few OFFBOARD body-rate response samples")
    measured = sum(sample[axis] for sample in samples) / len(samples)
    expected = command if axis == 0 else -command  # FLU -> AirSim FRD
    gain = measured / expected
    if not 0.7 <= gain <= 1.3:
        raise AssertionError(f"axis {axis} rate sign/unit/tracking mismatch: "
                             f"expected {math.degrees(expected):.2f}, "
                             f"measured {math.degrees(measured):.2f} deg/s (gain={gain:.2f})")
    return {"axis": axis, "command_flu_rad_s": command,
            "expected_frd_deg_s": math.degrees(expected),
            "measured_frd_deg_s": math.degrees(measured), "gain": gain}


def touchdown_confirmed(collision, airborne_timestamp, velocity):
    return bool(collision.has_collided) and collision.time_stamp > airborne_timestamp \
        and collision.normal.z_val < -0.5 and math.sqrt(sum(v * v for v in velocity)) < 0.25


class OffboardTest:
    def __init__(self, test, hover):
        self.test, self.client, self.vehicle = test, test.client, test.vehicle
        self.hover = hover
        self.results = []

    def command(self, rates, throttle):
        future = self.client.moveByAngleRatesThrottleAsync(
            *rates, throttle, 0.02, vehicle_name=self.vehicle)
        if future.get() is not True:
            raise RuntimeError("AirSim rejected the OFFBOARD rate/throttle command")

    def drive(self, target_height, duration, override=None, require_settled=False):
        samples = []
        start = time.monotonic()
        origin = self.test.origin
        if origin is None:
            raise RuntimeError("OFFBOARD flight has no launch origin")
        while time.monotonic() - start < duration:
            state = self.test.sample()
            modes = set(state["modes"])
            if not {0, 58} <= modes or modes & {1, 2, 3, 11, 56}:
                raise RuntimeError(f"OFFBOARD lost priority or arming: {sorted(modes)}")
            height = origin[2] - state["position"][2]
            if height > 6 or math.dist(origin[:2], state["position"][:2]) > 8 \
                    or max(abs(state["rpy"][0]), abs(state["rpy"][1])) > 20:
                raise RuntimeError("OFFBOARD test safety envelope exceeded")
            rates, throttle = rate_throttle(state["rpy"][0], state["rpy"][1], height,
                                            state["velocity"][2], target_height, self.hover, override)
            self.command(rates, throttle)
            k = self.client.simGetGroundTruthKinematics(vehicle_name=self.vehicle)
            if time.monotonic() - start >= duration - 0.3:
                samples.append((k.angular_velocity.x_val, k.angular_velocity.y_val,
                                k.angular_velocity.z_val))
            time.sleep(0.02)
        state = self.test.sample()
        if require_settled:
            height = origin[2] - state["position"][2]
            if abs(height - target_height) > 0.6 or abs(state["velocity"][2]) > 0.5 \
                    or max(abs(state["rpy"][0]), abs(state["rpy"][1])) > 3:
                raise AssertionError(f"OFFBOARD host-loop hover did not settle: {state}")
        return samples

    def run(self):
        test = self.test
        test.sensors()
        payload = test.msp.request(105)
        if len(payload) < 16 or tuple(int.from_bytes(payload[i:i + 2], "little")
                                     for i in range(0, 16, 2)) != \
                (1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000):
            raise RuntimeError("native AirSim RC is not neutral; set RC.RemoteControlID=-1")
        test.origin = test.sample()["position"]
        test.stage = "offboard-enable"
        if not self.client.armDisarm(False, vehicle_name=self.vehicle):
            raise RuntimeError("initial API arm request reset was not confirmed")
        self.client.enableApiControl(True, vehicle_name=self.vehicle)
        self.command((0, 0, 0), 0)
        def switch_ready(_state):
            payload = test.msp.request(105)
            return len(payload) >= 14 and int.from_bytes(payload[10:12], "little") == 1800 \
                and int.from_bytes(payload[12:14], "little") == 1000
        test.wait("AirSim holds AUX2 OFFBOARD; AUX3 ARM remains low", switch_ready, timeout=8)
        if not self.client.armDisarm(True, vehicle_name=self.vehicle):
            raise RuntimeError("AirSim host arming was refused")
        test.wait("host arm + OFFBOARD active", lambda s: {0, 58} <= set(s["modes"]), timeout=8)
        test.wait("OFFBOARD active; firmware angle/altitude/position loops inactive",
                  lambda s: {0, 58} <= set(s["modes"])
                  and not ({1, 2, 3, 11, 56} & set(s["modes"])), timeout=5, dwell=1)

        test.stage = "offboard-takeoff"
        self.drive(2, 8, require_settled=True)
        airborne_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle).timestamp
        control.log("PASS: OFFBOARD takeoff/hover at 2 m using native rate/throttle API")
        for axis, rate, duration in ((2, 0.35, 1.5), (2, -0.35, 1.5),
                                     (0, 0.20, 0.6), (1, 0.20, 0.6)):
            test.stage = f"offboard-axis-{axis}-{rate:+.2f}"
            samples = self.drive(2, duration, (axis, rate))
            result = assert_rate_response(samples, axis, rate)
            self.results.append(result)
            control.log(f"PASS: axis {axis} {rate:+.2f} rad/s -> "
                        f"{result['measured_frd_deg_s']:+.2f} deg/s (AirSim FRD)")
            test.stage = "offboard-relevel"
            self.drive(2, 2, require_settled=True)

        test.stage = "offboard-land"
        origin = test.origin
        if origin is None:
            raise RuntimeError("OFFBOARD landing has no origin")
        deadline = time.monotonic() + 20
        start = time.monotonic()
        while time.monotonic() < deadline:
            self.drive(2 - 0.7 * (time.monotonic() - start), 0.25)
            state = test.sample()
            if origin[2] - state["position"][2] < -5:
                raise RuntimeError("descended below the launch surface without confirmed touchdown")
            collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle)
            if touchdown_confirmed(collision, airborne_timestamp, state["velocity"]):
                break
        else:
            raise AssertionError("OFFBOARD host-controlled landing timed out")
        self.command((0, 0, 0), 0)
        if not self.client.armDisarm(False, vehicle_name=self.vehicle):
            raise RuntimeError("API disarm was not confirmed")
        test.wait("API arm=0 disarms", lambda s: 0 not in s["modes"], timeout=5)
        test.observe(0.5)
        test.wait("arming guards clear after leaving hold switches",
                  lambda s: s["arming_flags"] == 0, timeout=8)

        test.stage = "offboard-disable"
        if not self.client.armDisarm(True, vehicle_name=self.vehicle):
            raise RuntimeError("OFFBOARD re-arm was refused")
        test.wait("OFFBOARD can re-arm after landing", lambda s: {0, 58} <= set(s["modes"]),
                  timeout=5)
        self.command((0, 0, 0), 0)
        state = test.observe(0.5)
        if not {0, 58} <= set(state["modes"]):
            raise AssertionError("backend OFFBOARD heartbeat did not persist after the API command")
        self.client.enableApiControl(False, vehicle_name=self.vehicle)
        test.wait("disabling API drops OFFBOARD and host arming",
                  lambda s: not ({0, 58} & set(s["modes"])), timeout=3, dwell=0.3)
        # Clear the API's level arm request before any subsequent enable.
        if not self.client.armDisarm(False, vehicle_name=self.vehicle):
            raise RuntimeError("API arm state reset was not confirmed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--eeprom", type=Path, default=Path("eeprom.bin"))
    parser.add_argument("--vehicle", default="Copter")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    binary, source = args.binary.resolve(), args.eeprom.resolve()
    if not binary.is_file() or not source.is_file():
        parser.error("--binary and --eeprom must exist")
    try:
        import cosysairsim as airsim
        from msgpackrpc.error import RPCError
    except ImportError as exc:
        parser.error(f"Cosys-AirSim Python client required: {exc}")
    directory = args.output or Path(tempfile.mkdtemp(prefix="bf_offboard_"))
    if args.output:
        directory.mkdir(parents=True, exist_ok=False)
    control.log(f"OFFBOARD artifacts: {directory}")
    report = {"passed": False, "vehicle": args.vehicle}
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port, timeout_value=5)
    test = scenario = None
    try:
        control.require_free_ports()
        control.require_quiet_rc()
        control.check_vehicle(client, args.vehicle, airsim)
        shutil.copy2(source, directory / "eeprom.bin")
        text = control.provision(binary, directory, ["get ap_hover_throttle"], "inspect")
        match = re.search(r"^ap_hover_throttle = (\d+)\s*$", text, re.MULTILINE)
        if match is None or not 1100 <= int(match[1]) <= 1700:
            raise RuntimeError("OFFBOARD test requires a valid configured hover throttle")
        hover = (int(match[1]) - 1000) / 1000
        lines = [*CLI_CONFIG_LINES, "set custom_link_motors_stream = ON"]
        with control.flight_session(binary, client, args.vehicle, directory, lines,
                                    stream_rc=False) as test:
            scenario = OffboardTest(test, hover)
            scenario.run()
        report["passed"] = True
    except (AssertionError, RuntimeError, OSError, TimeoutError, ValueError, RPCError) as exc:
        report["error"] = str(exc)
        if exc.__context__ is not None:
            report["caused_by"] = str(exc.__context__)
        control.log(f"OFFBOARD FAIL: {exc}")
    finally:
        client.client.close()
        if test is not None:
            report["last_state"] = test.last_sample
        if scenario is not None:
            report["rate_checks"] = scenario.results
        (directory / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    control.log("OFFBOARD PASS" if report["passed"] else "OFFBOARD FAIL")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
