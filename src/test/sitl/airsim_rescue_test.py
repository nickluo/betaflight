#!/usr/bin/env python3
"""Real AirSim one-switch return-home test using the sixth joystick detent.

Clones current EEPROM, departs under ordinary pilot RC, then sends only
neutral sticks plus AUX1=2000. No teleport, injected GPS, OFFBOARD commands
or outbound mission are used. Source EEPROM is never changed by the test.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time

import airsim_control_test as control
from airsim_offboard_test import touchdown_confirmed
import sitl_joystick as joystick


SIM_RESCUE_CONFIG = (
    "feature GPS",
    "set gps_provider = VIRTUAL",
    "set trust_mag = ON",
    "set gps_rescue_min_start_dist = 15",
    "set gps_rescue_alt_mode = FIXED_ALT",
    "set gps_rescue_return_alt = 8",
    "set gps_rescue_initial_climb = 3",
    "set gps_rescue_ascend_rate = 150",
    "set gps_rescue_descend_rate = 50",
    "set gps_rescue_ground_speed = 250",
    "set gps_rescue_descent_dist = 8",
    "set gps_rescue_min_sats = 8",
    "set gps_rescue_allow_arming_without_fix = OFF",
)


def distance_home(state, origin):
    return math.dist(state["position"][:2], origin[:2])


def rescue_controller(modes):
    if 46 in modes:
        return "legacy-gps-rescue"
    if {56, 3, 11} <= set(modes):
        return "flight-plan-rescue"
    return None


def run_rescue(test, hover_pwm):
    test.sensors()
    gps = test.msp.request(106)
    if len(gps) < 2 or not gps[0] or gps[1] < 8:
        raise RuntimeError("GPS Rescue requires a fix and at least 8 satellites")
    test.takeoff(hover_pwm)
    origin = test.origin
    if origin is None:
        raise RuntimeError("Rescue test has no arming origin")
    airborne_timestamp = test.client.getMultirotorState(vehicle_name=test.vehicle).timestamp
    test.stage = "rescue-manual-departure"
    test.rc.set(pitch=1800)
    try:
        test.wait("ordinary pilot RC flies more than 30 m from home",
                  lambda s: distance_home(s, origin) >= 30, timeout=80)
    finally:
        test.rc.set(pitch=1500)
    test.wait("pilot POSHOLD+ALTHOLD brakes before Rescue",
              lambda s: math.hypot(*s["velocity"][:2]) < 0.5, timeout=25, dwell=1)
    departure = test.sample()
    if distance_home(departure, origin) < 25 or 56 in departure["modes"] or 58 in departure["modes"]:
        raise AssertionError("outbound flight was not a distant, manual-RC flight")

    return complete_rescue(test, origin, departure, airborne_timestamp)


def complete_rescue(test, origin, departure, airborne_timestamp):
    test.stage = "rescue-sixth-detent"
    switch_start = time.monotonic()
    test.rc.set_mode("GPSRESCUE")
    def engaged(state):
        data = test.msp.request(105)
        return len(data) >= 10 and int.from_bytes(data[8:10], "little") == 2000 \
            and rescue_controller(state["modes"])
    state = test.wait("sixth detent engages GPS Rescue return-home controller", engaged,
                      timeout=10)
    engage_latency = time.monotonic() - switch_start
    if 58 in state["modes"]:
        raise AssertionError("OFFBOARD remained active after GPS Rescue engagement")
    test.rc.set(roll=1500, pitch=1500, yaw=1500)
    implementation = rescue_controller(state["modes"])
    start = time.monotonic()
    deadline = start + 160
    peak_height = 0.0
    closest = distance_home(state, origin)
    ground_contact = False
    controller_seen = False
    return_seen = False
    while time.monotonic() < deadline:
        state = test.sample()
        if 58 in state["modes"]:
            raise AssertionError("old API command recaptured OFFBOARD during GPS Rescue")
        distance = distance_home(state, origin)
        closest = min(closest, distance)
        peak_height = max(peak_height, origin[2] - state["position"][2])
        controller_seen = controller_seen or bool(rescue_controller(state["modes"]))
        return_seen = return_seen or distance < 6
        collision = test.client.simGetCollisionInfo(vehicle_name=test.vehicle)
        ground_contact = ground_contact or touchdown_confirmed(
            collision, airborne_timestamp, state["velocity"])
        if 0 not in state["modes"]:
            if not controller_seen or not return_seen or not ground_contact \
                    or distance > 6 or math.sqrt(sum(v * v for v in state["velocity"])) > 0.3:
                raise AssertionError(f"Rescue disarmed before confirmed home touchdown: {state}")
            break
        time.sleep(0.1)
    else:
        raise AssertionError(f"GPS Rescue did not return/land/disarm: {test.last_sample}")
    if peak_height < 7 or peak_height > 12:
        raise AssertionError(f"Rescue did not respect the 8 m return profile: peak={peak_height:.2f} m")
    test.observe(0.5)
    if test.last_sample is None or 0 in test.last_sample["modes"]:
        raise AssertionError("held sixth detent unexpectedly re-armed after Rescue landing")
    test.rc.set(arm=1000, throttle=1000)
    test.rc.set_mode("ACRO")
    test.wait("ARM low and leaving Rescue restore arming readiness",
              lambda s: s["arming_flags"] == 0, timeout=8, dwell=0.5)
    control.log(f"PASS: one-switch Rescue landed {distance:.2f} m from home and auto-disarmed")
    return {"controller": implementation, "departure_distance_m": distance_home(departure, origin),
            "engage_latency_s": engage_latency,
            "closest_home_m": closest, "touchdown_distance_m": distance,
            "peak_height_m": peak_height, "return_duration_s": time.monotonic() - start,
            "ground_contact": ground_contact, "auto_disarmed": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--eeprom", type=Path, default=Path("eeprom.bin"))
    parser.add_argument("--vehicle", default="Copter")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--print-config", action="store_true",
                        help="export canonical sixth-detent and low-altitude simulation Rescue config")
    args = parser.parse_args()
    if args.print_config:
        print("\n".join((*joystick.CLI_CONFIG_LINES, *SIM_RESCUE_CONFIG)))
        return 0
    if args.binary is None or not args.binary.is_file() or not args.eeprom.is_file():
        parser.error("--binary and --eeprom must exist")
    try:
        import cosysairsim as airsim
        from msgpackrpc.error import RPCError
    except ImportError as exc:
        parser.error(f"Cosys-AirSim Python client required: {exc}")
    binary, source = args.binary.resolve(), args.eeprom.resolve()
    directory = args.output or Path(tempfile.mkdtemp(prefix="bf_rescue_"))
    if args.output:
        directory.mkdir(parents=True, exist_ok=False)
    original_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    report = {"passed": False, "vehicle": args.vehicle}
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port, timeout_value=5)
    test = None
    control.log(f"Rescue artifacts: {directory}")
    try:
        control.require_free_ports()
        control.require_quiet_rc()
        control.check_vehicle(client, args.vehicle, airsim)
        shutil.copy2(source, directory / "original-eeprom.bin")
        shutil.copy2(source, directory / "eeprom.bin")
        hover_pwm = control.read_hover_pwm(binary, directory)
        lines = [*joystick.CLI_CONFIG_LINES, *SIM_RESCUE_CONFIG,
                 "set custom_link_motors_stream = ON"]
        with control.flight_session(binary, client, args.vehicle, directory, lines) as test:
            report["rescue"] = run_rescue(test, hover_pwm)
        report["passed"] = True
    except (AssertionError, RuntimeError, OSError, TimeoutError, ValueError, RPCError) as exc:
        report["error"] = str(exc)
        if exc.__context__ is not None:
            report["caused_by"] = str(exc.__context__)
        control.log(f"Rescue FAIL: {exc}")
    finally:
        client.client.close()
        if test is not None:
            report["last_state"] = test.last_sample
        report["source_eeprom_unchanged"] = hashlib.sha256(source.read_bytes()).hexdigest() == original_digest
        if not report["source_eeprom_unchanged"]:
            report["passed"] = False
            report["error"] = "source EEPROM changed during test"
        (directory / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    control.log("Rescue PASS" if report["passed"] else "Rescue FAIL")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
