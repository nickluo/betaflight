#!/usr/bin/env python3
"""Bounded Yaw tuning against real Cosys-AirSim physics, never a harness plant.

Each trial clones the supplied EEPROM, starts its own SITL, flies two opposite
Yaw pulses, measures braking/ring-down, then lands. Only --apply can update the
source EEPROM, after an independent repeat meets absolute response limits and
improves the repeated baseline by at least 10%. No reset/teleport is used.
--objective response also requires faster rise and accurate rate tracking
without reintroducing ringing; it leaves Yaw I unchanged.
"""

import argparse
from collections.abc import Callable, Sequence
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import tempfile
import time
from typing import NotRequired, TypedDict

import airsim_control_test as control
import sitl_joystick as joystick


PARAMETERS = {"p": "p_yaw", "i": "i_yaw", "f": "f_yaw",
              "hold": "feedforward_yaw_hold_gain"}


@dataclass(frozen=True)
class Gains:
    p: int
    i: int
    f: int
    hold: int

    def lines(self):
        return [f"set {name} = {getattr(self, field)}" for field, name in PARAMETERS.items()]


@dataclass(frozen=True)
class Sample:
    time_s: float
    rate_dps: float
    target_dps: float
    heading_deg: float


@dataclass(frozen=True)
class Metrics:
    tracking_rmse: float
    response_gain: float
    travel_deg: float
    tail_rms_dps: float
    final_rms_dps: float
    reverse_deg: float
    settling_s: float
    rise80_s: float
    pulse_tracking_rmse: float
    overshoot_fraction: float

    @property
    def responsive(self):
        return 0.7 <= self.response_gain <= 1.3 and self.tracking_rmse <= 0.35 \
            and 15 <= self.travel_deg <= 100

    @property
    def stable(self):
        return self.responsive and self.settling_s <= 2 \
            and self.final_rms_dps <= 1.5 and self.reverse_deg <= 1.5

    @property
    def score(self):
        return self.tail_rms_dps + 2 * self.reverse_deg + 1.5 * self.settling_s \
            + 12 * self.tracking_rmse

    @property
    def response_ready(self):
        return self.stable and 0.9 <= self.response_gain <= 1.1 \
            and self.rise80_s <= 0.6 and self.pulse_tracking_rmse <= 0.45 \
            and self.overshoot_fraction <= 0.15

    @property
    def response_score(self):
        return 20 * self.pulse_tracking_rmse + 10 * self.rise80_s \
            + 2 * self.settling_s + 4 * self.reverse_deg + 2 * self.final_rms_dps


class TrialResult(TypedDict):
    score: float
    responsive: bool
    stable: bool
    directions: list[dict[str, float]]
    response_ready: bool
    response_score: float
    rise80_s: float
    gains: NotRequired[dict[str, int]]


def check_samples(samples: Sequence[Sample], minimum_span: float, minimum_count=8):
    if len(samples) < minimum_count:
        raise ValueError("too few Yaw samples")
    for sample in samples:
        if not all(math.isfinite(value) for value in asdict(sample).values()):
            raise ValueError("non-finite Yaw sample")
    gaps = [b.time_s - a.time_s for a, b in zip(samples, samples[1:])]
    if min(gaps) <= 0 or max(gaps) > 0.4:
        raise ValueError("Yaw samples are stale, unordered or too sparse")
    if samples[-1].time_s - samples[0].time_s < minimum_span:
        raise ValueError("Yaw observation window is too short")


def mean(samples: Sequence[Sample], value: Callable[[Sample], float]) -> float:
    duration = samples[-1].time_s - samples[0].time_s
    return sum((value(a) + value(b)) * 0.5 * (b.time_s - a.time_s)
               for a, b in zip(samples, samples[1:])) / duration


def sustained_rise80(step: Sequence[Sample], direction: int, target: float) -> float:
    for index, sample in enumerate(step):
        if direction * sample.rate_dps < 0.8 * target:
            continue
        window = [s for s in step[index:] if s.time_s <= sample.time_s + 0.2]
        if window[-1].time_s - sample.time_s >= 0.15 \
                and all(direction * s.rate_dps >= 0.8 * target for s in window):
            return sample.time_s
    return step[-1].time_s + 1


def measure(step: Sequence[Sample], tail: Sequence[Sample], direction: int) -> Metrics:
    if direction not in (-1, 1):
        raise ValueError("Yaw direction must be -1 or 1")
    check_samples(step, 0.9)
    check_samples(tail, 7, 50)
    steady = [s for s in step if s.time_s >= step[-1].time_s - 0.5]
    check_samples(steady, 0.25, 4)
    target = mean(steady, lambda s: direction * s.target_dps)
    if target < 10:
        raise ValueError("firmware Yaw target did not follow the commanded pulse")
    gain = mean(steady, lambda s: direction * s.rate_dps) / target
    tracking = math.sqrt(mean(steady, lambda s: (s.rate_dps - s.target_dps) ** 2)) / target
    pulse_tracking = math.sqrt(mean(step, lambda s: (direction * s.rate_dps - target) ** 2)) / target
    rise80 = sustained_rise80(step, direction, target)
    overshoot = max(0.0, max(direction * s.rate_dps for s in step) / target - 1)
    travel = sum(control.angle_error(b.heading_deg, a.heading_deg)
                 for a, b in zip(step, step[1:])) * direction
    coast = [s for s in tail if s.time_s >= 0.3]
    final = [s for s in tail if s.time_s >= tail[-1].time_s - 1]
    check_samples(coast, 6, 40)
    check_samples(final, 0.6, 4)
    if any(abs(s.target_dps) > 1 for s in coast):
        raise ValueError("Yaw target is not neutral after release; check competing RC senders")
    reverse = mean(coast, lambda s: max(0.0, -direction * s.rate_dps)) \
        * (coast[-1].time_s - coast[0].time_s)
    last_unsettled = max((i for i, s in enumerate(coast) if abs(s.rate_dps) > 2), default=-1)
    settling = coast[last_unsettled + 1].time_s if last_unsettled < len(coast) - 1 \
        else tail[-1].time_s + 1
    return Metrics(tracking, gain, travel,
                   math.sqrt(mean(coast, lambda s: s.rate_dps ** 2)),
                   math.sqrt(mean(final, lambda s: s.rate_dps ** 2)), reverse, settling,
                   rise80, pulse_tracking, overshoot)


def aggregate(metrics: Sequence[Metrics]) -> TrialResult:
    return {"score": max(m.score for m in metrics),
            "responsive": all(m.responsive for m in metrics),
            "stable": all(m.stable for m in metrics),
            "response_ready": all(m.response_ready for m in metrics),
            "response_score": max(m.response_score for m in metrics),
            "rise80_s": max(m.rise80_s for m in metrics),
            "directions": [asdict(m) for m in metrics]}


def validated_improvement(baseline: TrialResult, candidate: TrialResult, objective="ringdown") -> bool:
    if objective == "response":
        return candidate["response_ready"] and candidate["stable"] \
            and candidate["response_score"] < 0.9 * baseline["response_score"] \
            and candidate["rise80_s"] < 0.9 * baseline["rise80_s"]
    if objective != "ringdown":
        raise ValueError(f"unknown tuning objective: {objective}")
    return candidate["stable"] and candidate["responsive"] \
        and candidate["score"] < 0.9 * baseline["score"]


def better(candidate: TrialResult, current: TrialResult, objective="ringdown") -> bool:
    if not candidate["responsive"]:
        return False
    if not current["responsive"]:
        return True
    if candidate["stable"] != current["stable"]:
        return candidate["stable"]
    if objective == "response":
        if candidate["response_ready"] != current["response_ready"]:
            return candidate["response_ready"]
        return candidate["response_score"] < current["response_score"]
    if objective != "ringdown":
        raise ValueError(f"unknown tuning objective: {objective}")
    return candidate["score"] < current["score"]


def search_stages(objective, baseline):
    if objective == "response":
        return (("p", tuple(v for v in (90, 110, 130, 150) if v > baseline.p)),
                ("f", tuple(v for v in (60, 120) if v > baseline.f)),
                ("hold", tuple(v for v in (30,) if v > baseline.hold)))
    if objective == "ringdown":
        return (("hold", (0,)), ("p", (35, 55, 65, 75)),
                ("i", (40, 20, 10, 0)), ("f", (30, 0)))
    raise ValueError(f"unknown tuning objective: {objective}")


def parse_settings(text):
    profiles = re.findall(r"^profile (\d+)\s*$", text, re.MULTILINE)
    if not profiles:
        raise ValueError("SITL did not report its active PID profile")
    values = {}
    for name in (*PARAMETERS.values(), "ap_hover_throttle"):
        matches = re.findall(rf"^{name} = (\d+)\s*$", text, re.MULTILINE)
        if not matches:
            raise ValueError(f"SITL did not report {name}")
        values[name] = int(matches[-1])
    gains = Gains(*(values[name] for name in PARAMETERS.values()))
    if not (10 <= gains.p <= 150 and 0 <= gains.i <= 120 and 0 <= gains.f <= 240
            and 0 <= gains.hold <= 50 and 1100 <= values["ap_hover_throttle"] <= 1700):
        raise ValueError("baseline outside this tuner's bounded gain/hover range")
    return int(profiles[-1]), gains, values["ap_hover_throttle"]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_clock(first_ns, second_ns, elapsed_s):
    if not isinstance(first_ns, int) or not isinstance(second_ns, int) or elapsed_s <= 0:
        raise ValueError("invalid AirSim clock observations")
    ratio = (second_ns - first_ns) * 1e-9 / elapsed_s
    if not 0.9 <= ratio <= 1.1:
        raise RuntimeError(f"Yaw tuner requires a running real-time clock; observed ratio={ratio:.3f}")


def check_clock(client, vehicle, airsim):
    first = client.getMultirotorState(vehicle_name=vehicle)
    if not isinstance(first, airsim.MultirotorState):
        raise RuntimeError("AirSim did not return its clock")
    start = time.monotonic()
    time.sleep(1)
    second = client.getMultirotorState(vehicle_name=vehicle)
    if not isinstance(second, airsim.MultirotorState):
        raise RuntimeError("AirSim did not return its clock")
    verify_clock(first.timestamp, second.timestamp, time.monotonic() - start)


def inspect(binary, directory):
    return parse_settings(control.provision(
        binary, directory, ["profile", *(f"get {name}" for name in PARAMETERS.values()),
                            "get ap_hover_throttle"], "inspect"))


def capture(test, duration, command, directory, label):
    test.rc.set(yaw=command)
    samples = []
    start = time.monotonic()
    with (directory / f"{label}.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "actual_yaw_dps", "target_yaw_dps", "heading_deg", "rc_yaw_us"))
        while time.monotonic() - start < duration:
            state = test.sample()
            if not {0, 3, 11} <= set(state["modes"]):
                raise RuntimeError("lost ARM/ALTHOLD/POSHOLD during Yaw trial")
            origin = test.origin
            if origin is None:
                raise RuntimeError("Yaw trial has no takeoff origin")
            p = state["position"]
            if origin[2] - p[2] > 8 or math.dist(origin[:2], p[:2]) > 8 \
                    or max(abs(state["rpy"][0]), abs(state["rpy"][1])) > 25 \
                    or abs(state["yaw_rate_dps"]) > 120:
                raise RuntimeError("Yaw tuning safety envelope exceeded")
            debug = test.msp.request(254)
            if len(debug) < 2:
                raise RuntimeError("no firmware FEEDFORWARD Yaw setpoint")
            # BF gyro/setpoint is CCW-positive; AirSim FRD yaw is CW-positive.
            target = -float(int.from_bytes(debug[:2], "little", signed=True))
            rc_data = test.msp.request(105)
            if len(rc_data) < 8:
                raise RuntimeError("truncated MSP_RC")
            rc_yaw = int.from_bytes(rc_data[4:6], "little")
            elapsed = time.monotonic() - start
            if elapsed > 0.3 and rc_yaw != command:
                raise RuntimeError(f"RC conflict: requested {command}, received {rc_yaw}")
            sample = Sample(elapsed, state["yaw_rate_dps"], target, state["rpy"][2])
            samples.append(sample)
            writer.writerow((*asdict(sample).values(), rc_yaw))
            time.sleep(0.05)
    return samples


def trial(binary, client, vehicle, airsim, backup, directory, profile, gains, hover):
    directory.mkdir()
    shutil.copy2(backup, directory / "eeprom.bin")
    control.check_vehicle(client, vehicle, airsim)
    lines = [f"profile {profile}", *joystick.CLI_CONFIG_LINES, *gains.lines(),
             "set debug_mode = FEEDFORWARD", "set gyro_filter_debug_axis = YAW"]
    measurements = []
    with control.flight_session(binary, client, vehicle, directory, lines) as test:
        test.sensors()
        test.takeoff(hover)
        for direction in (1, -1):
            test.stage = f"yaw_{direction}_pulse"
            step = capture(test, 1.25, 1500 + direction * 100, directory, f"pulse_{direction}")
            test.stage = f"yaw_{direction}_release"
            tail = capture(test, 8, 1500, directory, f"release_{direction}")
            measurements.append(measure(step, tail, direction))
        test.land()
    result = aggregate(measurements)
    result["gains"] = asdict(gains)
    (directory / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    control.log(f"{directory.name}: {asdict(gains)}, score={result['score']:.3f}, "
                f"stable={result['stable']}, rise80={result['rise80_s']:.3f}s, "
                f"response_score={result['response_score']:.3f}, ready={result['response_ready']}")
    return result


def apply_best(binary, source, original_digest, directory, profile, gains):
    control.require_free_ports()
    if digest(source) != original_digest:
        raise RuntimeError("source EEPROM changed during tuning; refusing to overwrite it")
    directory.mkdir()
    shutil.copy2(source, directory / "eeprom.bin")
    control.provision(binary, directory, [f"profile {profile}", *gains.lines()], "apply")
    actual_profile, actual_gains, _ = inspect(binary, directory)
    if actual_profile != profile or actual_gains != gains:
        raise RuntimeError("best parameters failed EEPROM reload verification")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=source.parent, prefix=".yaw-eeprom-", delete=False) as f:
            temporary = Path(f.name)
            f.write((directory / "eeprom.bin").read_bytes())
            f.flush()
            os.fsync(f.fileno())
        os.chmod(temporary, stat.S_IMODE(source.stat().st_mode))
        if digest(source) != original_digest:
            raise RuntimeError("source EEPROM changed before apply; refusing to overwrite it")
        os.replace(temporary, source)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--eeprom", type=Path, default=Path("eeprom.bin"))
    parser.add_argument("--vehicle", default="Copter")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--output", type=Path, help="new artifact directory")
    parser.add_argument("--objective", choices=("ringdown", "response"), default="ringdown",
                        help="reduce ringing (default) or improve response while preserving stability")
    parser.add_argument("--apply", action="store_true", help="apply only independently verified improvement")
    args = parser.parse_args()
    binary, source = args.binary.resolve(), args.eeprom.resolve()
    if not binary.is_file() or not source.is_file():
        parser.error("--binary and --eeprom must exist")
    try:
        import cosysairsim as airsim
        from msgpackrpc.error import RPCError
    except ImportError as exc:
        parser.error(f"Cosys-AirSim Python client required: {exc}")
    output = args.output or Path(tempfile.mkdtemp(prefix="bf_yaw_tune_"))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    control.log(f"Yaw tuning artifacts: {output}")
    trials: list[TrialResult] = []
    report: dict[str, object] = {"validated": False, "applied": False, "trials": trials,
                                "objective": args.objective}
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port, timeout_value=5)
    try:
        control.require_free_ports()
        control.require_quiet_rc()
        control.check_vehicle(client, args.vehicle, airsim)
        check_clock(client, args.vehicle, airsim)
        original_digest = digest(source)
        backup = output / "original-eeprom.bin"
        shutil.copy2(source, backup)
        inspection = output / "baseline-config"
        inspection.mkdir()
        shutil.copy2(backup, inspection / "eeprom.bin")
        profile, baseline_gains, hover = inspect(binary, inspection)
        report["profile"] = profile

        def run(gains, label):
            result = trial(binary, client, args.vehicle, airsim, backup,
                           output / label, profile, gains, hover)
            trials.append(result)
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            return result

        baseline = run(baseline_gains, "baseline")
        best_gains, best = baseline_gains, baseline
        seen = {baseline_gains}
        for field, values in search_stages(args.objective, baseline_gains):
            seed = best_gains
            for value in values:
                candidate = replace(seed, **{field: value})
                if candidate in seen:
                    continue
                seen.add(candidate)
                result = run(candidate, f"trial-{len(seen):02d}-{field}-{value}")
                if better(result, best, args.objective):
                    best_gains, best = candidate, result
        report["best"] = best
        baseline_repeat = run(baseline_gains, "baseline-repeat")
        best_repeat = run(best_gains, "best-repeat")
        report["baseline_repeat"] = baseline_repeat
        report["best_repeat"] = best_repeat
        report["validated"] = best["stable"] and validated_improvement(
            baseline_repeat, best_repeat, args.objective)
        if not report["validated"]:
            raise RuntimeError("no independently verified stable improvement; original EEPROM unchanged")
        recommended = output / "recommended.cfg"
        recommended.write_text("\n".join([f"profile {profile}", *best_gains.lines(), "save"]) + "\n")
        if args.apply:
            apply_best(binary, source, original_digest, output / "apply", profile, best_gains)
            report["applied"] = True
        control.log(f"verified best Yaw gains: {asdict(best_gains)}, applied={report['applied']}")
    except (AssertionError, RuntimeError, OSError, TimeoutError, ValueError, RPCError) as exc:
        report["error"] = str(exc)
        if exc.__context__ is not None:
            report["caused_by"] = str(exc.__context__)
        control.log(f"Yaw tuning failed: {exc}")
    finally:
        client.client.close()
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["validated"] and "error" not in report else 1


if __name__ == "__main__":
    sys.exit(main())
