#!/usr/bin/env python3
"""Disposable SITL only: native ANGLE authority, rate compatibility, pilot takeover and watchdog."""
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time

from airsim_control_test import require_free_ports, stop_process
from sitl_custom_link_sim_test import Feed, Telemetry, wait_for


def main():
    require_free_ports()
    binary = Path(__file__).resolve().parents[3] / "obj/main/betaflight_SITL.elf"
    with tempfile.TemporaryDirectory(prefix="bf_angle_sitl_") as directory:
        config = Path(directory) / "config.cli"
        config.write_text("set small_angle = 180\nset runaway_takeoff_prevention = OFF\n"
                          "set auto_disarm_delay = 0\nmap AETR1234\n"
                          "aux 0 0 0 1700 2100 0 0\naux 1 1 1 1700 2100 0 0\n")
        provision = subprocess.run([str(binary), "--config", str(config)], cwd=directory,
                                   capture_output=True, text=True, timeout=30)
        assert provision.returncode == 0, provision.stdout + provision.stderr
        with (Path(directory) / "sitl.log").open("w") as log:
            process = subprocess.Popen([str(binary)], cwd=directory, stdout=log, stderr=log)
            connection = feed = telemetry = None
            try:
                deadline = time.monotonic() + 10
                while connection is None:
                    assert process.poll() is None and time.monotonic() < deadline
                    try:
                        connection = socket.create_connection(("127.0.0.1", 5763), timeout=.2)
                    except ConnectionRefusedError:
                        time.sleep(.1)
                connection.settimeout(None)
                telemetry = Telemetry(connection)
                feed = Feed(connection, threading.Lock())
                assert wait_for("fresh disarmed telemetry", lambda: telemetry.gyro() is not None)
                time.sleep(3)
                feed.rc_channels[5] = 2000  # physical ANGLE switch in simulated RC only
                assert wait_for("pilot ANGLE before takeover", lambda: telemetry.last.get("mode_flags", 0) & 1)
                feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1, mode=3)
                assert wait_for("native ANGLE+OFFBOARD armed", lambda: telemetry.armed() and telemetry.offboard()
                                and telemetry.last.get("mode_flags", 0) & 1)
                feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1, mode=2)
                assert wait_for("rate OFFBOARD disables ANGLE", lambda: telemetry.offboard()
                                and not telemetry.last.get("mode_flags", 0) & 1)
                feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1, mode=3)
                assert wait_for("native ANGLE reentry", lambda: telemetry.offboard()
                                and telemetry.last.get("mode_flags", 0) & 1)
                feed.rc_channels[5] = 1000
                assert wait_for("pilot mode switch takes authority", lambda: not telemetry.offboard())
                feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1, mode=0)
                time.sleep(.2)
                feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=1, mode=3)
                assert wait_for("explicit fresh ANGLE request", lambda: telemetry.offboard()
                                and telemetry.last.get("mode_flags", 0) & 1)
                feed.control = None
                assert wait_for("watchdog releases native ANGLE host authority", lambda: not telemetry.offboard())
                print("PASS: actual SITL native ANGLE selection, rate compatibility, pilot takeover and watchdog")
            finally:
                if feed is not None:
                    feed.stop()
                if connection is not None:
                    connection.close()
                if telemetry is not None:
                    telemetry.stop()
                stop_process(process)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
