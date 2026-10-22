#!/usr/bin/env python3
"""Disarmed-only, isolated SITL check of acknowledged runtime PID application."""

import socket
from pathlib import Path
import subprocess
import tempfile
import threading
import time

from airsim_control_test import require_free_ports, stop_process
from sitl_custom_link_cli_test import CliReader, do_cli


def main():
    require_free_ports()
    binary = Path(__file__).resolve().parents[3] / "obj/main/betaflight_SITL.elf"
    with tempfile.TemporaryDirectory(prefix="bf_pid_protocol_") as directory:
        with (Path(directory) / "sitl.log").open("w") as log:
            process = subprocess.Popen([str(binary)], cwd=directory, stdout=log, stderr=log)
            reader = connection = None
            try:
                deadline = time.monotonic() + 15
                while connection is None:
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError("SITL did not start")
                    try:
                        connection = socket.create_connection(("127.0.0.1", 5763), timeout=.2)
                    except ConnectionRefusedError:
                        time.sleep(.1)
                connection.settimeout(None)
                reader = CliReader(connection)
                time.sleep(.3)
                lock = threading.Lock()
                sequence = 0

                def request(cmd, args, ok=True):
                    nonlocal sequence
                    sequence += 1
                    status, text, _, error = do_cli(connection, lock, reader, sequence, cmd, args)
                    if error or status != (0 if ok else 1):
                        raise AssertionError((cmd, args, status, text, error))
                    return text.decode()

                assert request(5, "check") == "PID PROFILE 0"
                original = request(0, "p_roll")
                gain = int(original.split(" = ")[1])
                assert request(1, f"p_roll = {gain + 1}") == f"p_roll = {gain + 1}"
                assert request(5, "0") == "PID PROFILE 0"
                assert request(5, "1", ok=False) == "PID PROFILE MISMATCH"
                assert request(5, "", ok=False) == "PID PROFILE MISMATCH"
                assert request(6, "0", ok=False) == "BAD REQUEST"
                assert request(1, original) == original
                assert request(5, "0") == "PID PROFILE 0"
                assert request(0, "p_roll") == original
                print("PASS: actual SET/GET/APPLY_PID, profile mismatch and old/unknown command errors")
            finally:
                if connection is not None:
                    connection.close()
                if reader is not None:
                    reader.stop()
                stop_process(process)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
