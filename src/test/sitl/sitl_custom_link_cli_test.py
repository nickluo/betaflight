#!/usr/bin/env python3
"""SITL end-to-end test for the CLI read/write channel of the custom link.

Brings up Betaflight SITL (custom companion-computer link on UART3/tcp:5763,
no UDP bridge, no RC/IMU injection needed - everything runs disarmed) and
exercises the 0x24 HOST_CLI / 0x15 FC_CLI frames against the real settings
table through the same engine the serial CLI and MSP2_CLI_SETTING use:

  1. link bring-up        : telemetry flows (init race guard)
  2. get direct setting   : "failsafe_delay = N", single chunk
  3. set + readback       : set failsafe_delay, response echoes the new value
  4. restore              : original value is written back
  5. set out of range     : rejected with SET FAILED, value unchanged
  6. set syntax error     : missing '=' rejected
  7. get unknown          : NOT FOUND
  8. malformed request    : nonzero reserved -> BAD REQUEST, req_id echoed
  9. lookup get/set/info  : motor_pwm_protocol - INFO values list drives the
                            choice of a different valid token, then set and
                            read back, then restore
10. info chunking         : reassembled text spans multiple 0x15 chunks with
                            contiguous offsets, total_len matches, flags
                            MORE set on intermediate chunks
11. string setting        : MODE_STRING round trip (craft_name)
12. stream interruption   : a GET superseding a multi-chunk INFO stream kills
                            the old stream (no further chunks, MORE never
                            clears) while answering normally - last-wins
13. telemetry alive       : 0x10 keeps flowing after all CLI traffic (the
                            streamer never wedges the 200 Hz task)

Usage:
  python3 src/test/sitl/sitl_custom_link_cli_test.py \
      [--binary obj/betaflight_2026.6.1_SITL] [--workdir /tmp/bf_cl_cli]

Notes:
  * A stale SITL process will poison the run (port 5763 + eeprom): the
    script pkills first (both binary spellings).
  * All writes are RAM-only (CLI "set" without "save") and the SITL process
    is disposable, so no eeprom hygiene is needed beyond a fresh workdir.
  * Every multi-chunk response here exercises the INFO windowed-regeneration
    path; no SITL setting produces a "name = value" longer than one chunk
    (longest string setting is 16 chars, the 50-element array settings are
    compiled only on FRSKY_SPI builds). The snapshot path's chunk math is
    identical by construction (shared clCliResponseHeader + send/advance
    epilogue in customLinkCliPump) and is covered by
    CustomLinkCliTest.TestCliResponseHeaderChunking in the unit tests.
"""

import argparse
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import sitl_custom_link_sim_test as sim

LINK_TCP_PORT = sim.LINK_TCP_PORT
SYNC = sim.SYNC
encode = sim.encode
crc16_xmodem = sim.crc16_xmodem

MSG_FC_FAST = sim.MSG_FC_FAST
MSG_FC_CLI = 0x15
MSG_HOST_CLI = 0x24

CL_CLI_CMD_GET = 0
CL_CLI_CMD_SET = 1
CL_CLI_CMD_INFO = 2

CL_CLI_STATUS_OK = 0
CL_CLI_STATUS_ERR = 1
CL_CLI_FLAG_MORE = 0x01

CLI_RESP_HDR = struct.Struct("<HHHBB")   # req_id, total_len, offset, status, flags + data[]


class CliReader:
    """Reader thread collecting 0x15 chunks and telemetry liveness counters."""

    def __init__(self, sock):
        self.sock = sock
        self.parser = sim.Parser()
        self.running = True
        self.cli_frames = []          # (monotonic, req_id, total, offset, status, flags, data)
        self.telemetry_count = 0
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        while self.running:
            try:
                data = self.sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            self.parser.feed(data)
            with self.lock:
                for msg_id, seq, payload, stamp in self.parser.frames:
                    if msg_id == MSG_FC_CLI and len(payload) >= 8:
                        req_id, total, offset, status, flags = CLI_RESP_HDR.unpack_from(payload, 0)
                        self.cli_frames.append(
                            (stamp, req_id, total, offset, status, flags, payload[8:]))
                    elif msg_id != MSG_HOST_CLI:
                        self.telemetry_count += 1
                self.parser.frames = []

    def stop(self):
        self.running = False
        self.t.join(timeout=2.0)


def send_cli(sock, send_lock, req_id, cmd, args):
    payload = struct.pack("<HBB", req_id, cmd, 0) + args.encode() + b"\x00"
    with send_lock:
        sock.sendall(encode(MSG_HOST_CLI, req_id & 0xFF, payload))


def collect_cli(reader, req_id, timeout=3.0):
    """Collect the reassembled 0x15 response for req_id (blocks until complete)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with reader.lock:
            frames = [f for f in reader.cli_frames if f[1] == req_id]
        if frames and not (frames[-1][5] & CL_CLI_FLAG_MORE):
            break
        time.sleep(0.02)
    else:
        return None, None, 0, "timeout waiting for response"

    # reassemble in arrival order, verifying contiguity
    chunks = [(f[3], f[6]) for f in frames]
    text = b""
    expect = 0
    for offset, data in chunks:
        if offset != expect:
            return None, None, len(chunks), f"chunk gap: expected offset {expect}, got {offset}"
        text += data
        expect += len(data)
    total = frames[-1][2]
    if len(text) != total:
        return None, total, len(chunks), f"reassembled {len(text)} bytes, total_len says {total}"
    return frames[-1][4], text, len(chunks), None


def do_cli(sock, send_lock, reader, req_id, cmd, args, timeout=3.0):
    """Send one 0x24 request and collect the reassembled 0x15 response."""
    send_cli(sock, send_lock, req_id, cmd, args)
    return collect_cli(reader, req_id, timeout)


def parse_value(text):
    """'name = value' -> (name, value) or (None, None)."""
    m = re.match(rb"^(\S+) = (.*)$", text, re.S)
    return (m.group(1).decode(), m.group(2).decode()) if m else (None, None)


def main():
    ap = argparse.ArgumentParser()
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    default_bin = os.path.join(repo_root, "obj", "betaflight_SITL.elf")
    ap.add_argument("--binary", default=os.path.normpath(default_bin),
                    help="path to the betaflight SITL executable")
    ap.add_argument("--workdir", default="/tmp/bf_custom_link_cli")
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        alt = os.path.join(os.path.dirname(binary), "betaflight_2026.6.1_SITL")
        if os.path.exists(alt):
            binary = alt
        else:
            print(f"binary not found: {args.binary} (build with: make TARGET=SITL)")
            return 1

    results = {}

    # 残留 SITL 进程会抢占 tcp/5763，先清理（行首锚定避免自杀）。构建产
    # 物有两种拼写（betaflight_SITL.elf / betaflight_<ver>_SITL），都可能
    # 被残留进程以任一拼写拉起，全部清理。
    for stale in {binary, os.path.join(os.path.dirname(binary), "betaflight_2026.6.1_SITL")}:
        subprocess.run(["pkill", "-f", f"^{stale}"], check=False)
    time.sleep(1.0)

    os.makedirs(args.workdir, exist_ok=True)
    eeprom = os.path.join(args.workdir, "eeprom.bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    sitl_log = open(os.path.join(args.workdir, "sitl.log"), "w")
    sitl = subprocess.Popen(["stdbuf", "-oL", "-eL", binary], cwd=args.workdir,
                            stdout=sitl_log, stderr=sitl_log,
                            preexec_fn=os.setsid)

    sock = None
    reader = None
    send_lock = threading.Lock()
    next_req_id = [0x1000]

    def request(cmd, args_str, timeout=3.0):
        next_req_id[0] += 1
        return do_cli(sock, send_lock, reader, next_req_id[0], cmd, args_str, timeout)

    try:
        # ---- 1. link bring-up (init race: no uplink before telemetry) ----
        print("== 1. link bring-up ==")
        sock = None
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection(("127.0.0.1", LINK_TCP_PORT), timeout=0.5)
                s.settimeout(2.0)
                try:
                    data = s.recv(64)
                except socket.timeout:
                    data = b""
                if SYNC in data:
                    sock = s
                    break
                s.close()
            except OSError:
                pass
            time.sleep(0.3)
        if sock is None:
            print("  [FAIL] SITL did not stream custom link telemetry on tcp/5763")
            return 1
        print("  [ok] telemetry streaming")
        results["link_up"] = True
        sock.settimeout(None)
        reader = CliReader(sock)
        time.sleep(1.0)
        c0 = reader.telemetry_count
        results["telemetry_baseline"] = c0 > 0

        # ---- 2. get direct setting ----
        print("== 2. get direct setting (failsafe_delay) ==")
        status, text, nchunks, err = request(CL_CLI_CMD_GET, "failsafe_delay")
        name, value = parse_value(text or b"")
        print(f"  status={status} chunks={nchunks} text={text!r}")
        results["get_direct"] = (err is None and status == CL_CLI_STATUS_OK and
                                 name == "failsafe_delay" and value is not None and
                                 value.isdigit() and nchunks == 1)
        orig_delay = int(value) if results["get_direct"] else 4

        # ---- 3. set + readback ----
        print(f"== 3. set + readback (failsafe_delay = 20, was {orig_delay}) ==")
        status, text, nchunks, err = request(CL_CLI_CMD_SET, "failsafe_delay = 20")
        name, value = parse_value(text or b"")
        print(f"  status={status} text={text!r}")
        results["set_readback"] = (err is None and status == CL_CLI_STATUS_OK and
                                   name == "failsafe_delay" and value == "20")
        status, text, _, err = request(CL_CLI_CMD_GET, "failsafe_delay")
        results["set_persists_in_ram"] = (err is None and status == CL_CLI_STATUS_OK and
                                          parse_value(text or b"")[1] == "20")

        # ---- 4. restore ----
        print(f"== 4. restore (failsafe_delay = {orig_delay}) ==")
        status, text, _, err = request(CL_CLI_CMD_SET, f"failsafe_delay = {orig_delay}")
        results["restore"] = (err is None and status == CL_CLI_STATUS_OK and
                              parse_value(text or b"")[1] == str(orig_delay))

        # ---- 5. out-of-range set rejected ----
        print("== 5. set out of range (failsafe_delay = 999) ==")
        status, text, _, err = request(CL_CLI_CMD_SET, "failsafe_delay = 999")
        print(f"  status={status} text={text!r}")
        status2, text2, _, err2 = request(CL_CLI_CMD_GET, "failsafe_delay")
        results["set_range_rejected"] = (err is None and status == CL_CLI_STATUS_ERR and
                                         text == b"SET FAILED" and err2 is None and
                                         parse_value(text2 or b"")[1] == str(orig_delay))

        # ---- 6. syntax error rejected ----
        print("== 6. set syntax error (missing '=') ==")
        status, text, _, err = request(CL_CLI_CMD_SET, "failsafe_delay")
        results["set_syntax_rejected"] = (err is None and status == CL_CLI_STATUS_ERR and
                                          text == b"SET FAILED")

        # ---- 7. unknown setting ----
        print("== 7. get unknown setting ==")
        status, text, _, err = request(CL_CLI_CMD_GET, "zzz_no_such_setting")
        print(f"  status={status} text={text!r}")
        results["get_unknown"] = (err is None and status == CL_CLI_STATUS_ERR and
                                  text == b"NOT FOUND")

        # ---- 8. malformed request (nonzero reserved) ----
        print("== 8. malformed request (reserved != 0) ==")
        req_id = next_req_id[0] + 1
        next_req_id[0] = req_id
        payload = struct.pack("<HBB", req_id, CL_CLI_CMD_GET, 1) + b"failsafe_delay\x00"
        with send_lock:
            sock.sendall(encode(MSG_HOST_CLI, req_id & 0xFF, payload))
        deadline = time.monotonic() + 3.0
        frames = []
        while time.monotonic() < deadline:
            with reader.lock:
                frames = [f for f in reader.cli_frames if f[1] == req_id]
            if frames:
                break
            time.sleep(0.02)
        print(f"  frames={len(frames)} text={frames[-1][6] if frames else None!r}")
        results["bad_request"] = (bool(frames) and frames[-1][4] == CL_CLI_STATUS_ERR and
                                  frames[-1][6] == b"BAD REQUEST")

        # ---- 9. lookup setting: info drives a valid alternative, then set ----
        print("== 9. lookup setting round trip (motor_pwm_protocol) ==")
        status, text, nchunks, err = request(CL_CLI_CMD_INFO, "motor_pwm_protocol")
        info_nchunks = nchunks
        info_len = len(text or b"")
        print(f"  info: status={status} chunks={nchunks} len={info_len}")
        info_text = (text or b"").decode(errors="replace")
        m_values = re.search(r"^values=(.+)$", info_text, re.M)
        results["info_shape"] = (err is None and status == CL_CLI_STATUS_OK and
                                 "pgn=" in info_text and "type=lookup" in info_text and
                                 "default=" in info_text and m_values is not None)
        status, text, _, err = request(CL_CLI_CMD_GET, "motor_pwm_protocol")
        name, cur = parse_value(text or b"")
        alt = None
        if m_values:
            for v in m_values.group(1).split(","):
                if v != cur:
                    alt = v
                    break
        print(f"  current={cur} alternative={alt}")
        ok_set = False
        if alt is not None:
            status, text, _, err = request(CL_CLI_CMD_SET, f"motor_pwm_protocol = {alt}")
            ok_set = (err is None and status == CL_CLI_STATUS_OK and
                      parse_value(text or b"")[1] == alt)
            status, text, _, err = request(CL_CLI_CMD_GET, "motor_pwm_protocol")
            ok_set = ok_set and parse_value(text or b"")[1] == alt
            if cur is not None:
                status, text, _, err = request(CL_CLI_CMD_SET, f"motor_pwm_protocol = {cur}")
                ok_set = ok_set and err is None and status == CL_CLI_STATUS_OK
        results["lookup_roundtrip"] = bool(results["info_shape"] and cur is not None and ok_set)

        # ---- 10. info chunking invariants (captured in step 9) ----
        print("== 10. info chunking (motor_pwm_protocol) ==")
        results["info_multi_chunk"] = results["info_shape"] and info_nchunks >= 2
        results["info_total_consistent"] = info_len > 0

        # ---- 11. string setting ----
        print("== 11. string setting round trip (craft_name) ==")
        status, text, _, err = request(CL_CLI_CMD_GET, "craft_name")
        name, cur_name = parse_value(text or b"")
        print(f"  current craft_name={cur_name!r} (status={status})")
        status, text, _, err = request(CL_CLI_CMD_SET, "craft_name = clilink_test")
        results["string_roundtrip"] = (err is None and status == CL_CLI_STATUS_OK and
                                       parse_value(text or b"")[1] == "clilink_test")

        # ---- 12. interrupting an in-flight stream: last request wins ----
        # Start a long multi-chunk INFO stream (debug_mode has a ~40-entry
        # lookup table -> hundreds of bytes -> many chunks), wait for its
        # first chunk, then supersede it with a GET. The GET must complete
        # normally; the INFO stream must stop dead (no further chunks, MORE
        # never clears) - the actual last-wins property, not just "the channel
        # still answers after a burst".
        print("== 12. interrupting an in-flight stream: last wins ==")
        rid_a, rid_b = next_req_id[0] + 1, next_req_id[0] + 2
        next_req_id[0] = rid_b
        send_cli(sock, send_lock, rid_a, CL_CLI_CMD_INFO, "debug_mode")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with reader.lock:
                a_started = any(f[1] == rid_a for f in reader.cli_frames)
            if a_started:
                break
            time.sleep(0.002)
        send_cli(sock, send_lock, rid_b, CL_CLI_CMD_GET, "motor_poles")
        status, text, _, err = collect_cli(reader, rid_b)
        b_ok = (err is None and status == CL_CLI_STATUS_OK and
                parse_value(text or b"")[0] == "motor_poles")
        # grace period: any straggler rid_a chunk would land well inside 300 ms
        time.sleep(0.3)
        with reader.lock:
            a_frames = [f for f in reader.cli_frames if f[1] == rid_a]
        a_total = a_frames[0][2] if a_frames else 0
        a_bytes = sum(len(f[6]) for f in a_frames)
        a_completed = bool(a_frames) and not (a_frames[-1][5] & CL_CLI_FLAG_MORE)
        print(f"  GET completed={b_ok}; interrupted INFO: {len(a_frames)} chunks, "
              f"{a_bytes}/{a_total} bytes, completed={a_completed}")
        time.sleep(0.3)
        with reader.lock:
            a_frames_after = [f for f in reader.cli_frames if f[1] == rid_a]
        results["rapid_last_wins"] = (a_started and b_ok and
                                      a_total > 3 * 56 and          # genuinely multi-chunk
                                      a_bytes < a_total and          # genuinely interrupted
                                      not a_completed and           # never finished
                                      len(a_frames_after) == len(a_frames))  # and stayed dead

        # ---- 13. telemetry still alive ----
        print("== 13. telemetry alive after CLI traffic ==")
        time.sleep(1.0)
        c1 = reader.telemetry_count
        print(f"  telemetry frames: {c0} -> {c1}")
        results["telemetry_alive"] = c1 - c0 > 150

    finally:
        if reader is not None:
            reader.stop()
        if sock is not None:
            sock.close()
        try:
            os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
        except OSError:
            pass
        try:
            sitl.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
        sitl_log.close()

    print("\n===== summary =====")
    all_ok = True
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
        all_ok &= bool(v)
    print("RESULT:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
