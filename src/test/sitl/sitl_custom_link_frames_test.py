#!/usr/bin/env python3
"""SITL closed-loop coordinate-frame validation for the custom link OFFBOARD path.

Adds a rigid-body physics model on top of the pure custom-link simulator
(sitl_custom_link_sim_test.py drives the link but has no physics, which is
exactly how a frame-convention bug survives protocol round-trip tests).

Betaflight native conventions this test relies on (anchored in the firmware
sources: telemetry/mavlink.c:892 negates pitch for the MAVLink ATTITUDE
message whose pitch is nose-up positive; platform/SIMULATOR/sitl_gyro.h
"BF positive = nose down"; fc/rc.c updateRcCommands negates the yaw stick;
flight/pid.c pidLevel uses attitude and rate on the same sign per axis):

  * gyro       = FLU angular velocity, verbatim: roll right +, pitch
                nose-down + (both are the FLU right-handed +axis rotation),
                yaw CCW +
  * acc        = FLU specific force: X fwd +, Y LEFT +, Z up + (+1 g level)
  * attitude   roll right +, pitch nose-down + (FLU algebra), yaw =
                right-handed math angle CCW +, 0 = north  (0x10)

i.e. 0x10 gyro/acc are exactly the FLU values, 0x10 attitude is the FLU
tf2-style RPY (pitch nose-down +) with yaw re-referenced to north, and the
0x20 rate targets live in the 0x10 gyro frame. The ROS bridge therefore
needs NO sign flips on gyro, acc, attitude or uplink rates - only the yaw
reference rotation (+pi/2 to ENU).

Phases:

  A. firmware convention check (open loop through the real PID/mixer):
     command +50 deg/s on each BF axis via 0x20, verify the reported gyro
     tracks, the simulated airframe rotates in the physically correct
     direction, and 0x10 attitude matches the physics euler IN SIGN AND
     magnitude (direct comparison, not magnitude matching).

  B. bridge-transform closed loop (replicates custom_link_vehicle_node.cpp
     handleFast + pushControl exactly, plus a simple SO(3) attitude
     controller driving bodyrates):
       fixed  : all signs pass-through       -> attitude converges
       buggy  : current node code (-gy,-gz,-wy,-wz,-ay) -> diverges

Usage:
  python3 src/test/sitl/sitl_custom_link_frames_test.py [--binary ...]
      [--phase A|B|AB] [--workdir /tmp/bf_cl_frames]
"""

import argparse
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

LINK_TCP_PORT = 5763
SYNC = b"\xeb\x90"

BOX_ARM = 0
BOX_OFFBOARD = 58
OFFBOARD_MODE_BIT = 1 << 13
STATUS_ARMED = 0x0001

MSG_FC_FAST = 0x10
MSG_FC_MOTORS = 0x13
MSG_HOST_CONTROL = 0x20
MSG_HOST_IMU = 0x21
MSG_HOST_ENV = 0x22
MSG_HOST_RC = 0x23


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_id, seq, payload):
    hdr = bytes([msg_id, len(payload), seq & 0xFF])
    crc = crc16_xmodem(hdr + payload)
    return SYNC + hdr + payload + struct.pack("<H", crc)


class Parser:
    def __init__(self):
        self.buf = b""
        self.frames = []

    def feed(self, data):
        self.buf += data
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                self.buf = self.buf[-1:]
                return
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                return
            msg_id, length, seq = self.buf[2], self.buf[3], self.buf[4]
            if length == 0 or length > 64:
                self.buf = self.buf[2:]
                continue
            if len(self.buf) < 7 + length:
                return
            payload = self.buf[5:5 + length]
            crc = struct.unpack("<H", self.buf[5 + length:7 + length])[0]
            self.buf = self.buf[7 + length:]
            if crc != crc16_xmodem(bytes([msg_id, length, seq]) + payload):
                continue
            self.frames.append((msg_id, payload, time.monotonic()))


# ----------------------------------------------------------------------
# rigid-body physics: FLU body frame (x fwd, y left, z up), right-handed.
# euler_flu pitch is therefore NOSE-DOWN positive (FLU algebra).
# ----------------------------------------------------------------------
class Physics:
    # QuadX: M0 rear-right, M1 front-right, M2 rear-left, M3 front-left.
    # Spin senses (Betaflight QuadX canonical, viewed from above): M0/M3
    # (RR/FL) spin CW -> CCW reaction torque; M1/M2 (FR/RL) spin CCW -> CW
    # reaction. A +pitch BF command speeds up the REAR pair (mixer table)
    # -> nose-down torque (rear up); in FLU algebra nose-down is +y.
    MASS = 0.65
    ARM = 0.16           # m
    KF = 8.0             # N per motor at normalized output 1.0
    KM = 0.9             # yaw torque factor (x ARM*KF)
    IX = 0.020           # kg m^2
    IY = 0.020
    IZ = 0.035
    MOTOR_TAU = 0.015    # rotor lag
    G = 9.81

    def __init__(self):
        # q_phys: FLU body -> world (z up), spawn level
        self.q = (1.0, 0.0, 0.0, 0.0)          # w x y z
        self.w = [0.0, 0.0, 0.0]               # body rates FLU, rad/s
        self.motor_norm = [0.0] * 4            # smoothed 0..1
        self.motor_target = [0.0] * 4

    def set_motors_us(self, values):
        for i in range(4):
            us = values[i] if values and i < len(values) else 1000.0
            self.motor_target[i] = max(0.0, min(1.0, (us - 1000.0) / 1000.0))

    def step(self, dt):
        a = 1.0 - math.exp(-dt / self.MOTOR_TAU)
        for i in range(4):
            self.motor_norm[i] += a * (self.motor_target[i] - self.motor_norm[i])
        m = self.motor_norm
        f = [self.KF * v for v in m]
        # torques about FLU axes (roll right +, nose-down +, CCW yaw +)
        tx = self.ARM * (f[2] + f[3] - f[0] - f[1])   # left pair up
        ty = self.ARM * (f[0] + f[2] - f[1] - f[3])   # rear pair up
        tz = self.KM * self.ARM * (f[0] + f[3] - f[1] - f[2])  # RR/FL pair up
        Iw = [self.IX * self.w[0], self.IY * self.w[1], self.IZ * self.w[2]]
        gyro = [self.w[1] * Iw[2] - self.w[2] * Iw[1],
                self.w[2] * Iw[0] - self.w[0] * Iw[2],
                self.w[0] * Iw[1] - self.w[1] * Iw[0]]
        self.w[0] += dt * (tx - gyro[0]) / self.IX
        self.w[1] += dt * (ty - gyro[1]) / self.IY
        self.w[2] += dt * (tz - gyro[2]) / self.IZ
        w0, w1, w2 = self.w
        qw, qx, qy, qz = self.q
        dqw = 0.5 * (-qx * w0 - qy * w1 - qz * w2)
        dqx = 0.5 * (qw * w0 - qz * w1 + qy * w2)
        dqy = 0.5 * (qw * w1 + qz * w0 - qx * w2)
        dqz = 0.5 * (qw * w2 - qy * w0 + qx * w1)
        qw += dt * dqw; qx += dt * dqx; qy += dt * dqy; qz += dt * dqz
        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        self.q = (qw / n, qx / n, qy / n, qz / n)

    # ---- sensor outputs in Betaflight conventions (= FLU verbatim) ----
    def gyro_bf_dps(self):
        return (math.degrees(self.w[0]), math.degrees(self.w[1]),
                math.degrees(self.w[2]))

    def acc_flu_mg(self):
        # specific force, hover assumption: +1 g along world-up in body FLU
        qw, qx, qy, qz = self.q
        ux = 2 * (qx * qz - qw * qy)
        uy = 2 * (qy * qz + qw * qx)
        uz = 1 - 2 * (qx * qx + qy * qy)
        return (ux * 1000.0, uy * 1000.0, uz * 1000.0)

    def quat_bf(self):
        # BF internal quaternion; consumed only by the virtual-mag synthesis
        # (off by default). (w, x, -y, -z) of the FLU->world quaternion.
        return (self.q[0], self.q[1], -self.q[2], -self.q[3])

    def euler_flu(self):
        """FLU-algebra euler: roll right+, pitch NOSE-DOWN+, yaw CCW+ (rad)."""
        qw, qx, qy, qz = self.q
        roll = math.atan2(2 * (qw * qx + qy * qz),
                          1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
        return roll, pitch, yaw


# ----------------------------------------------------------------------
# bridge-transform replica (custom_link_vehicle_node.cpp)
# ----------------------------------------------------------------------
def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(a):
    return (a[0], -a[1], -a[2], -a[3])


def quat_from_rpy(roll, pitch, yaw):
    """Identical formula to tf2 Quaternion::setRPY (standard ZYX)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


class BridgeReplica:
    """Transform pair replicating custom_link_vehicle_node.cpp.

    fixed=True is the CORRECTED mapping: Betaflight gyro/acc are FLU
    verbatim and the attitude euler is the FLU RPY (pitch nose-down +) --
    no sign flips anywhere, only the yaw reference rotation to ENU.

    fixed=False replicates the CURRENT node code, which negates gyro
    pitch/yaw, acc Y and the uplink pitch/yaw rate commands (its comments
    misread the Betaflight conventions as a left-handed frame).
    """

    def __init__(self, fixed):
        self.fixed = fixed

    def attitude_from_telemetry(self, att):
        """0x10 attitude (0.01 deg) -> orientation quaternion as published
        on /mavros/imu/data via tf2 setRPY. Both variants pass pitch
        through (that part of the node was already right)."""
        roll = math.radians(att[0] * 0.01)
        pitch = math.radians(att[1] * 0.01)
        yaw_enu = norm_angle(math.radians(att[2] * 0.01) + math.pi / 2)
        return quat_from_rpy(roll, pitch, yaw_enu)

    def gyro_flu(self, gyro):
        """0x10 gyro (0.1 deg/s, BF) -> FLU rad/s."""
        sy = 1.0 if self.fixed else -1.0   # node negates pitch/yaw
        return (gyro[0] * 0.1 * math.pi / 180.0,
                gyro[1] * 0.1 * math.pi / 180.0 * sy,
                gyro[2] * 0.1 * math.pi / 180.0 * sy)

    @staticmethod
    def acc_flu_buggy(acc):
        """Current node code: {ax, -ay, az}."""
        return (acc[0] * 0.001, -acc[1] * 0.001, acc[2] * 0.001)

    @staticmethod
    def acc_flu_fixed(acc):
        """Corrected: BF acc is already FLU (Y left+)."""
        return (acc[0] * 0.001, acc[1] * 0.001, acc[2] * 0.001)

    def rates_to_bf(self, wx, wy, wz):
        """FLU bodyrates rad/s -> 0x20 rate deg/s (BF axes)."""
        sy = 1.0 if self.fixed else -1.0   # node negates pitch/yaw
        return (math.degrees(wx), math.degrees(wy) * sy,
                math.degrees(wz) * sy)


class AttitudeController:
    """SO(3) proportional attitude -> FLU bodyrate setpoints."""

    def __init__(self, k=6.0, max_rate=math.radians(150)):
        self.k = k
        self.max_rate = max_rate

    def rates(self, q_cur, q_des):
        qe = quat_mul(quat_conj(q_des), q_cur)
        s = 1.0 if qe[0] >= 0 else -1.0
        wx = -2 * self.k * s * qe[1]
        wy = -2 * self.k * s * qe[2]
        wz = -2 * self.k * s * qe[3]
        n = math.sqrt(wx * wx + wy * wy + wz * wz)
        if n > self.max_rate:
            wx *= self.max_rate / n
            wy *= self.max_rate / n
            wz *= self.max_rate / n
        return wx, wy, wz


# ----------------------------------------------------------------------
# feed + telemetry threads
# ----------------------------------------------------------------------
class Feed:
    """250 Hz IMU (physics-driven), 10 Hz env, 50 Hz RC, 50 Hz control."""

    def __init__(self, sock, send_lock, physics):
        self.sock = sock
        self.send_lock = send_lock
        self.phys = physics
        self.running = True
        self.seq = 0
        self.rc_channels = [1500, 1500, 1000, 1500] + [1000] * 12
        self.control = None  # dict(throttle, rates, arm)
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _send(self, msg_id, payload):
        with self.send_lock:
            try:
                self.sock.sendall(encode(msg_id, self.seq, payload))
            except OSError:
                self.running = False
        self.seq = (self.seq + 1) & 0xFF

    def _loop(self):
        t0 = time.monotonic()
        last_imu = last_env = last_rc = last_ctrl = last_phys = 0.0
        while self.running:
            now = time.monotonic() - t0
            ts_us = int(now * 1e6) & 0xFFFFFFFF

            if now - last_phys >= 0.004:
                self.phys.step(0.004)
                last_phys = now

            if now - last_imu >= 0.004:
                last_imu = now
                g = self.phys.gyro_bf_dps()
                a = self.phys.acc_flu_mg()
                q = self.phys.quat_bf()
                gx, gy, gz = (int(round(v * 10)) for v in g)
                ax, ay, az = (int(round(v)) for v in a)
                qq = [max(-32768, min(32767, int(round(v * 16384)))) for v in q]
                self._send(MSG_HOST_IMU, struct.pack(
                    "<I3h3h4h", ts_us, gx, gy, gz, ax, ay, az, *qq))

            if now - last_env >= 0.1:
                last_env = now
                self._send(MSG_HOST_ENV, struct.pack(
                    "<IIhiii3hHHBB",
                    ts_us, 101325, 2500,
                    -23000000, 134000000, 123456,
                    0, 0, 0, 0, 0, 1, 12))

            if now - last_rc >= 0.02:
                last_rc = now
                self._send(MSG_HOST_RC, struct.pack("<I16H", ts_us,
                                                    *self.rc_channels))

            if self.control is not None and now - last_ctrl >= 0.02:
                last_ctrl = now
                c = self.control
                rx, ry, rz = (int(round(v * 10)) for v in c["rates"])
                self._send(MSG_HOST_CONTROL, struct.pack(
                    "<QHBBH3h",
                    int(time.monotonic() * 1e6) & 0xFFFFFFFFFFFFFFFF,
                    0, c["arm"], 2, int(c["throttle"]), rx, ry, rz))

            time.sleep(0.001)

    def stop(self):
        self.running = False
        self.t.join(timeout=2.0)


class Telemetry:
    def __init__(self, sock):
        self.sock = sock
        self.parser = Parser()
        self.running = True
        self.last = {}
        self.motor_values = None
        self.hist = []  # (t, gyro0.1dps, att 0.01deg)
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
            for msg_id, payload, stamp in self.parser.frames:
                self._decode(msg_id, payload, stamp)
            self.parser.frames = []

    def _decode(self, msg_id, p, stamp):
        try:
            if msg_id == MSG_FC_FAST:
                ts, gx, gy, gz, ax, ay, az, r, pi_, yaw = struct.unpack(
                    "<I3h3h3h", p[:22])
                self.last["gyro"] = (gx, gy, gz)
                self.last["acc"] = (ax, ay, az)
                self.last["att"] = (r, pi_, yaw)
                self.hist.append((stamp, (gx, gy, gz), (r, pi_, yaw)))
                if len(self.hist) > 4000:
                    del self.hist[:1000]
            elif msg_id == 0x12:
                ts, vbat, cur, mah, fix, sats, lat, lon, alt, spd, crs, \
                    mode_flags, status = struct.unpack("<IHihBBiiiHHIH", p[:36])
                self.last["mode_flags"] = mode_flags
                self.last["status"] = status
            elif msg_id == MSG_FC_MOTORS:
                count = p[0]
                self.motor_values = struct.unpack("<8H", p[1:17])[:count]
        except struct.error:
            pass

    def stop(self):
        self.running = False
        self.t.join(timeout=2.0)

    def armed(self):
        s = self.last.get("status")
        return bool(s is not None and (s & STATUS_ARMED))

    def offboard(self):
        f = self.last.get("mode_flags")
        return bool(f is not None and (f & OFFBOARD_MODE_BIT))

    def att_deg(self):
        att = self.last.get("att")
        return None if att is None else (att[0] * 0.01, att[1] * 0.01,
                                         att[2] * 0.01)

    def gyro_dps(self):
        g = self.last.get("gyro")
        return None if g is None else (g[0] * 0.1, g[1] * 0.1, g[2] * 0.1)


def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else 0.0


def wait_until(pred, timeout, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return False


def ensure_armed(feed, tel, attempts=4):
    """Arm via host arm edge. The edge is one-shot: if tryArm() rejects it
    (boot grace, etc.) the request must be re-issued from arm=0."""
    for _ in range(attempts):
        feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=0)
        time.sleep(0.4)
        feed.control["arm"] = 1
        if wait_until(lambda: tel.armed() and tel.offboard(), 6.0):
            return True
        print(f"  arm/offboard retry (armed={tel.armed()} offboard={tel.offboard()})")
    return False


# ----------------------------------------------------------------------
def hold_level_direct(feed, phys, settle=7.0):
    """Re-level the sim using physics truth only (no bridge transform), so
    Phase A stays independent of Phase B's transform under test."""
    ctrl = AttitudeController(k=5.0)
    q_des = quat_from_rpy(0.0, 0.0, phys.euler_flu()[2])
    t_end = time.monotonic() + settle
    while time.monotonic() < t_end:
        q_cur = quat_from_rpy(*phys.euler_flu())
        wx, wy, wz = ctrl.rates(q_cur, q_des)
        # FLU rad/s -> BF deg/s: identical frame, unit conversion only
        feed.control["rates"] = (math.degrees(wx), math.degrees(wy),
                                 math.degrees(wz))
        e = phys.euler_flu()
        if abs(math.degrees(e[0])) < 2.0 and abs(math.degrees(e[1])) < 2.0:
            return True
        time.sleep(0.02)
    return False


def run_attitude_hold(feed, tel, bridge, phys, target_deg, hold_s,
                      k=6.0, settle=6.0, debug=False):
    """Closed loop through the bridge replica. Returns (converged, samples).
    target_deg = (roll, pitch) in FLU algebra degrees (pitch nose-down +)."""
    ctrl = AttitudeController(k=k)
    att0 = tel.att_deg() or (0.0, 0.0, 0.0)
    yaw0 = math.radians(att0[2]) + math.pi / 2
    roll_t, pitch_t = target_deg
    q_des = quat_from_rpy(math.radians(roll_t), math.radians(pitch_t), yaw0)
    t_end = time.monotonic() + settle + hold_s
    samples = []
    converged_t0 = None
    dbg_t0 = time.monotonic()
    feed.control["throttle"] = 1300
    while time.monotonic() < t_end:
        att = tel.last.get("att")
        if att is None:
            time.sleep(0.01)
            continue
        q_cur = bridge.attitude_from_telemetry(att)
        wx, wy, wz = ctrl.rates(q_cur, q_des)
        rates = bridge.rates_to_bf(wx, wy, wz)
        feed.control["rates"] = rates
        e = phys.euler_flu()
        samples.append((math.degrees(e[0]), math.degrees(e[1]),
                        math.degrees(e[2])))
        if debug and time.monotonic() - dbg_t0 >= 1.0:
            dbg_t0 = time.monotonic()
            print(f"    dbg est att={tel.att_deg()} "
                  f"phys(r,p,y)=({math.degrees(e[0]):.1f},{math.degrees(e[1]):.1f},{math.degrees(e[2]):.1f}) "
                  f"rates_bf={tuple(round(v,1) for v in rates)} "
                  f"armed={tel.armed()} ob={tel.offboard()} motors={tel.motor_values}")
        in_tol = (abs(math.degrees(e[1]) - pitch_t) <= 4.0
                  and abs(math.degrees(e[0]) - roll_t) <= 8.0)
        if in_tol:
            if converged_t0 is None:
                converged_t0 = time.monotonic()
            elif time.monotonic() - converged_t0 >= 1.0 and len(samples) > 30:
                return True, samples
        else:
            converged_t0 = None
        # divergence guard: any axis past 60 deg means the loop blew up
        if any(abs(v) > 60.0 for v in samples[-1]):
            return False, samples
        time.sleep(0.02)
    return False, samples


def run_phase_a(feed, tel, phys, results):
    print("== Phase A: firmware frame conventions (open-loop rate pulses) ==")
    feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=0)
    # boot grace (~5 s) must expire before the arm edge is spent
    time.sleep(5.5)
    if not ensure_armed(feed, tel):
        print("  [FAIL] could not arm / enter OFFBOARD")
        results["A_armed"] = False
        return
    print(f"  armed + OFFBOARD, motors={tel.motor_values}")
    results["A_armed"] = True
    feed.control["throttle"] = 1300
    time.sleep(0.4)

    def pulse(name, axis, cmd, dur, checks):
        feed.control["rates"] = (0, 0, 0)
        time.sleep(0.3)
        start_euler = phys.euler_flu()
        tel.hist.clear()
        feed.control["rates"] = tuple(
            cmd if i == axis else 0 for i in range(3))
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            time.sleep(0.02)
        mid_motors = tel.motor_values
        feed.control["rates"] = (0, 0, 0)
        end_euler = phys.euler_flu()
        # telemetry averages over the last 60% of the pulse
        gyro_samples = [h[1] for h in tel.hist if h[0] > t_end - dur * 0.6]
        att_samples = [h[2] for h in tel.hist if h[0] > t_end - 0.15]
        g_mean = tuple(mean(g[i] for g in gyro_samples) for i in range(3))
        att_end = tuple(mean(a[i] for a in att_samples) for i in range(3))
        att_end_deg = tuple(v * 0.01 for v in att_end)
        g_mean_dps = tuple(v * 0.1 for v in g_mean)
        print(f"  [{name}] cmd={cmd} deg/s  gyro mean={tuple(round(v,1) for v in g_mean_dps)}"
              f"  motors={mid_motors}"
              f"  phys euler delta={tuple(round(math.degrees(e-s),1) for e,s in zip(end_euler,start_euler))}"
              f"  att deg={tuple(round(v,1) for v in att_end_deg)}")
        return checks(g_mean_dps, start_euler, end_euler, att_end_deg)

    def chk_roll(g, s, e, att):
        d = math.degrees(e[0] - s[0])
        return abs(g[0] - 50) <= 18 and d > 12 and abs(att[0] - d) <= 12

    def chk_pitch(g, s, e, att):
        # BF pitch+ = nose-down = FLU algebra +y: direct comparison
        d = math.degrees(e[1] - s[1])
        return abs(g[1] - 50) <= 18 and d > 12 and abs(att[1] - d) <= 12

    def chk_yaw(g, s, e, att):
        # BF yaw+ = CCW = FLU z+: direct comparison with the math yaw
        d = math.degrees(e[2] - s[2])
        return abs(g[2] - 50) <= 18 and d > 12 and abs(att[2] - d) <= 12

    results["A_roll"] = pulse("roll", 0, 50, 1.2, chk_roll)
    lv = hold_level_direct(feed, phys)
    print(f"  re-level after roll: {lv}")
    results["A_pitch"] = pulse("pitch", 1, 50, 1.2, chk_pitch)
    lv = hold_level_direct(feed, phys)
    print(f"  re-level after pitch: {lv}")
    results["A_yaw"] = pulse("yaw", 2, 50, 1.2, chk_yaw)
    lv = hold_level_direct(feed, phys)
    print(f"  re-level after yaw: {lv}")

    # published gyro/acc vs physics truth (FLU verbatim expectation)
    g_phys = phys.gyro_bf_dps()
    g_tel = tel.gyro_dps() or (0, 0, 0)
    results["A_gyro_feed"] = all(abs(a - b) < 40 for a, b in zip(g_tel, g_phys))
    a_phys = phys.acc_flu_mg()
    a_tel = tel.last.get("acc") or (0, 0, 0)
    results["A_acc_feed"] = all(abs(a - b) < 150 for a, b in zip(a_tel, a_phys))
    print(f"  gyro tel={tuple(round(v,1) for v in g_tel)} phys={tuple(round(v,1) for v in g_phys)}"
          f"  acc tel={a_tel} phys={tuple(round(v,1) for v in a_phys)}")


def run_phase_b(feed, tel, phys, results):
    print("== Phase B: bridge transform closed loop ==")
    if not (tel.armed() and tel.offboard()):
        feed.control = dict(throttle=1000, rates=(0, 0, 0), arm=0)
        time.sleep(5.5)
        if not ensure_armed(feed, tel):
            results["B_armed"] = False
            return
    results["B_armed"] = True

    bridge_fixed = BridgeReplica(fixed=True)
    conv, samples = run_attitude_hold(feed, tel, bridge_fixed, phys,
                                      (0.0, 0.0), hold_s=1.0, settle=8.0,
                                      debug=True)
    print(f"  fixed transform -> level hold converged: {conv}, "
          f"final phys (r,p,y)={tuple(round(v,1) for v in samples[-1]) if samples else None}")
    results["B_level_fixed"] = conv

    # nose-up 8 deg = FLU-algebra pitch -8
    conv, samples = run_attitude_hold(feed, tel, bridge_fixed, phys,
                                      (0.0, -8.0), hold_s=1.5, settle=8.0,
                                      debug=True)
    tail = [s[1] for s in samples[-25:]] if samples else [0]
    print(f"  fixed transform -> nose-up 8 deg target converged: {conv}, "
          f"phys pitch tail mean={mean(tail):.1f} deg (FLU alg, exp ~ -8)")
    results["B_pitch_fixed"] = conv

    # re-level before the buggy run so both start from the same state
    run_attitude_hold(feed, tel, bridge_fixed, phys, (0.0, 0.0),
                      hold_s=0.5, settle=6.0)

    # BUGGY transform (current node code): inverted pitch/yaw rate commands
    bridge_buggy = BridgeReplica(fixed=False)
    conv, samples = run_attitude_hold(feed, tel, bridge_buggy, phys,
                                      (0.0, -8.0), hold_s=1.5, settle=8.0)
    pitches = [s[1] for s in samples]
    diverged = (not conv) and pitches and (
        max(pitches) > 30.0 or min(pitches) < -30.0)
    print(f"  buggy transform -> diverged: {diverged} (converged={conv}), "
          f"phys pitch range [{min(pitches):.1f}, {max(pitches):.1f}] deg")
    results["B_pitch_buggy_diverges"] = diverged

    feed.control["rates"] = (0, 0, 0)
    feed.control["arm"] = 0
    time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    default_bin = os.path.join(repo_root, "obj", "betaflight_SITL.elf")
    ap.add_argument("--binary", default=os.path.normpath(default_bin))
    ap.add_argument("--workdir", default="/tmp/bf_cl_frames")
    ap.add_argument("--phase", default="AB", choices=["A", "B", "AB"])
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)
    if not os.path.exists(binary):
        # the build also copies to obj/betaflight_<version>_SITL
        alt = os.path.join(os.path.dirname(binary), "betaflight_2026.6.1_SITL")
        if os.path.exists(alt):
            binary = alt
        else:
            print(f"binary not found: {args.binary} (build with: make TARGET=SITL)")
            return 1

    subprocess.run(["pkill", "-f", f"^{binary}"], check=False)
    time.sleep(1.0)

    os.makedirs(args.workdir, exist_ok=True)
    eeprom = os.path.join(args.workdir, "eeprom.bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    cfg = os.path.join(args.workdir, "config.txt")
    with open(cfg, "w") as f:
        f.write("\n".join([
            "set small_angle = 180",
            f"aux 0 {BOX_ARM} 0 1700 2100 0 0",
            f"aux 1 {BOX_OFFBOARD} 1 1700 2100 0 0",
        ]) + "\n")
    res = subprocess.run([binary, "--config", cfg], cwd=args.workdir,
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0 or not os.path.exists(eeprom):
        print("provisioning failed:", res.stdout[-400:], res.stderr[-400:])
        return 1

    sitl_log = open(os.path.join(args.workdir, "sitl.log"), "w")
    sitl = subprocess.Popen(["stdbuf", "-oL", "-eL", binary], cwd=args.workdir,
                            stdout=sitl_log, stderr=sitl_log,
                            preexec_fn=os.setsid)

    phys = Physics()
    sock = feed = tel = None
    results = {}
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection(("127.0.0.1", LINK_TCP_PORT),
                                             timeout=0.5)
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
            print("[FAIL] no telemetry on tcp/5763")
            return 1
        print("[ok] link up")
        sock.settimeout(None)
        send_lock = threading.Lock()
        tel = Telemetry(sock)
        time.sleep(0.5)
        feed = Feed(sock, send_lock, phys)
        feed.rc_channels[5] = 1800  # BOXOFFBOARD switch held
        # let the estimator settle at level before any control
        time.sleep(3.0)

        # motor values from 0x13 drive the physics
        def pump():
            while True:
                phys.set_motors_us(tel.motor_values or [1000] * 4)
                time.sleep(0.005)
        pump_t = threading.Thread(target=pump, daemon=True)
        pump_t.start()

        if "A" in args.phase:
            run_phase_a(feed, tel, phys, results)
        if "B" in args.phase:
            run_phase_b(feed, tel, phys, results)
    finally:
        if feed:
            feed.stop()
        if tel:
            tel.stop()
        if sock:
            try:
                sock.close()
            except OSError:
                pass
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
