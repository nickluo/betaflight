#!/usr/bin/env python3
"""Camera8-referenced virtual yaw/pitch launcher for AirSim SITL.

The launcher constrains the Copter only while mounted. After release, AirSim
physics and Betaflight alone drive the vehicle; the pad levels for return.
"""

import math
import threading
import time

import cosysairsim as airsim


def quaternion_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return airsim.Quaternionr(sr * cp * cy - cr * sp * sy,
                             cr * sp * cy + sr * cp * sy,
                             cr * cp * sy - sr * sp * cy,
                             cr * cp * cy + sr * sp * sy)


def rotate(q, v):
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    return (
        (1 - 2 * (y*y + z*z)) * v[0] + 2 * (x*y - w*z) * v[1] + 2 * (x*z + w*y) * v[2],
        2 * (x*y + w*z) * v[0] + (1 - 2 * (x*x + z*z)) * v[1] + 2 * (y*z - w*x) * v[2],
        2 * (x*z - w*y) * v[0] + 2 * (y*z + w*x) * v[1] + (1 - 2 * (x*x + y*y)) * v[2])


def launcher_angles(position, target, aim_pitch_deg):
    delta = tuple(t - p for p, t in zip(position, target))
    bearing = math.atan2(delta[1], delta[0])
    elevation = math.atan2(-delta[2], math.hypot(delta[0], delta[1]))
    # Camera AIM is 90 + pitch_aim relative to body forward, not world pitch.
    body_pitch = elevation - math.radians(90 + aim_pitch_deg)
    if abs(body_pitch) > math.radians(65):
        raise RuntimeError("target requires launcher tilt beyond 65 degrees")
    return bearing, body_pitch, elevation


def move_actor(client, name, pose):
    if client.simSetObjectPose(name, pose, True):
        return
    # UE reports False for an unchanged transform; verify the requested goal.
    actual = client.simGetObjectPose(name)
    a, p = actual.position, pose.position
    qa, qp = actual.orientation, pose.orientation
    dot = sum(a*b for a, b in zip((qa.x_val, qa.y_val, qa.z_val, qa.w_val),
                                  (qp.x_val, qp.y_val, qp.z_val, qp.w_val)))
    norm = math.sqrt((qa.x_val**2+qa.y_val**2+qa.z_val**2+qa.w_val**2) *
                     (qp.x_val**2+qp.y_val**2+qp.z_val**2+qp.w_val**2))
    distance = math.dist((a.x_val, a.y_val, a.z_val), (p.x_val, p.y_val, p.z_val))
    if (not math.isfinite(distance) or norm < 1e-6 or not math.isfinite(norm)
            or distance > .001 or abs(dot/norm) < math.cos(.001/2)):
        raise RuntimeError(f"launcher actuator {name} did not reach requested pose")


class AirsimLauncher:
    VEHICLE = "Copter"
    TARGET = "TargetDrone"
    MOUNT_OFFSET = .60

    def __init__(self, client, host, port, reference_camera="CineCameraActor_0", forward=8.0,
                 aim_pitch_deg=-10.0):
        self.client = client
        self.host, self.port = host, port
        self.reference_camera = reference_camera
        self.aim_pitch_deg = aim_pitch_deg
        self.anchor = client.simGetObjectPose(reference_camera)
        p, q = self.anchor.position, self.anchor.orientation
        if not all(math.isfinite(v) for v in
                   (p.x_val, p.y_val, p.z_val, q.x_val, q.y_val, q.z_val, q.w_val)):
            raise RuntimeError(f"reference camera {reference_camera!r} is unavailable")
        self.heading = math.atan2(2 * (q.w_val*q.z_val + q.x_val*q.y_val),
                                  1 - 2 * (q.y_val*q.y_val + q.z_val*q.z_val))
        self.ground_xy = (p.x_val + forward * math.cos(self.heading),
                          p.y_val + forward * math.sin(self.heading))
        self.refresh_spawn_offset()
        for name in ("BFLauncher_Base", "BFLauncher_Pedestal", "BFLauncher_Turntable",
                     "BFLauncher_Tray", "BFLauncher_Saddle"):
            if name in client.simListSceneObjects("^" + name + "$") and not client.simDestroyObject(name):
                raise RuntimeError(f"cannot remove previous launcher object {name}")
        self.pivot = None
        self.position = None
        self.yaw = self.heading
        self.pitch = 0.0
        self.thread = None
        self.stop_event = threading.Event()
        self.error = None
        self.released = False
        self.stowed = False
        self.last_aim_time = None
        self.pinned = False
        assets = set(client.simListAssets())
        if "Cube" in assets:
            self.mesh_asset, self.mesh_edge_m = "Cube", 1.0
        elif "EditorCube" in assets:
            self.mesh_asset, self.mesh_edge_m = "EditorCube", 2.56
        else:
            raise RuntimeError("no supported UE cube mesh for the virtual launcher")

    def refresh_spawn_offset(self):
        local = self.client.simGetVehiclePose(vehicle_name=self.VEHICLE).position
        world = self.client.simGetObjectPose(self.VEHICLE).position
        offset = (world.x_val-local.x_val, world.y_val-local.y_val, world.z_val-local.z_val)
        if not all(math.isfinite(value) for value in offset):
            raise RuntimeError("invalid launcher vehicle spawn frame")
        self.spawn_offset = offset

    def set_vehicle_pose(self, world_position, orientation):
        local = tuple(v-o for v, o in zip(world_position, self.spawn_offset))
        state = airsim.KinematicsState()
        state.position = airsim.Vector3r(*local)
        state.orientation = orientation
        state.linear_velocity = airsim.Vector3r()
        state.angular_velocity = airsim.Vector3r()
        state.linear_acceleration = airsim.Vector3r()
        state.angular_acceleration = airsim.Vector3r()
        self.client.simSetKinematics(state, True, vehicle_name=self.VEHICLE)

    def _object(self, name, position, scale, orientation=None):
        scale = tuple(v/self.mesh_edge_m for v in scale)
        pose = airsim.Pose(airsim.Vector3r(*position), orientation or airsim.Quaternionr(0, 0, 0, 1))
        if name in self.client.simListSceneObjects("^" + name + "$"):
            move_actor(self.client, name, pose)
            if not self.client.simSetObjectScale(name, airsim.Vector3r(*scale)):
                raise RuntimeError(f"cannot scale launcher object {name}")
        else:
            result = self.client.simSpawnObject(name, self.mesh_asset, pose, airsim.Vector3r(*scale), False, False)
            if result != name:
                raise RuntimeError(f"launcher object name mismatch: {result!r}, wanted {name!r}")

    def build(self, ground_z):
        x, y = self.ground_xy
        self._object("BFLauncher_Base", (x, y, ground_z - .09), (2.2, 2.2, .18))
        self._object("BFLauncher_Pedestal", (x, y, ground_z - .85), (.45, .45, 1.35))
        self._object("BFLauncher_Turntable", (x, y, ground_z - 1.55), (.85, .85, .15))
        self.pivot = (x, y, ground_z - 1.75)
        self._object("BFLauncher_Tray", self.pivot, (1.2, 1.4, .12))
        self._object("BFLauncher_Saddle", (x, y, self.pivot[2]-.25), (.5, .6, .4))
        self.position = (x, y, self.pivot[2] - self.MOUNT_OFFSET)
        if not self.client.simRunConsoleCommand("airsim.LauncherLock." + self.VEHICLE + " 1"):
            raise RuntimeError("AirSim plugin has no virtual launcher constraint; rebuild it first")
        self.pinned = True
        self.set_vehicle_pose(self.position, quaternion_rpy(0, 0, self.heading))

    def aim_at(self, target, client=None):
        if self.released:
            raise RuntimeError("cannot aim the launcher after release")
        client = client or self.client
        now = time.monotonic()
        dt = .05 if self.last_aim_time is None else max(.001, min(.1, now-self.last_aim_time))
        self.last_aim_time = now
        desired_yaw, desired_pitch, _ = launcher_angles(self.position, target, self.aim_pitch_deg)
        old_yaw, old_pitch = self.yaw, self.pitch
        limit = math.radians(20) * dt
        yaw_error = math.atan2(math.sin(desired_yaw-old_yaw), math.cos(desired_yaw-old_yaw))
        self.yaw += max(-limit, min(limit, yaw_error))
        self.pitch += max(-limit, min(limit, desired_pitch-old_pitch))
        q = quaternion_rpy(0, self.pitch, self.yaw)
        offset = rotate(q, (0, 0, -self.MOUNT_OFFSET))
        old_position = self.position
        self.position = tuple(p+d for p, d in zip(self.pivot, offset))
        move_actor(client, "BFLauncher_Turntable", airsim.Pose(
            airsim.Vector3r(self.pivot[0], self.pivot[1], self.pivot[2]+.2),
            quaternion_rpy(0, 0, self.yaw)))
        move_actor(client, "BFLauncher_Tray", airsim.Pose(airsim.Vector3r(*self.pivot), q))
        saddle = tuple(p+d for p, d in zip(self.pivot, rotate(q, (0, 0, -.25))))
        move_actor(client, "BFLauncher_Saddle", airsim.Pose(airsim.Vector3r(*saddle), q))
        local = tuple(v-o for v, o in zip(self.position, self.spawn_offset))
        state = airsim.KinematicsState()
        state.position = airsim.Vector3r(*local)
        state.orientation = q
        state.linear_velocity = airsim.Vector3r(*( (p-o)/dt for p, o in zip(self.position, old_position)))
        pitch_rate, yaw_rate = (self.pitch-old_pitch)/dt, (self.yaw-old_yaw)/dt
        state.angular_velocity = airsim.Vector3r(-yaw_rate * math.sin(self.pitch),
                                                pitch_rate, yaw_rate * math.cos(self.pitch))
        state.linear_acceleration = airsim.Vector3r()
        state.angular_acceleration = airsim.Vector3r()
        client.simSetKinematics(state, True, vehicle_name=self.VEHICLE)

    def start_aiming(self):
        def hold():
            client = airsim.MultirotorClient(ip=self.host, port=self.port, timeout_value=3)
            try:
                while not self.stop_event.is_set():
                    p = client.simGetObjectPose(self.TARGET).position
                    self.aim_at((p.x_val, p.y_val, p.z_val), client)
                    self.stop_event.wait(.05)
            except Exception as exc:
                self.error = exc
            finally:
                client.client.close()
        self.thread = threading.Thread(target=hold, name="launcher-aim", daemon=True)
        self.thread.start()

    def release(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("launcher actuator did not release")
        if self.pinned:
            if not self.client.simRunConsoleCommand("airsim.LauncherLock." + self.VEHICLE + " 0"):
                raise RuntimeError("AirSim launcher constraint failed to release")
            self.pinned = False
        self.released = True
        if self.error is not None:
            raise RuntimeError(f"launcher actuator failed: {self.error}")

    def stow(self):
        move_actor(self.client, "BFLauncher_Tray", airsim.Pose(
            airsim.Vector3r(*self.pivot), quaternion_rpy(0, 0, self.heading)))
        move_actor(self.client, "BFLauncher_Saddle", airsim.Pose(
            airsim.Vector3r(self.position[0], self.position[1], self.pivot[2]-.25),
            quaternion_rpy(0, 0, self.heading)))
        self.stowed = True

    def description(self):
        p = self.anchor.position
        return {"reference_actor": self.reference_camera,
                "reference_world_ned_m": (p.x_val, p.y_val, p.z_val),
                "pivot_world_ned_m": self.pivot, "mounted_world_ned_m": self.position,
                "vehicle_spawn_offset_ned_m": self.spawn_offset,
                "yaw_deg": math.degrees(self.yaw), "pitch_deg": math.degrees(self.pitch),
                "pitch_aim_deg": self.aim_pitch_deg}
