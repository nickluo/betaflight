#!/usr/bin/env python3
"""AirSim interception target: flies the TargetDrone balloon to a standoff point.

Deploys through a checked climb/transit corridor, verifies settled arrival,
and cancels native flight tasks on every exit. Ground truth controls only this
simulation target, never the interceptor's visual guidance.

Run alongside the interception mission (airsim_intercept_mission.py); the two
scripts use independent AirSim RPC clients.
"""

import argparse
import itertools
import json
import math
import signal
import sys
import threading
import time

import cosysairsim as airsim

INTERCEPTOR_NAME = "Copter"
TARGET_NAME = "TargetDrone"


def world_to_geopoint(position, origin):
    """Invert the WGS84 local tangent frame used by AirSim's visibility API."""
    lat, lon = math.radians(origin.latitude), math.radians(origin.longitude)
    a, e2 = 6378137.0, 6.69437999014e-3
    slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    n = a / math.sqrt(1 - e2 * slat * slat)
    north, east, down = position
    x = (n + origin.altitude) * clat * clon - slat * clon * north - slon * east - clat * clon * down
    y = (n + origin.altitude) * clat * slon - slat * slon * north + clon * east - clat * slon * down
    z = (n * (1 - e2) + origin.altitude) * slat + clat * north - slat * down
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1 - e2))
    for _ in range(8):
        n = a / math.sqrt(1 - e2 * math.sin(latitude) ** 2)
        latitude = math.atan2(z + e2 * n * math.sin(latitude), horizontal)
    altitude = horizontal / math.cos(latitude) - n
    point = airsim.GeoPoint()
    point.latitude, point.longitude, point.altitude = (
        math.degrees(latitude), math.degrees(math.atan2(y, x)), altitude)
    return point


def park_target(client):
    """Cancel UE-side tasks and freeze a disarmed target without a return flight."""
    client.cancelLastTask(vehicle_name=TARGET_NAME)
    if not client.simRunConsoleCommand(f"airsim.LauncherLock.{TARGET_NAME} 1"):
        raise RuntimeError("AirSim target parking constraint unavailable; rebuild the launcher plugin")
    state = client.simGetGroundTruthKinematics(vehicle_name=TARGET_NAME)
    state.linear_velocity = airsim.Vector3r()
    state.angular_velocity = airsim.Vector3r()
    state.linear_acceleration = airsim.Vector3r()
    state.angular_acceleration = airsim.Vector3r()
    client.simSetKinematics(state, True, vehicle_name=TARGET_NAME)
    client.enableApiControl(True, vehicle_name=TARGET_NAME)
    if not client.isApiControlEnabled(vehicle_name=TARGET_NAME):
        raise RuntimeError("balloon API control required for disarming was refused")
    # SimpleFlight returns False for an already inactive/disarmed vehicle.
    if not client.armDisarm(False, vehicle_name=TARGET_NAME):
        print("[balloon] target was already inactive/disarmed", flush=True)
    client.enableApiControl(False, vehicle_name=TARGET_NAME)
    if client.isApiControlEnabled(vehicle_name=TARGET_NAME):
        raise RuntimeError("balloon API control was not released")
    print("[balloon] native task cancelled; target parked and API released", flush=True)


class BalloonTarget:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port, timeout_value=5)
        self.client.confirmConnection()
        park_target(self.client)
        time.sleep(.1)
        self.standoff = (args.x, args.y, args.z)
        local = self.client.simGetVehiclePose(vehicle_name=TARGET_NAME).position
        world = self.client.simGetObjectPose(TARGET_NAME).position
        self.spawn_offset = (world.x_val - local.x_val, world.y_val - local.y_val,
                             world.z_val - local.z_val)
        self.local_standoff = tuple(p - o for p, o in zip(self.standoff, self.spawn_offset))
        settings = json.loads(self.client.getSettingsString())
        origin = settings.get("OriginGeopoint", {})
        self.geo_origin = airsim.GeoPoint()
        self.geo_origin.latitude = origin.get("Latitude", 47.641468)
        self.geo_origin.longitude = origin.get("Longitude", -122.140165)
        self.geo_origin.altitude = origin.get("Altitude", 122.0)
        self.route = []
        self.flight_timestamp = None

    def vehicle_position(self, name):
        p = self.client.simGetObjectPose(name).position
        return (p.x_val, p.y_val, p.z_val)

    def reset_to_spawn(self):
        """Place a stationary target at a checked staging point before arming."""
        park_target(self.client)
        if not self.client.simSetObjectScale(
                TARGET_NAME, airsim.Vector3r(*(self.args.scale,) * 3)):
            raise RuntimeError("balloon scale was not applied")
        state = airsim.KinematicsState()
        state.position = airsim.Vector3r()
        state.orientation = airsim.Quaternionr(0, 0, 0, 1)
        self.client.simSetKinematics(state, True, vehicle_name=TARGET_NAME)
        if self.args.spawn_x is not None:
            world = (self.args.spawn_x, self.args.spawn_y, self.args.spawn_z)
        else:
            world = (self.spawn_offset[0], self.spawn_offset[1] + 5, self.spawn_offset[2] - 5)
        self.route = self.plan_route(world)
        state = airsim.KinematicsState()
        state.position = airsim.Vector3r(*(p-o for p, o in zip(self.route[0], self.spawn_offset)))
        state.orientation = airsim.Quaternionr(0, 0, 0, 1)
        self.client.simSetKinematics(state, True, vehicle_name=TARGET_NAME)
        time.sleep(.1)
        if math.dist(self.vehicle_position(TARGET_NAME), self.route[0]) > .1:
            raise RuntimeError("balloon staging pose was not applied")
        print(f"[balloon] checked staging/climb/transit route: {self.route}", flush=True)

    def corridor_clear(self, route):
        clearance = max(2.0, self.args.scale * 1.5)
        offsets = itertools.product((-clearance, 0, clearance), repeat=3)
        for offset in offsets:
            points = [world_to_geopoint(tuple(p+d for p, d in zip(point, offset)),
                                       self.geo_origin) for point in route]
            for start, end in zip(points, points[1:]):
                if not self.client.simTestLineOfSightBetweenPoints(start, end):
                    return False
        return True

    def plan_route(self, preferred):
        for lift in (0, 5, 10, 20):
            for dx, dy in ((0, 0), (0, -8), (0, 8), (8, 0), (-8, 0)):
                spawn = (preferred[0] + dx, preferred[1] + dy, preferred[2] - lift)
                climb = (spawn[0], spawn[1], min(spawn[2] - 3, self.standoff[2]))
                route = [spawn, climb, (self.standoff[0], self.standoff[1], climb[2]), self.standoff]
                if self.corridor_clear(route):
                    return route
                print(f"[balloon] rejected obstructed staging route from {spawn}", flush=True)
        raise RuntimeError("no obstacle-clear balloon staging/climb/transit corridor; target not armed")

    def command_velocity(self, velocity, duration=.2):
        if not self.client.moveByVelocityAsync(
                *velocity, duration=duration, vehicle_name=TARGET_NAME).get():
            raise RuntimeError("balloon velocity command failed/cancelled")

    def fly_leg(self, goal):
        initial_distance = math.dist(self.vehicle_position(TARGET_NAME), goal)
        initial_velocity = self.client.simGetGroundTruthKinematics(vehicle_name=TARGET_NAME).linear_velocity
        error = initial_distance
        velocity = (initial_velocity.x_val, initial_velocity.y_val, initial_velocity.z_val)
        deadline = time.monotonic() + max(20, initial_distance / self.args.cruise * 3 + 10)
        progress_time, progress_distance = time.monotonic(), initial_distance
        since = None
        while time.monotonic() < deadline:
            if self.stop.is_set():
                raise InterruptedError("balloon deployment cancelled")
            position = self.vehicle_position(TARGET_NAME)
            k = self.client.simGetGroundTruthKinematics(vehicle_name=TARGET_NAME)
            velocity = (k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val)
            error = math.dist(position, goal)
            collision = self.client.simGetCollisionInfo(vehicle_name=TARGET_NAME)
            if collision.has_collided and collision.time_stamp > self.flight_timestamp:
                raise RuntimeError(f"balloon collision with {collision.object_name} at {position}; "
                                   f"goal={goal}, error={error:.2f} m")
            if error < .75 and math.hypot(*velocity) < .25:
                since = since or time.monotonic()
                if time.monotonic() - since >= 2:
                    print(f"[balloon] leg settled at {position}, error {error:.2f} m", flush=True)
                    return error
            else:
                since = None
            if progress_distance - error > .25:
                progress_time, progress_distance = time.monotonic(), error
            elif error >= .75 and time.monotonic() - progress_time > 8:
                raise RuntimeError(f"balloon stalled en route: goal={goal}, position={position}, "
                                   f"error={error:.2f} m")
            command = tuple(.8 * (g-p) - .4 * v for g, p, v in zip(goal, position, velocity))
            speed = math.hypot(*command)
            if speed > self.args.cruise:
                command = tuple(v * self.args.cruise / speed for v in command)
            self.command_velocity(command)
        raise RuntimeError(f"balloon did not settle at waypoint {goal}: error={error:.2f} m, "
                           f"speed={math.hypot(*velocity):.2f} m/s")

    def fly_to_standoff(self):
        self.client.enableApiControl(True, TARGET_NAME)
        if not self.client.armDisarm(True, TARGET_NAME):
            raise RuntimeError("balloon arming was refused")
        self.flight_timestamp = self.client.getMultirotorState(vehicle_name=TARGET_NAME).timestamp
        if not self.client.simRunConsoleCommand(f"airsim.LauncherLock.{TARGET_NAME} 0"):
            raise RuntimeError("balloon staging constraint failed to release")
        for goal in self.route[1:]:
            error = self.fly_leg(goal)
        pos = self.vehicle_position(TARGET_NAME)
        print(f"[balloon] standoff reached: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) "
              f"error {error:.2f} m", flush=True)
        return error

    def hold_orbit(self):
        """Hover at the standoff point, or orbit it with a small radius."""
        radius, speed = self.args.orbit_radius, self.args.orbit_speed
        omega = speed / radius if radius > 0 else 0.0
        dt = 0.1
        angle = 0.0
        start = time.monotonic()
        while not self.stop.is_set():
            if self.args.motion == "orbit":
                vx = speed * math.sin(angle)
                vy = -speed * math.cos(angle)
                state = self.client.getMultirotorState(vehicle_name=TARGET_NAME)
                z = state.kinematics_estimated.position.z_val
                vz = (self.local_standoff[2] - z) / dt
                self.command_velocity((vx, vy, max(-2.0, min(2.0, vz))), duration=dt)
                angle += omega * dt
                if angle > 2 * math.pi:
                    angle -= 2 * math.pi
            else:
                pos = self.vehicle_position(TARGET_NAME)
                self.command_velocity(tuple(.8 * (g-p) for g, p in zip(self.standoff, pos)))
            if self.args.duration > 0 and time.monotonic() - start > self.args.duration:
                break
            if int(time.monotonic() - start) % 5 == 0:
                pos = self.vehicle_position(TARGET_NAME)
                drone = self.vehicle_position(INTERCEPTOR_NAME)
                rel = (pos[0] - drone[0], pos[1] - drone[1], pos[2] - drone[2])
                horiz = math.hypot(rel[0], rel[1])
                print(f"[balloon] t={time.monotonic() - start:5.1f}s pos=({pos[0]:6.1f},{pos[1]:6.1f},{pos[2]:6.1f}) "
                      f"rel horiz {horiz:5.1f} m alt {-rel[2]:5.1f} m", flush=True)
                time.sleep(1.0)  # avoid duplicate prints within the same second

    def run(self):
        try:
            if self.args.reset:
                self.reset_to_spawn()
            else:
                position = self.vehicle_position(TARGET_NAME)
                self.route = self.plan_route(position)
                if self.route[0] != position:
                    raise RuntimeError("current balloon position has no clear route; use --reset for staging")
            self.fly_to_standoff()
            self.hold_orbit()
        finally:
            try:
                park_target(self.client)
            finally:
                self.client.client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--x", type=float, default=60.0, help="standoff X (NED, m)")
    parser.add_argument("--y", type=float, default=36.0, help="standoff Y (NED, m)")
    parser.add_argument("--z", type=float, default=-45.0, help="standoff Z (NED, m; negative = aloft)")
    parser.add_argument("--motion", choices=("hover", "orbit"), default="hover")
    parser.add_argument("--orbit-radius", type=float, default=5.0)
    parser.add_argument("--orbit-speed", type=float, default=1.5)
    parser.add_argument("--cruise", type=float, default=8.0, help="transit speed (m/s)")
    parser.add_argument("--scale", type=float, default=1.5, help="visual scale factor")
    parser.add_argument("--duration", type=float, default=0.0, help="0 = until stopped")
    parser.add_argument("--reset", action=argparse.BooleanOptionalAction, default=True,
                        help="reset to a checked staging point before arming (default enabled)")
    parser.add_argument("--spawn-x", type=float, default=None, help="optional spawn X in common-world NED")
    parser.add_argument("--spawn-y", type=float, default=None, help="optional spawn Y in common-world NED")
    parser.add_argument("--spawn-z", type=float, default=None, help="optional spawn Z in common-world NED")
    parser.add_argument("--no-land", dest="land", action="store_false",
                        help="compatibility flag: cleanup now always cancels, parks and releases the target")
    args = parser.parse_args()
    if any(v is not None for v in (args.spawn_x, args.spawn_y, args.spawn_z)) and \
            any(v is None for v in (args.spawn_x, args.spawn_y, args.spawn_z)):
        parser.error("--spawn-x, --spawn-y and --spawn-z must be provided together")
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z, args.cruise, args.scale)) \
            or args.cruise <= 0 or args.scale <= 0:
        parser.error("target coordinates must be finite; cruise and scale must be positive")
    if args.spawn_x is not None and not all(math.isfinite(v) for v in
                                           (args.spawn_x, args.spawn_y, args.spawn_z)):
        parser.error("spawn coordinates must be finite")

    target = BalloonTarget(args)

    def request_stop(signum, frame):  # noqa: ARG001
        target.stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    target.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
