#!/usr/bin/env python3
"""Triangle formation mission - thin runner over the experiment layer (Stage 9).

All the *how* (vehicle I/O) is behind ``PlatformInterface``; the coordination and
decision logic live in ``as2_experiment_aerpaw_multiuav.experiment``; the platform
adapter (``as2_platform_aerpaw``) is not referenced here at all. A mission author
writes against AeroStack2 behaviours, not AERPAW.

Run (with the AERPAW subsystem + runners up, Config C / run_aerpaw_experiment.sh):

    python3 triangle_formation.py [num_drones]
"""

import sys
import threading
import time

import rclpy

from as2_experiment_aerpaw_multiuav.experiment import (
    DroneInterfacePlatform,
    TrianglePatrolPolicy,
    UnavailableRFSource,
    assign_roles,
    triangle_offsets,
)

ALTITUDE = 25.0
SIDE = 50.0
SPEED = 3.0
# Leader path: square waypoints as (north, east).
SQUARE = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]


def _parallel(fn, items, *args, **kwargs):
    threads = [threading.Thread(target=fn, args=(it, *args), kwargs=kwargs) for it in items]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def _fly_vehicle(platform, path, speed):
    for x, y, z in path:
        platform.go_to(x, y, z, speed=speed)


def run_mission(num_drones: int = 3) -> int:
    rclpy.init()
    vehicles = [f"drone{i}" for i in range(num_drones)]
    platforms = [DroneInterfacePlatform(vid) for vid in vehicles]

    # Experiment logic: roles + geometry + decision policy (RF-aware when available).
    plan = assign_roles(vehicles, triangle_offsets())
    policy = TrianglePatrolPolicy(altitude=ALTITUDE, square=SQUARE, speed=SPEED)
    rf = UnavailableRFSource().poll(vehicles)   # degrades to geometry-only
    targets = policy.targets_for(plan, rf)

    print("[triangle] arm + offboard + takeoff")
    _parallel(lambda p: p.arm(), platforms)
    _parallel(lambda p: p.takeoff(ALTITUDE, SPEED), platforms)
    time.sleep(8.0)
    _parallel(lambda p: p.offboard(), platforms)
    time.sleep(1.0)

    print("[triangle] fly leader path + follower offsets (decision from experiment layer)")
    for i in range(len(policy.waypoints())):
        threads = [
            threading.Thread(target=_fly_vehicle, args=(p, [targets[p.vehicle_id][i]], SPEED))
            for p in platforms
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    print("[triangle] land")
    _parallel(lambda p: p.land(1.0), platforms)
    for p in platforms:
        p.close()
    rclpy.shutdown()
    print("[triangle] done")
    return 0


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    try:
        sys.exit(run_mission(n))
    except KeyboardInterrupt:
        print("\n[triangle] Interrupted", file=sys.stderr)
        sys.exit(130)
