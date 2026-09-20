#!/usr/bin/env python3
"""Triangle formation: leader flies a 50 m square at 25 m altitude.

Followers maintain a fixed NE lateral offset from the leader using
position control. Works on all three AS2 platform backends:
  - Multirotor Simulator
  - Gazebo
  - AERPAW Digital Twin / SITL

Run from a sourced AS2 workspace with:

  python3 triangle_formation.py

Uses threading for concurrent multi-drone control (DroneInterface calls
are synchronous/blocking).
"""

import sys
import time
import threading

import rclpy
from as2_python_api.drone_interface import DroneInterface

ALTITUDE = 25.0
SIDE = 50.0
SPEED = 3.0
OFFSETS = [(0, 0), (10, 0), (-10, 0)]


def _parallel(fn, drones, *args, **kwargs):
    """Run fn(drone, *args, **kwargs) concurrently for each drone, wait all."""
    threads = [
        threading.Thread(target=fn, args=(d, *args), kwargs=kwargs)
        for d in drones
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def _takeoff(drone, altitude, speed):
    drone.takeoff(height=altitude, speed=speed)


def _go_to(drone, x, y, z, speed):
    drone.go_to.go_to(x, y, z, speed=speed)


def _land(drone, speed):
    drone.land(speed=speed)


def run_mission():
    rclpy.init()
    drones = [DroneInterface(f"drone{i}", verbose=True) for i in range(3)]

    print("[triangle] Arming all drones...")
    _parallel(lambda d: d.arm(), drones)

    print(f"[triangle] Taking off all drones to {ALTITUDE} m...")
    _parallel(_takeoff, drones, ALTITUDE, SPEED)
    time.sleep(8.0)

    print("[triangle] Enabling offboard mode...")
    _parallel(lambda d: d.offboard(), drones)
    time.sleep(1.0)

    # Square waypoints: (north, east)
    square = [
        (SIDE, 0.0),
        (SIDE, SIDE),
        (0.0, SIDE),
        (0.0, 0.0),
    ]

    for wp_north, wp_east in square:
        print(f"[triangle] Flying to waypoint ({wp_north}, {wp_east})...")

        def _wp(drone, _n=wp_north, _e=wp_east, _idx=0):
            off_n, off_e = OFFSETS[_idx]
            _go_to(drone, _e + off_e, _n + off_n, ALTITUDE, SPEED)

        threads = [
            threading.Thread(target=_wp, args=(d,), kwargs={"_idx": i})
            for i, d in enumerate(drones)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    print("[triangle] Formation complete - landing...")
    _parallel(_land, drones, 0.5)
    time.sleep(5.0)

    for d in drones:
        d.disarm()

    for d in drones:
        d.destroy_node()
    rclpy.shutdown()
    print("[triangle] Done!")


if __name__ == "__main__":
    try:
        run_mission()
    except KeyboardInterrupt:
        print("\n[triangle] Interrupted", file=sys.stderr)
