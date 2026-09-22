#!/usr/bin/env python3
"""Stage-8 reuse demo: drive an AERPAW-backed UAV with UNMODIFIED AeroStack2
behaviors / APIs.

Nothing in this script is AERPAW-specific: it uses the standard
`as2_python_api.DroneInterface`, whose behaviors (takeoff, go_to, follow_path,
land) talk to the motion controller, which talks to the AERPAW platform through
the AS2 `as2::AerialPlatform` abstraction. That is the point of the integration -
a researcher writes a normal AS2 mission and it runs on AERPAW unchanged.

Behaviors exercised (all existing AS2, position-mode plugins -> AERPAW-compatible):
  - takeoff  : TakeoffModule  (takeoff_plugin_platform)
  - go_to    : GoToModule      (go_to_plugin_position)      -> navigation
  - follow_path: FollowPathModule (follow_path_plugin_position) -> trajectory/path following
  - land     : LandModule      (land_plugin_platform)

Run against an AERPAW platform namespace (see as2_stack.launch.py / Config C):

    python3 reuse_capabilities.py            # defaults to drone0

Requires the subsystem running with the position plugins (no TRAJECTORY/offboard
mode needed, so it works on the AERPAW testbed). See docs/AS2_CAPABILITIES.md.
"""

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from as2_python_api.drone_interface import DroneInterface

ALT = 25.0          # AERPAW copter min safe altitude (m)
SPEED = 3.0


def _make_path(points):
    """Build a nav_msgs/Path from [(x,y,z), ...] in the odom/earth frame."""
    path = Path()
    path.header.frame_id = "drone0/odom"
    for x, y, z in points:
        ps = PoseStamped()
        ps.header.frame_id = path.header.frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else "drone0"
    rclpy.init()
    drone = DroneInterface(ns, verbose=True)

    print("[reuse] arm + offboard (AS2 platform services)")
    drone.arm()
    drone.offboard()

    print("[reuse] takeoff (AS2 Takeoff behavior)")
    if not drone.takeoff(height=ALT, speed=SPEED):
        print("[reuse] takeoff failed - is the AERPAW platform + estimator up?")
        drone.destroy_node()
        rclpy.shutdown()
        return 1

    print("[reuse] go_to waypoints (AS2 GoTo behavior)")
    for x, y in [(30.0, 0.0), (30.0, 30.0), (0.0, 30.0), (0.0, 0.0)]:
        drone.go_to(x, y, ALT, speed=SPEED)

    print("[reuse] follow_path (AS2 FollowPath behavior, position plugin)")
    p = _make_path([(0.0, 0.0, ALT), (20.0, 10.0, ALT), (40.0, 0.0, ALT)])
    p.header.frame_id = f"{ns}/odom"
    for pose in p.poses:
        pose.header.frame_id = f"{ns}/odom"
    drone.follow_path(p, speed=SPEED)

    print("[reuse] land (AS2 Land behavior)")
    drone.land(speed=1.0)

    drone.disarm()
    drone.destroy_node()
    rclpy.shutdown()
    print("[reuse] Done - ran entirely on stock AeroStack2 behaviors.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[reuse] Interrupted", file=sys.stderr)
        sys.exit(130)
