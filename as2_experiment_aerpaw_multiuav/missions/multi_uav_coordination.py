#!/usr/bin/env python3
"""Multi-UAV wireless-driven experiment (Stage 12).

Runs N AERPAW-backed vehicles as first-class entities through the shared closed loop:
fleet state -> coordination policy (formation + collision avoidance) -> per-vehicle
AS2 references. Demonstrates formation control with reactive separation; swap in
coverage / cooperative-planning / comm-aware policies the same way.

Each vehicle is an independent AERPAW UAV -> adapter -> AS2 platform (own namespace,
unique id, own state/command topics). Coordination uses the shared FleetState; commands
are still emitted per vehicle, so there is no cross-control.

Run (N drones' subsystems + runners up, e.g. run_aerpaw_experiment.sh NUM_DRONES=3):
    python3 multi_uav_coordination.py [num_drones] [duration_s]
"""

import sys
import time

import rclpy

from as2_experiment_aerpaw_multiuav.experiment import (
    ClosedLoopRunner,
    CollisionAvoidancePolicy,
    DroneInterfacePlatform,
    FormationPolicy,
    TopicMeasurementSource,
)

ALT = 25.0
LEADER_PATH = [(20.0, 0.0, ALT), (20.0, 20.0, ALT), (0.0, 20.0, ALT), (0.0, 0.0, ALT)]


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

    rclpy.init()
    ids = [f"drone{i}" for i in range(n)]
    platforms = [DroneInterfacePlatform(vid) for vid in ids]

    # Shared fleet state (states via PlatformInterface; RF via the adapter topic).
    source = TopicMeasurementSource(platforms[0].node, {vid: vid for vid in ids})

    # Formation: drone0 leader, others hold NE offsets; wrapped in collision avoidance.
    offsets = {"drone0": (0.0, 0.0)}
    for i in range(1, n):
        offsets[f"drone{i}"] = (10.0 * ((-1) ** i), 10.0 * i)
    formation = FormationPolicy("drone0", offsets, LEADER_PATH, speed=3.0)
    policy = CollisionAvoidancePolicy(formation, safe_distance=8.0, push=4.0)

    print(f"[multi-uav] arming + taking off {n} vehicles")
    for p in platforms:
        p.arm()
        p.takeoff(ALT, 3.0)
        p.offboard()
    time.sleep(6.0)

    print(f"[multi-uav] closed-loop coordination for {duration}s")
    runner = ClosedLoopRunner(platforms, source, policy)
    steps = runner.run(duration_s=duration, period_s=3.0)
    print(f"[multi-uav] {steps} coordination steps; landing")

    for p in platforms:
        p.land(1.0)
        p.close()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[multi-uav] Interrupted", file=sys.stderr)
        sys.exit(130)
