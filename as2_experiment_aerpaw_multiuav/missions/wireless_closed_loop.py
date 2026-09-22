#!/usr/bin/env python3
"""Wireless-driven closed-loop experiment (Stage 10).

The central research loop, run entirely with stock AeroStack2 + the AERPAW adapter:

    UAV moves -> wireless env changes -> AERPAW measures RF
      -> adapter forwards metrics to <ns>/aerpaw/measurements (transport only)
      -> TopicMeasurementSource (experiment) -> RobotPolicy.decide()
      -> PlatformInterface (AS2 behaviour) -> UAV moves -> ...

The policy is *injected*: swap in your own algorithm without touching the adapter.
Nothing about the RF algorithm is hard-coded in the platform layer.

Prereqs: AERPAW subsystem + runners up, with the runner emitting measurements:
    aerpaw_as2_runner.py ... --measure-checkpoint "sinr_db=radio/snr,rssi_dbm=radio/rssi"

Run:
    python3 wireless_closed_loop.py [num_drones] [duration_s]

If no measurements ever arrive (e.g. plain SITL), the policy degrades to geometry
fallback - it never invents RF values.
"""

import sys
import time

import rclpy

from as2_experiment_aerpaw_multiuav.experiment import (
    ClosedLoopRunner,
    DroneInterfacePlatform,
    TopicMeasurementSource,
)
from as2_experiment_aerpaw_multiuav.experiment.policies import SignalAwareWaypointPolicy

ALT = 25.0
# Candidate waypoints (ENU/odom). The policy selects among them from live RF metrics.
CANDIDATES = [(10.0, 0.0, ALT), (30.0, 0.0, ALT), (50.0, 0.0, ALT)]


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    rclpy.init()
    ids = [f"drone{i}" for i in range(n)]
    platforms = [DroneInterfacePlatform(vid) for vid in ids]

    # Consume AERPAW measurements from the adapter's passthrough topic.
    source = TopicMeasurementSource(platforms[0].node, {vid: vid for vid in ids})
    policy = SignalAwareWaypointPolicy(CANDIDATES, speed=3.0, prefer="sinr_db", maximize=True)

    print("[closed-loop] arm + offboard + takeoff")
    for p in platforms:
        p.arm()
        p.takeoff(ALT, 3.0)
        p.offboard()
    time.sleep(6.0)

    print(f"[closed-loop] running {duration}s; RF-driven decisions each 2s")
    runner = ClosedLoopRunner(platforms, source, policy)
    steps = runner.run(duration_s=duration, period_s=2.0)
    print(f"[closed-loop] {steps} iterations; last RF sample available="
          f"{any(m.available for m in runner.last_metrics.values())}")

    print("[closed-loop] land")
    for p in platforms:
        p.land(1.0)
        p.close()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[closed-loop] Interrupted", file=sys.stderr)
        sys.exit(130)
