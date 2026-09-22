"""Example wireless-driven policies (illustrative; NOT wired into the platform).

These show how a researcher plugs an algorithm into ``closed_loop.RobotPolicy``. The
integration layer ships none of these - swap in your own decision logic; the loop and
the adapter do not change.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from .closed_loop import RobotCommand
from .interfaces import VehicleState
from .rf_metrics import RFMetrics


class SignalAwareWaypointPolicy:
    """Fly each vehicle to the waypoint whose measured link quality is best.

    Purely illustrative: candidate waypoints are given up front; for each vehicle it
    picks the candidate with the highest available score. When RF metrics are
    unavailable it falls back to the first candidate (never invents a measurement).
    """

    def __init__(self, candidates: Sequence[Tuple[float, float, float]],
                 speed: float = 3.0, prefer: str = "sinr_db", maximize: bool = True) -> None:
        self.candidates = list(candidates)
        self.speed = speed
        self.prefer = prefer
        self.maximize = maximize

    def _score(self, m: Optional[RFMetrics]) -> Optional[float]:
        if m is None or not m.available:
            return None
        v = getattr(m, self.prefer, None)
        return None if v is None else float(v)

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]:
        out: Dict[str, RobotCommand] = {}
        fallback = self.candidates[0]
        best = fallback
        # React to the freshest measurement across the fleet to demonstrate the loop.
        # A real algorithm would model per-waypoint signal; the point is the seam.
        avail = [self._score(metrics.get(vid)) for vid in states]
        avail = [a for a in avail if a is not None]
        if avail:
            mean = sum(avail) / len(avail)
            threshold = 0.1 if self.prefer == "packet_loss" else 0.0
            is_good = mean < threshold if not self.maximize else mean > threshold
            best = self.candidates[-1] if is_good else self.candidates[0]
        for vid in states:
            out[vid] = RobotCommand.go_to(best[0], best[1], best[2], speed=self.speed)
        return out


class StaticHoverPolicy:
    """Minimal policy that just hovers everything - useful as a smoke test."""

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]:
        return {vid: RobotCommand.hover() for vid in states}


# --- Multi-UAV coordination (Stage 12): policies that reason about the FLEET ------
# Each ``decide`` receives the full dict of every vehicle's state + metrics, i.e. a
# shared multi-UAV view. Formation / collision-avoidance / coverage examples below
# are illustrative - a researcher swaps in their own without touching the platform.

from .fleet import FleetState  # noqa: E402  (after base policy for readability)


class FormationPolicy:
    """Leader-follower formation using shared fleet state.

    The leader advances along ``leader_path`` (cycling waypoints); every follower
    holds a fixed NE offset from the leader's *current* position (from FleetState).
    Demonstrates formation control over independently-namespaced vehicles.
    """

    def __init__(self, leader: str, offsets: Dict[str, Tuple[float, float]],
                 leader_path: Sequence[Tuple[float, float, float]], speed: float = 3.0) -> None:
        self.leader = leader
        self.offsets = offsets            # vid -> (north, east) offset
        self.leader_path = list(leader_path)
        self.speed = speed
        self._i = 0

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]:
        fleet = FleetState.from_maps(states, metrics)
        out: Dict[str, RobotCommand] = {}
        # leader: next waypoint
        target = self.leader_path[self._i % len(self.leader_path)]
        self._i += 1
        out[self.leader] = RobotCommand.go_to(target[0], target[1], target[2], speed=self.speed)
        # followers: leader position + offset (shared state)
        lx, ly, lz = fleet.position(self.leader)
        for vid in states:
            if vid == self.leader:
                continue
            off_n, off_e = self.offsets.get(vid, (0.0, 0.0))
            out[vid] = RobotCommand.go_to(lx + off_e, ly + off_n, lz, speed=self.speed)
        return out


class CollisionAvoidancePolicy:
    """Wrap a base policy and push any too-close vehicle pair apart (separation).

    Uses fleet state to detect violations of a minimum separation and nudges the
    commanded targets along the line between the pair - a minimal reactive collision
    avoidance that composes with any inner policy.
    """

    def __init__(self, inner, safe_distance: float = 10.0, push: float = 5.0) -> None:
        self.inner = inner
        self.safe = safe_distance
        self.push = push

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]:
        fleet = FleetState.from_maps(states, metrics)
        cmds = self.inner.decide(states, metrics)
        ids = list(states.keys())
        # Reactive separation along the line joining each too-close PAIR (by CURRENT
        # position), applied to their commanded targets so they pull apart mid-flight.
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                if fleet.distance(a, b) >= self.safe:
                    continue
                ax, ay, _ = fleet.position(a)
                bx, by, _ = fleet.position(b)
                dx, dy = ax - bx, ay - by
                n = math.hypot(dx, dy)
                if n < 1e-6:            # coincident: pick a deterministic axis
                    dx, dy, n = 1.0, 0.0, 1.0
                ux, uy = dx / n, dy / n
                if a in cmds:
                    cmds[a].x += ux * self.push
                    cmds[a].y += uy * self.push
                if b in cmds:
                    cmds[b].x -= ux * self.push
                    cmds[b].y -= uy * self.push
        return cmds


class CoverageAssignmentPolicy:
    """Greedy nearest-neighbour assignment of waypoints to vehicles (coverage).

    Each vehicle is assigned a distinct unclaimed waypoint (nearest first) - a simple
    distributed coverage / task-assignment over shared fleet state.
    """

    def __init__(self, waypoints: Sequence[Tuple[float, float, float]], speed: float = 3.0,
                 altitude: Optional[float] = None) -> None:
        self.waypoints = list(waypoints)
        self.speed = speed
        self.altitude = altitude

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]:
        out: Dict[str, RobotCommand] = {}
        remaining = list(self.waypoints)
        for vid, s in states.items():
            if not remaining:
                break
            z = self.altitude if self.altitude is not None else s.z
            best = min(remaining, key=lambda w: (w[0] - s.x) ** 2 + (w[1] - s.y) ** 2)
            remaining.remove(best)
            out[vid] = RobotCommand.go_to(best[0], best[1], z, speed=self.speed)
        return out
