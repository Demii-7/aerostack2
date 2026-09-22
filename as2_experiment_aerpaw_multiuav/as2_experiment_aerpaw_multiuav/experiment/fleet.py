"""Shared multi-UAV state (Stage 12).

A coherent whole-fleet view built from the per-vehicle ``VehicleState`` (independent
per-namespace interfaces from ``PlatformInterface``) plus optional per-vehicle RF
metrics. Coordination policies receive this to reason about *other* vehicles — the
basis for formation, cooperative planning, collision avoidance, coverage, distributed
planning and communication-aware coordination.

This is experiment-layer only; it consumes AS2 state, never the adapter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .interfaces import VehicleState
from .rf_metrics import RFMetrics


@dataclass
class FleetState:
    """Aggregate of all vehicles' states (+ wireless metrics) for one decision tick."""

    states: Dict[str, VehicleState] = field(default_factory=dict)
    metrics: Dict[str, RFMetrics] = field(default_factory=dict)

    def ids(self) -> List[str]:
        return list(self.states.keys())

    def position(self, vid: str) -> Tuple[float, float, float]:
        s = self.states.get(vid)
        return (s.x, s.y, s.z) if s else (0.0, 0.0, 0.0)

    def distance(self, a: str, b: str) -> float:
        ax, ay, az = self.position(a)
        bx, by, bz = self.position(b)
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)

    def nearest(self, vid: str, exclude_self: bool = True) -> Optional[Tuple[str, float]]:
        best: Optional[Tuple[str, float]] = None
        for other in self.states:
            if exclude_self and other == vid:
                continue
            d = self.distance(vid, other)
            if best is None or d < best[1]:
                best = (other, d)
        return best

    def centroid_xy(self) -> Tuple[float, float]:
        if not self.states:
            return (0.0, 0.0)
        n = len(self.states)
        return (sum(s.x for s in self.states.values()) / n,
                sum(s.y for s in self.states.values()) / n)

    def link(self, vid: str) -> Optional[RFMetrics]:
        return self.metrics.get(vid)

    def all_links_available(self) -> bool:
        return bool(self.metrics) and all(m.available for m in self.metrics.values())

    @classmethod
    def from_maps(cls, states: Dict[str, VehicleState],
                  metrics: Dict[str, RFMetrics]) -> "FleetState":
        return cls(states=dict(states), metrics=dict(metrics))
