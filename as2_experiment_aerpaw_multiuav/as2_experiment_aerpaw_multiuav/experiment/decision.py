"""Decision making / objectives — experiment logic (Stage 9 placement).

A policy maps (vehicle states + RF metrics + plan) -> per-vehicle next targets.
This is the researcher's algorithm and the RF/network-aware decision layer. It
depends only on the experiment-side ``PlatformInterface`` / ``VehicleState`` /
``RFMetrics`` types - never on the platform adapter.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .coordination import FormationPlan
from .interfaces import VehicleState
from .rf_metrics import RFMetrics

Target = Tuple[float, float, float]  # (x, y, z) in AS2 ENU/odom


class TrianglePatrolPolicy:
    """Fly a leader along a square; followers hold triangle offsets.

    Demonstrates the seam: geometry comes from ``FormationPlan``, targets are returned
    for the mission runner to send via ``PlatformInterface``. If RF metrics are
    available an implementation hook could, e.g., bias the route by link quality;
    when unavailable it degrades to pure geometry (no fabricated RF data).
    """

    def __init__(self, altitude: float, square: Sequence[Tuple[float, float]],
                 speed: float = 3.0) -> None:
        self.altitude = altitude
        self.square = list(square)
        self.speed = speed

    def waypoints(self) -> List[Tuple[float, float]]:
        return list(self.square)

    def targets_for(self, plan: FormationPlan, rf: Optional[Dict[str, RFMetrics]] = None
                    ) -> Dict[str, List[Target]]:
        """Per-vehicle ordered target list (leader path + follower offsets).

        ``square`` entries are (north, east); offsets are (north, east). ENU target is
        x=east, y=north (see docs/COORDINATE_FRAMES.md). ``rf`` is accepted so an
        RF-aware policy can bias routing when metrics exist; when None/unavailable the
        policy uses pure geometry (no fabricated values).
        """
        out: Dict[str, List[Target]] = {}
        for vid, (off_n, off_e) in plan.offsets.items():
            path: List[Target] = []
            for north, east in self.square:
                x = east + off_e     # ENU x = east
                y = north + off_n    # ENU y = north
                path.append((x, y, self.altitude))
            out[vid] = path
        return out
