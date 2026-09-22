"""Multi-UAV coordination logic — experiment-specific (Stage 9 placement).

Formation geometry / role assignment is an *experiment* concern and lives here, on
the experiment side, NOT in the platform adapter. The adapter only moves a single
vehicle when told to; how several vehicles are coordinated is decided here and sent
through ``PlatformInterface``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple


@dataclass
class FormationPlan:
    """A leader + follower offset assignment in the AS2 ENU/odom frame."""

    leader: str
    # vehicle_id -> (north offset, east offset) relative to the leader target
    offsets: Dict[str, Tuple[float, float]] = field(default_factory=dict)


def triangle_offsets(north: float = 0.0, east: float = 0.0) -> List[Tuple[float, float]]:
    """Default triangle NE offsets for (leader, f1, f2) - previously inline in the mission."""
    return [(north, east), (north + 10.0, east), (north - 10.0, east)]


def assign_roles(vehicle_ids: Sequence[str],
                 offsets: Sequence[Tuple[float, float]]) -> FormationPlan:
    """Assign the first vehicle as leader, others as followers with the given offsets."""
    if not vehicle_ids:
        raise ValueError("empty vehicle list")
    leader = vehicle_ids[0]
    plan = FormationPlan(leader=leader, offsets={leader: tuple(offsets[0])})
    for vid, off in zip(vehicle_ids[1:], offsets[1:]):
        plan.offsets[vid] = tuple(off)
    return plan
