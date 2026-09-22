"""Experiment-logic layer (Stages 9-10).

Public surface for researchers: talk to vehicles via ``PlatformInterface``, read
``VehicleState``, poll ``RFMetrics`` (``MeasurementSource``), plan coordination and
decisions, and run the wireless-driven **closed loop** via ``ClosedLoopRunner`` + an
injected ``RobotPolicy``. Nothing here depends on the AERPAW platform adapter.
"""

from .closed_loop import ClosedLoopRunner, RobotCommand, RobotPolicy
from .coordination import FormationPlan, assign_roles, triangle_offsets
from .decision import TrianglePatrolPolicy
from .fleet import FleetState
from .interfaces import DroneInterfacePlatform, PlatformInterface, VehicleState
from .policies import (
    CollisionAvoidancePolicy,
    CoverageAssignmentPolicy,
    FormationPolicy,
    SignalAwareWaypointPolicy,
    StaticHoverPolicy,
)
from .rf_metrics import (
    MeasurementSource,
    RFMetrics,
    TopicMeasurementSource,
    UnavailableRFSource,
    CallbackMeasurementSource,
)

__all__ = [
    "PlatformInterface",
    "DroneInterfacePlatform",
    "VehicleState",
    "RFMetrics",
    "MeasurementSource",
    "UnavailableRFSource",
    "CallbackMeasurementSource",
    "TopicMeasurementSource",
    "FormationPlan",
    "assign_roles",
    "triangle_offsets",
    "TrianglePatrolPolicy",
    "FleetState",
    "RobotCommand",
    "RobotPolicy",
    "ClosedLoopRunner",
    "SignalAwareWaypointPolicy",
    "StaticHoverPolicy",
    "FormationPolicy",
    "CollisionAvoidancePolicy",
    "CoverageAssignmentPolicy",
]
