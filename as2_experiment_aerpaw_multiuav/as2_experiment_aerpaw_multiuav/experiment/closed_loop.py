"""Closed-loop wireless-driven robotics harness (Stage 10).

Implements the research cycle generically:

    vehicle state + RF/wireless metrics  ->  RobotPolicy.decide()  ->  RobotCommand
        ->  PlatformInterface (AeroStack2 behaviour)  ->  vehicle moves  ->  ...

The loop knows NOTHING about a particular research algorithm: a policy is *injected*
(implementing ``RobotPolicy``). Metrics come from an injected ``MeasurementSource``
(e.g. TopicMeasurementSource fed by the AERPAW adapter, or a simulator). This is the
"one experiment" glue between robotics and wireless, kept in the EXPERIMENT layer.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, Sequence, Tuple, runtime_checkable

from .interfaces import PlatformInterface, VehicleState
from .rf_metrics import MeasurementSource, RFMetrics, UnavailableRFSource


@dataclass
class RobotCommand:
    """One closed-loop action for one vehicle (all coords AS2 ENU/odom, rad/speed SI)."""

    kind: str                       # 'go_to' | 'hover' | 'land' | 'takeoff' | 'follow_path'
    x: float = 0.0
    y: float = 0.0
    z: float = 25.0
    yaw: Optional[float] = None
    speed: float = 3.0
    path: Optional[Sequence[Tuple[float, float, float]]] = None
    height: float = 25.0

    @staticmethod
    def go_to(x, y, z, speed=3.0, yaw=None) -> "RobotCommand":
        return RobotCommand("go_to", x=x, y=y, z=z, yaw=yaw, speed=speed)

    @staticmethod
    def hover() -> "RobotCommand":
        return RobotCommand("hover")

    @staticmethod
    def land(speed=1.0) -> "RobotCommand":
        return RobotCommand("land", speed=speed)

    @staticmethod
    def takeoff(height=25.0, speed=3.0) -> "RobotCommand":
        return RobotCommand("takeoff", height=height, speed=speed)

    @staticmethod
    def follow_path(points: Sequence[Tuple[float, float, float]], speed=3.0) -> "RobotCommand":
        return RobotCommand("follow_path", path=list(points), speed=speed)


@runtime_checkable
class RobotPolicy(Protocol):
    """User-supplied research algorithm: map (states, metrics) -> per-vehicle command.

    Implement this to plug in ANY wireless-driven behaviour without touching the
    integration layer. Returning no entry for a vehicle = leave it as-is.
    """

    def decide(self, states: Dict[str, VehicleState],
               metrics: Dict[str, RFMetrics]) -> Dict[str, RobotCommand]: ...


class ClosedLoopRunner:
    """Drive the wireless<->robotics cycle by invoking a policy over platforms."""

    def __init__(self, platforms: Sequence[PlatformInterface],
                 source: Optional[MeasurementSource] = None,
                 policy: Optional[RobotPolicy] = None) -> None:
        self._platforms = {p.vehicle_id: p for p in platforms}
        self._source = source or UnavailableRFSource()
        self._policy = policy
        self.last_metrics: Dict[str, RFMetrics] = {}

    def _snapshot(self):
        ids = list(self._platforms.keys())
        states = {vid: self._platforms[vid].get_state() for vid in ids}
        metrics = self._source.poll(ids)
        return states, metrics

    def step(self) -> Dict[str, RobotCommand]:
        """One iteration: sense -> decide -> act. No algorithm lives here."""
        if self._policy is None:
            raise RuntimeError("ClosedLoopRunner has no RobotPolicy set")
        states, metrics = self._snapshot()
        self.last_metrics = metrics
        commands = self._policy.decide(states, metrics)
        for vid, cmd in commands.items():
            p = self._platforms.get(vid)
            if p is None:
                continue
            if cmd.kind == "go_to":
                p.go_to(cmd.x, cmd.y, cmd.z, speed=cmd.speed, yaw=cmd.yaw)
            elif cmd.kind == "hover":
                p.hover()
            elif cmd.kind == "land":
                p.land(cmd.speed)
            elif cmd.kind == "takeoff":
                p.takeoff(cmd.height, cmd.speed)
            elif cmd.kind == "follow_path" and cmd.path:
                p.follow_path(cmd.path, speed=cmd.speed)
        return commands

    def run(self, duration_s: float, period_s: float = 1.0,
            max_steps: Optional[int] = None) -> int:
        """Loop the cycle for `duration_s`. Returns the number of steps taken."""
        deadline = time.monotonic() + duration_s
        n = 0
        while time.monotonic() < deadline and (max_steps is None or n < max_steps):
            self.step()
            n += 1
            time.sleep(period_s)
        return n
