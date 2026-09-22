"""Experiment<->platform seam.

The ONLY contract experiment logic uses to touch a vehicle. Implemented over the
standard AeroStack2 API (``as2_python_api.DroneInterface``); it deliberately does
not reach into the AERPAW adapter (that layer lives in ``as2_platform_aerpaw`` and
owns translation/communication).

Keep new experiment code depending on ``PlatformInterface`` (and ``VehicleState``),
not on DroneInterface directly, so the boundary stays explicit and testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable


@dataclass
class VehicleState:
    """Platform-agnostic view of a vehicle, populated from AeroStack2 topics.

    Units/frame follow AeroStack2 conventions (see docs/COORDINATE_FRAMES.md):
    ENU metres, yaw radians (CCW, 0=East), battery fraction 0..1.
    """

    vehicle_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    battery_fraction: Optional[float] = None
    connected: bool = False
    mode: str = ""


@runtime_checkable
class PlatformInterface(Protocol):
    """High-level vehicle operations the experiment may request.

    These map onto AeroStack2 behaviours/controllers; implementations must translate
    them through the AS2 public API only. Coordinates are in the AS2 odom/earth (ENU)
    frame, speed in m/s, yaw in radians.
    """

    vehicle_id: str

    def arm(self) -> None: ...

    def offboard(self) -> None: ...

    def takeoff(self, height: float, speed: float) -> bool: ...

    def land(self, speed: float) -> bool: ...

    def go_to(self, x: float, y: float, z: float, speed: float,
              yaw: Optional[float] = None) -> bool: ...

    def follow_path(self, points: list, speed: float) -> bool: ...

    def hover(self) -> None: ...

    def get_state(self) -> VehicleState: ...

    def close(self) -> None: ...


class DroneInterfacePlatform:
    """``PlatformInterface`` over ``as2_python_api.DroneInterface``.

    AS2 imports are lazy so this module is importable (and boundary-checkable)
    without a ROS runtime.
    """

    def __init__(self, vehicle_id: str, ns: Optional[str] = None, verbose: bool = False,
                 use_sim_time: bool = False) -> None:
        from as2_python_api.drone_interface import DroneInterface  # lazy

        self.vehicle_id = vehicle_id
        self._ns = ns or vehicle_id
        self._di = DroneInterface(self._ns, verbose=verbose, use_sim_time=use_sim_time)

    @property
    def node(self):
        """The underlying rclpy Node (DroneInterface IS a Node), for creating
        subscriptions (e.g. TopicMeasurementSource) without exposing privates."""
        return self._di

    @property
    def namespace(self) -> str:
        return self._ns

    def arm(self) -> None:
        self._di.arm()

    def offboard(self) -> None:
        self._di.offboard()

    def takeoff(self, height: float, speed: float) -> bool:
        return bool(self._di.takeoff(height=height, speed=speed))

    def land(self, speed: float) -> bool:
        return bool(self._di.land(speed=speed))

    def go_to(self, x: float, y: float, z: float, speed: float,
              yaw: Optional[float] = None) -> bool:
        if yaw is None:
            return bool(self._di.go_to(x, y, z, speed=speed))
        return bool(self._di.go_to.go_to_with_yaw(x, y, z, speed=speed, yaw=yaw))

    def follow_path(self, points: list, speed: float) -> bool:
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path

        path = Path()
        path.header.frame_id = f"{self._ns}/odom"
        for x, y, z in points:
            ps = PoseStamped()
            ps.header.frame_id = path.header.frame_id
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return bool(self._di.follow_path(path, speed=speed))

    def hover(self) -> None:
        # AS2 hover: send a hover motion reference through the standard handler.
        from as2_motion_reference_handlers.hover_motion import HoverMotion

        HoverMotion(self._di).send_hover()

    def get_state(self) -> VehicleState:
        pose = getattr(self._di, "current_pose", None)
        st = VehicleState(vehicle_id=self.vehicle_id)
        if pose is not None:
            st.x = pose.pose.position.x
            st.y = pose.pose.position.y
            st.z = pose.pose.position.z
            st.connected = True
        return st

    def close(self) -> None:
        try:
            self._di.close()
        except Exception:
            pass
