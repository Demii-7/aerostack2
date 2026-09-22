"""Wireless / RF / network metrics — experiment logic (Stage 9/10 placement).

RF and network measurements are part of the EXPERIMENT, not the platform adapter.
They originate on AERPAW's radio/RF infrastructure; the adapter only *transports*
them (opaque JSON on `<ns>/aerpaw/measurements`). This module defines the metric
type and the **source interface** different experiments use to consume them, plus a
few sources. Metrics are NEVER fabricated: absent values are `None`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Protocol, runtime_checkable

# Standard map of AERPAW wireless metric names. A source may emit any subset; the
# policy handles None gracefully.
KNOWN_METRICS = ("sinr_db", "rssi_dbm", "throughput_mbps", "packet_loss", "channel")


@dataclass
class RFMetrics:
    """Per-vehicle wireless/network metrics. None = not measured (never assume 0)."""

    vehicle_id: str
    sinr_db: Optional[float] = None
    rssi_dbm: Optional[float] = None
    throughput_mbps: Optional[float] = None
    packet_loss: Optional[float] = None
    channel: Optional[str] = None
    timestamp: Optional[float] = None
    extra: Optional[Dict[str, object]] = None

    @property
    def available(self) -> bool:
        return any(v is not None for v in
                   (self.sinr_db, self.rssi_dbm, self.throughput_mbps, self.packet_loss))

    @classmethod
    def from_json(cls, vehicle_id: str, payload: str, ts: Optional[float] = None) -> "RFMetrics":
        """Parse an adapter-passthrough metrics object (transport schema)."""
        try:
            d = json.loads(payload) if isinstance(payload, str) else dict(payload)
        except (ValueError, TypeError):
            d = {}
        known = {k: d.get(k) for k in ("sinr_db", "rssi_dbm", "throughput_mbps",
                                       "packet_loss", "channel")}
        extra = {k: v for k, v in d.items() if k not in known}
        m = cls(vehicle_id=vehicle_id, timestamp=ts, extra=extra or None, **known)
        return m


@runtime_checkable
class MeasurementSource(Protocol):
    """A pluggable provider of RF/network metrics keyed by vehicle id."""

    def poll(self, vehicle_ids: List[str]) -> Dict[str, RFMetrics]: ...


class UnavailableRFSource:
    """Default source: no measurement feed wired -> everything unavailable.

    Decision logic must degrade gracefully (geometry-only) rather than invent values.
    """

    def poll(self, vehicle_ids: List[str]) -> Dict[str, RFMetrics]:
        return {vid: RFMetrics(vehicle_id=vid) for vid in vehicle_ids}


class CallbackMeasurementSource:
    """Adapt any callable `fn(vehicle_ids) -> dict[id, RFMetrics]` into a source.

    Lets a researcher plug a custom AERPAW reader (e.g. OEO/data-collector client,
    a simulator, or `aerpawlib` checkpoint polling) without changing the harness.
    """

    def __init__(self, fn: Callable[[List[str]], Dict[str, RFMetrics]]) -> None:
        self._fn = fn

    def poll(self, vehicle_ids: List[str]) -> Dict[str, RFMetrics]:
        return self._fn(vehicle_ids)


class TopicMeasurementSource:
    """Consume the adapter's passthrough topic `<ns>/aerpaw/measurements` (Stage 10).

    Subscribes a standard `std_msgs/String` (JSON metrics payload) per vehicle
    namespace and caches the latest parsed `RFMetrics`. This is the experiment-side
    consumer that closes the wireless->robot loop. rclpy/std_msgs are imported lazily
    so the module stays importable (and boundary-checkable) without a ROS runtime.
    """

    TOPIC = "aerpaw/measurements"

    def __init__(self, node, ns_to_id: Dict[str, str]) -> None:
        """node: an rclpy Node; ns_to_id: maps each vehicle namespace -> vehicle id."""
        from std_msgs.msg import String  # lazy

        self._latest: Dict[str, RFMetrics] = {}
        for ns, vid in ns_to_id.items():
            topic = f"/{ns}/{self.TOPIC}" if ns else f"/{self.TOPIC}"
            node.create_subscription(String, topic, self._make_cb(vid), 10)

    def _make_cb(self, vehicle_id: str):
        def cb(msg) -> None:
            self._latest[vehicle_id] = RFMetrics.from_json(vehicle_id, msg.data)
        return cb

    def poll(self, vehicle_ids: List[str]) -> Dict[str, RFMetrics]:
        return {vid: self._latest.get(vid, RFMetrics(vehicle_id=vid)) for vid in vehicle_ids}
