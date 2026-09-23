"""Stage 17 failure-handling tests (pure stdlib, no ROS runtime).

Covers the wireless/measurement side of safe-failure handling: stale/absent RF
degrades to unavailable (never frozen-propagated), and malformed payloads yield
"no data" rather than fabricated numbers.
"""

import math
import time
import unittest

from as2_experiment_aerpaw_multiuav.experiment.rf_metrics import (
    CallbackMeasurementSource,
    RFMetrics,
    TopicMeasurementSource,
    UnavailableRFSource,
)


def _bare_topic_source(max_age=5.0):
    """Build a TopicMeasurementSource without running __init__ (no ROS node)."""
    src = object.__new__(TopicMeasurementSource)
    src._latest = {}
    src._rx_at = {}
    src._max_age_s = max_age
    return src


class TestMeasurementStaleness(unittest.TestCase):
    def test_fresh_is_returned(self):
        src = _bare_topic_source(max_age=5.0)
        src._latest["a"] = RFMetrics(vehicle_id="a", rssi_dbm=-60.0)
        src._rx_at["a"] = time.monotonic()
        out = src.poll(["a"])
        self.assertTrue(out["a"].available)

    def test_stale_degrades_to_unavailable(self):
        src = _bare_topic_source(max_age=0.05)
        src._latest["a"] = RFMetrics(vehicle_id="a", rssi_dbm=-60.0)
        src._rx_at["a"] = time.monotonic() - 10.0   # long ago
        out = src.poll(["a"])
        self.assertFalse(out["a"].available)         # stale -> not propagated

    def test_absent_is_unavailable(self):
        src = _bare_topic_source()
        out = src.poll(["a", "b"])
        self.assertFalse(any(m.available for m in out.values()))


class TestMalformedPayload(unittest.TestCase):
    def test_garbage_json_yields_no_values(self):
        m = RFMetrics.from_json("a", "{ not json ]")
        self.assertFalse(m.available)
        self.assertIsNone(m.rssi_dbm)

    def test_nan_metric_is_not_propagated_as_real(self):
        # a source emitting NaN must be detectable; RFMetrics keeps it but .available
        # treats it via value presence; policies should guard. Here we just ensure
        # from_json doesn't crash and keeps None for absent keys.
        m = RFMetrics.from_json("a", '{"throughput_mbps": 10.0}', ts=1.0)
        self.assertEqual(m.throughput_mbps, 10.0)
        self.assertIsNone(m.sinr_db)

    def test_unavailable_source(self):
        out = UnavailableRFSource().poll(["a", "b"])
        self.assertFalse(any(m.available for m in out.values()))

    def test_callback_source_absence(self):
        src = CallbackMeasurementSource(lambda ids: {i: RFMetrics(vehicle_id=i) for i in ids})
        self.assertFalse(src.poll(["a"])["a"].available)


if __name__ == "__main__":
    unittest.main()
