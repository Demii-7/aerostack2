"""Closed-loop harness tests (Stage 10). Pure stdlib: no ROS, no aerpawlib, no vehicle.

Proves the wireless->robot loop composes correctly: measurements influence commands,
unavailable measurements degrade to fallback (never fabricated), and no algorithm is
baked into the runner.
"""

import unittest

from as2_experiment_aerpaw_multiuav.experiment import (
    CallbackMeasurementSource,
    ClosedLoopRunner,
    RFMetrics,
    RobotCommand,
    StaticHoverPolicy,
)
from as2_experiment_aerpaw_multiuav.experiment.policies import SignalAwareWaypointPolicy


class MockPlatform:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.calls = []

    def arm(self): self.calls.append(("arm",))
    def offboard(self): self.calls.append(("offboard",))
    def takeoff(self, h, s): self.calls.append(("takeoff", h, s)); return True
    def land(self, s): self.calls.append(("land", s)); return True
    def go_to(self, x, y, z, speed, yaw=None): self.calls.append(("go_to", x, y, z, speed, yaw)); return True
    def follow_path(self, pts, speed): self.calls.append(("follow_path", pts, speed)); return True
    def hover(self): self.calls.append(("hover",))
    def get_state(self):
        from as2_experiment_aerpaw_multiuav.experiment import VehicleState
        return VehicleState(vehicle_id=self.vehicle_id, connected=True)
    def close(self): pass


def source(**by_id):
    def poll(ids):
        return {i: by_id.get(i, RFMetrics(vehicle_id=i)) for i in ids}
    return CallbackMeasurementSource(poll)


class TestClosedLoop(unittest.TestCase):
    def test_good_link_drives_to_far_waypoint(self):
        cands = [(0.0, 0.0, 25.0), (50.0, 50.0, 25.0)]
        pol = SignalAwareWaypointPolicy(cands, prefer="sinr_db", maximize=True)
        a = MockPlatform("drone0")
        runner = ClosedLoopRunner([a], source(
            drone0=RFMetrics(vehicle_id="drone0", sinr_db=20.0)), pol)
        cmds = runner.step()
        # high SINR -> far candidate (last)
        self.assertEqual(cmds["drone0"].kind, "go_to")
        self.assertEqual((cmds["drone0"].x, cmds["drone0"].y), (50.0, 50.0))
        self.assertIn(("go_to", 50.0, 50.0, 25.0, pol.speed, None), a.calls)

    def test_bad_link_drives_to_near_waypoint(self):
        cands = [(0.0, 0.0, 25.0), (50.0, 50.0, 25.0)]
        pol = SignalAwareWaypointPolicy(cands, prefer="sinr_db", maximize=True)
        a = MockPlatform("drone0")
        runner = ClosedLoopRunner([a], source(
            drone0=RFMetrics(vehicle_id="drone0", sinr_db=-5.0)), pol)
        cmds = runner.step()
        self.assertEqual((cmds["drone0"].x, cmds["drone0"].y), (0.0, 0.0))

    def test_unavailable_metrics_fall_back_no_fabrication(self):
        pol = SignalAwareWaypointPolicy([(1.0, 1.0, 25.0), (9.0, 9.0, 25.0)])
        a = MockPlatform("drone0")
        # empty RF (all None) -> fallback = first candidate; nothing invented
        runner = ClosedLoopRunner([a], source(drone0=RFMetrics(vehicle_id="drone0")), pol)
        cmds = runner.step()
        self.assertEqual((cmds["drone0"].x, cmds["drone0"].y), (1.0, 1.0))
        self.assertFalse(runner.last_metrics["drone0"].available)

    def test_hover_policy_and_multi_vehicle(self):
        a, b = MockPlatform("drone0"), MockPlatform("drone1")
        runner = ClosedLoopRunner([a, b], source(), StaticHoverPolicy())
        cmds = runner.step()
        self.assertEqual(set(cmds), {"drone0", "drone1"})
        self.assertEqual(a.calls[-1], ("hover",))
        self.assertEqual(b.calls[-1], ("hover",))

    def test_command_factory_kinds(self):
        self.assertEqual(RobotCommand.go_to(1, 2, 3).kind, "go_to")
        self.assertEqual(RobotCommand.hover().kind, "hover")
        self.assertEqual(RobotCommand.land().kind, "land")
        self.assertEqual(RobotCommand.takeoff(30.0).kind, "takeoff")
        self.assertEqual(RobotCommand.follow_path([(0, 0, 1)]).kind, "follow_path")

    def test_run_iterates_and_stops(self):
        a = MockPlatform("drone0")
        runner = ClosedLoopRunner([a], source(), StaticHoverPolicy())
        n = runner.run(duration_s=0.3, period_s=0.05, max_steps=3)
        self.assertGreaterEqual(n, 1)
        self.assertLessEqual(n, 3)
        self.assertEqual(n, len(a.calls))

    def test_online_reference_updates_each_step(self):
        """Stage 11: a policy that re-decides every step pushes updated references
        during flight (not a static preplanned trajectory)."""
        class MovingTargetPolicy:
            def __init__(self):
                self.i = 0

            def decide(self, states, metrics):
                self.i += 1
                return {vid: RobotCommand.go_to(float(self.i), 0.0, 25.0, speed=2.0)
                        for vid in states}

        a = MockPlatform("drone0")
        runner = ClosedLoopRunner([a], source(), MovingTargetPolicy())
        gotos = []
        for _ in range(4):
            cmds = runner.step()
            gotos.append(cmds["drone0"].x)
        # each step is a *new* reference -> strictly increasing x
        self.assertEqual(gotos, [1.0, 2.0, 3.0, 4.0])
        # the platform actually received the updated go_to references in order
        recv = [c[1] for c in a.calls if c[0] == "go_to"]
        self.assertEqual(recv, [1.0, 2.0, 3.0, 4.0])

    def test_rf_change_alters_next_reference(self):
        """Wireless measurement change -> different in-flight reference (closed loop)."""
        cands = [(0.0, 0.0, 25.0), (100.0, 0.0, 25.0)]
        pol = SignalAwareWaypointPolicy(cands, prefer="sinr_db", maximize=True)
        a = MockPlatform("drone0")
        # feed good SINR first step, then bad -> target flips far->near online
        seq = [
            RFMetrics(vehicle_id="drone0", sinr_db=20.0),
            RFMetrics(vehicle_id="drone0", sinr_db=-20.0),
        ]
        it = iter(seq)
        src = CallbackMeasurementSource(lambda ids: {"drone0": next(it)})
        runner = ClosedLoopRunner([a], src, pol)
        far = runner.step()["drone0"].x      # good link -> far (100)
        near = runner.step()["drone0"].x     # bad link -> near (0)
        self.assertEqual((far, near), (100.0, 0.0))


if __name__ == "__main__":
    unittest.main()
