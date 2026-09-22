"""Multi-UAV coordination + shared state tests (Stage 12). Pure stdlib, no ROS.

Proves: shared fleet state drives formation/collision-avoidance/coverage, and each
vehicle's command is computed from the whole fleet but emitted only for that vehicle
(no cross-control / no command bleed between namespaces).
"""

import unittest

from as2_experiment_aerpaw_multiuav.experiment import (
    ClosedLoopRunner,
    CollisionAvoidancePolicy,
    CoverageAssignmentPolicy,
    FleetState,
    FormationPolicy,
    RFMetrics,
    StaticHoverPolicy,
    VehicleState,
)
from test.test_closed_loop import MockPlatform, source


def vs(vid, x=0.0, y=0.0, z=25.0):
    return VehicleState(vehicle_id=vid, x=x, y=y, z=z, connected=True)


class TestFleetState(unittest.TestCase):
    def test_geometry_queries(self):
        f = FleetState.from_maps(
            {"a": vs("a", 0, 0), "b": vs("b", 3, 4), "c": vs("c", 6, 8)}, {})
        self.assertAlmostEqual(f.distance("a", "b"), 5.0)
        self.assertEqual(f.nearest("c")[0], "b")
        self.assertAlmostEqual(f.centroid_xy()[0], 3.0)
        self.assertAlmostEqual(f.centroid_xy()[1], 4.0)


class TestFormation(unittest.TestCase):
    def test_followers_hold_offsets_from_leader(self):
        pol = FormationPolicy(
            leader="drone0",
            offsets={"drone0": (0, 0), "drone1": (0, 10), "drone2": (0, -10)},
            leader_path=[(10.0, 0.0, 25.0)], speed=3.0)
        states = {"drone0": vs("drone0", 0, 0), "drone1": vs("drone1", 0, 0),
                  "drone2": vs("drone2", 0, 0)}
        cmds = pol.decide(states, {})
        # leader -> waypoint
        self.assertEqual((cmds["drone0"].x, cmds["drone0"].y), (10.0, 0.0))
        # followers -> leader CURRENT position (0,0) + NE offset (east, north)
        self.assertEqual((cmds["drone1"].x, cmds["drone1"].y), (10.0, 0.0))  # east +10
        self.assertEqual((cmds["drone2"].x, cmds["drone2"].y), (-10.0, 0.0))  # east -10


class TestCollisionAvoidance(unittest.TestCase):
    def test_too_close_pair_pushed_apart(self):
        base = StaticHoverPolicy()  # base commands all identical -> overlap
        pol = CollisionAvoidancePolicy(base, safe_distance=15.0, push=5.0)
        states = {"a": vs("a", 0, 0), "b": vs("b", 5, 0)}  # 5 m apart < safe
        # StaticHover returns kind 'hover' (no x/y), so wrap a position base instead:
        class GoToOrigin:
            def decide(self, s, m):
                from as2_experiment_aerpaw_multiuav.experiment import RobotCommand
                return {v: RobotCommand.go_to(0.0, 0.0, 25.0) for v in s}
        pol2 = CollisionAvoidancePolicy(GoToOrigin(), safe_distance=15.0, push=5.0)
        cmds = pol2.decide(states, {})
        # a(0,0) b(5,0): a pushed toward -x, b toward +x -> commanded targets diverge
        self.assertGreater(cmds["b"].x - cmds["a"].x, 1e-9)
        self.assertLess(cmds["a"].x, 0.0)
        self.assertGreater(cmds["b"].x, 0.0)

    def test_far_pair_untouched(self):
        from as2_experiment_aerpaw_multiuav.experiment import RobotCommand

        class GoToOrigin:
            def decide(self, s, m):
                return {v: RobotCommand.go_to(0.0, 0.0, 25.0) for v in s}
        pol = CollisionAvoidancePolicy(GoToOrigin(), safe_distance=10.0)
        states = {"a": vs("a", 0, 0), "b": vs("b", 100, 0)}  # far apart
        cmds = pol.decide(states, {})
        self.assertEqual(cmds["a"].x, 0.0)
        self.assertEqual(cmds["b"].x, 0.0)


class TestCoverage(unittest.TestCase):
    def test_distinct_waypoint_assignment(self):
        wps = [(0, 0, 25), (10, 0, 25), (20, 0, 25)]
        pol = CoverageAssignmentPolicy(wps)
        states = {"a": vs("a", 0, 0), "b": vs("b", 11, 0), "c": vs("c", 19, 0)}
        cmds = pol.decide(states, {})
        assigned = [(cmds[v].x) for v in ("a", "b", "c")]
        # each vehicle got a distinct, geographically sensible waypoint (no collision)
        self.assertEqual(sorted(assigned), [0.0, 10.0, 20.0])
        self.assertAlmostEqual(cmds["b"].x, 10.0)


class TestNoCrossControl(unittest.TestCase):
    def test_each_command_targets_only_its_own_vehicle(self):
        # shared state visible to policy, but each MockPlatform only records its own cmds
        pol = FormationPolicy(
            leader="drone0",
            offsets={"drone0": (0, 0), "drone1": (0, 10)},
            leader_path=[(5.0, 0.0, 25.0)])
        p0, p1 = MockPlatform("drone0"), MockPlatform("drone1")
        runner = ClosedLoopRunner([p0, p1], source(
            drone0=RFMetrics(vehicle_id="drone0"),
            drone1=RFMetrics(vehicle_id="drone1")), pol)
        # states come from platforms' get_state (0,0); drive one step
        runner.step()
        gotos0 = [c for c in p0.calls if c[0] == "go_to"]
        gotos1 = [c for c in p1.calls if c[0] == "go_to"]
        self.assertTrue(gotos0 and gotos1)
        # drone0 (leader) -> 5,0 ; drone1 (follower) -> leader(0,0)+east10 = 10,0
        self.assertEqual((gotos0[-1][1], gotos0[-1][2]), (5.0, 0.0))
        self.assertEqual((gotos1[-1][1], gotos1[-1][2]), (10.0, 0.0))


if __name__ == "__main__":
    unittest.main()
