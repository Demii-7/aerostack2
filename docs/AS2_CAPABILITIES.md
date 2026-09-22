# Reusing AeroStack2 Robotics Capabilities on AERPAW (Stage 8)

The whole point of the integration is to **reuse AeroStack2, not reimplement it**.
An AERPAW UAV is surfaced as a normal `as2::AerialPlatform`, so existing AS2
controllers, behaviors and planners drive it through the standard abstraction. The
researcher writes an ordinary AS2 mission; it runs on AERPAW unchanged.

## Desired flow (exactly how it is wired)

```
Researcher            reuse_capabilities.py / triangle_formation.py   (as2_python_api)
   ↓
AERPAW experiment     aerpawlib runner (owns lifecycle/resources/RF)
   ↓
AS2 behavior/planner  go_to · follow_path · follow_reference · takeoff · land · planner · swarm
   ↓
AS2 motion controller pid_speed_controller
   ↓
AS2 platform API      as2::AerialPlatform (self_localization/*, actuator_command/*, set_control_mode)
   ↓
AERPAW adapter        as2_platform_aerpaw (command/state translation)
   ↓
AERPAW UAV            aerpawlib → MAVLink
```

The subsystem launch (`as2_platform_aerpaw/launch/as2_stack.launch.py`) wires the
**standard, unmodified AS2 nodes** (state estimator, motion controller, behaviors)
per drone namespace. Config-C (`as2_experiment_aerpaw_multiuav`) now **includes that
same launch** rather than duplicating it.

## What the AERPAW platform exposes to AS2

- Control modes (`config/control_modes.yaml`): **POSITION** (yaw ANGLE), **HOVER**,
  and **SPEED** (yaw SPEED; executed only in SITL — the testbed filter blocks
  velocity/offboard, see `command_capability.hpp` / `docs/ARCHITECTURE.md` §8).
- State: `sensor_measurements/{odom,gps,imu,battery}` → estimator →
  `self_localization/{pose,twist,...}` (`raw_odometry`).
- Services: arm / offboard / set-control-mode / takeoff / land (base platform).

## Capability reuse matrix

AS2 behaviors reach the platform via the controller. `pid_speed_controller` emits
**POSITION/HOVER** setpoints to the platform, so the behaviors below need **no**
offboard/TRAJECTORY support and run on AERPAW unchanged.

| AS2 capability | Plugin(s) that work on AERPAW | Runs unchanged? | Notes |
|---|---|:--:|---|
| Takeoff behavior | `takeoff_plugin_platform` / `takeoff_plugin_position` | ✅ | |
| Land behavior | `land_plugin_platform` / `land_plugin_position` | ✅ | |
| Navigation (`go_to`) | `go_to_plugin_position` | ✅ | |
| Path following | `follow_path_plugin_position` | ✅ | `mission/reuse_capabilities.py` |
| Follow reference (position) | `follow_reference_plugin_position` | ✅ | leader-follower / teleop |
| Hover / stop | controller HOVER | ✅ | testbed via reposition (no LOITER) |
| Yaw / orientation | `go_to_with_yaw`, `follow_path_with_yaw` | ✅ | via goto `target_heading` |
| Path planning (`a_star`, `voronoi`) | planner → waypoint list → `go_to` | ✅ | planner outputs poses; execution is position-based |
| Multi-UAV swarm flocking | `as2_behaviors_swarm_flocking` | ✅ | consumes `self_localization` of each drone, emits velocity/pose refs → pid_speed → platform |
| Trajectory **following** via controller | needs platform **TRAJECTORY** mode | ❌ on DT/testbed, ✅ in SITL | see incompatibility note |
| `*_plugin_trajectory` (go_to/follow_path/follow_reference/takeoff/land) | uses differential-flatness/trajectory refs | ❌ on DT/testbed | use the `_position` variant |
| Trajectory generation (`generate_polynomial_trajectory`) | publishes trajectory refs | ⚠️ | works for planning/visualisation; executing on AERPAW must fall back to position sampling |
| Attitude / body-rate control | platform `ATTITUDE`/`BODY_RATES` | ❌ | AERPAW has no offboard rate/attitude path |

## The one genuine platform incompatibility

AERPAW (via the E-VM command filter) executes **position/goto, takeoff, land,
arm** but **not offboard velocity/trajectory/attitude/rate** commands (see
`docs/ARCHITECTURE.md` §8 and the `set_velocity` analysis). Therefore:

- Any AS2 behavior configured with the **`_position`** plugin and the
  **`pid_speed_controller`** works on AERPAW with **zero rewriting** — the
  controller closes the loop internally and hands the platform POSITION/HOVER
  setpoints, which AERPAW executes.
- The **`_trajectory`** plugins / `differential_flatness_controller` require the
  platform to accept **TRAJECTORY** commands. That is the *single* case where an
  existing AS2 capability cannot run on AERPAW as-is. **No new behavior is needed**:
  select the `_position` plugin instead (they are the same behaviors, different
  plugin), which is exactly what `as2_stack.launch.py` does.

So the rule holds: the researcher does **not** rewrite an AS2 behavior for AERPAW;
they pick the position-mode plugin already shipped by AS2. If/when AERPAW supports
offboard (filter change), the trajectory plugins become usable by flipping the
platform `control_modes.yaml` + the capability entry — no behavior change.

## Not reimplemented (deliberate)

This integration adds **no** control law, planner, trajectory generator, or
behavior. Those all live in AeroStack2 and are consumed as-is. The adapter contains
only command/state **translation** (`as2_platform_aerpaw`), consistent with the
architecture rule that AERPAW owns the testbed/experiment and AS2 owns the robotics.
