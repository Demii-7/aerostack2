# AERPAW ↔ AeroStack2 — Current Architecture (developer doc)

> **Purpose**: record the *as-is* architecture before any further integration work, so we
> reuse rather than duplicate. AeroStack2 (AS2) provides the reusable robotics layer;
> AERPAW owns the experiment + testbed + RF. The single seam between them is the
> **AERPAW–AeroStack2 adapter** = `as2_aerial_platforms/as2_platform_aerpaw`.
>
> This doc is a snapshot from read-only inspection. Paths and `file:line` refs point at
> real code. It is **not** a how-to — see [`E2E_MANUAL.md`](E2E_MANUAL.md) and the two
> package READMEs for that.

## 1. Component inventory (what already exists)

### 1.1 AeroStack2 core — REUSE, do not reimplement

| Capability in target diagram | AS2 component | Notes |
|---|---|---|
| Vehicle / platform abstraction | `as2_core` `as2::AerialPlatform` (`as2_core/include/as2_core/aerial_platform.hpp:79`) | The adapter subclasses this. Virtuals used: `ownSendCommand`, `ownSetArmingState`, `ownSetOffboardControl`, `ownSetPlatformControlMode`, `ownTakeoff/Land/KillSwitch/StopPlatform`, `configureSensors`. |
| State estimation / localization | `as2_state_estimator` (`raw_odometry` plugin) | Consumes `sensor_measurements/*`, produces `self_localization/*`. |
| Motion control | `as2_motion_controller` (`pid_speed_controller`) | ENU-odom-frame commands. |
| Trajectory / planning / behaviors | `as2_motion_reference_handlers`, `as2_behaviors_motion` (go_to/takeoff/land), `as2_behaviors_platform` (arm/offboard), `as2_behaviors_trajectory_generation`, `as2_behavior_tree`, `as2_behaviors_path_planning`, `as2_behaviors_swarm_flocking` | Full stack reused verbatim by Config C. |
| Researcher programmatic entry | `as2_python_api` `DroneInterface` / `DroneInterfaceGPS` | What missions/`triangle_formation.py` drive. |
| Multi-UAV capabilities | per-namespace stacks + `as2_python_api` | Multi-drone = N namespaced single-drone stacks. |
| Message/contract types | `as2_msgs` (`ControlMode`, `PlatformInfo`, `TrajectorySetpoints`, …) | Extend, do not fork. |
| Other platform backends (reference) | `as2_platform_multirotor_simulator`, `as2_platform_gazebo` | Show the adapter pattern; the AERPAW node follows them. |
| Viz / UI / logging | `as2_visualization`, `as2_rviz_plugins`, `as2_alphanumeric_viewer`, `as2_keyboard_teleoperation`, `as2_cli` | Reuse. |

### 1.2 AERPAW adapter — `as2_aerial_platforms/as2_platform_aerpaw` (the bridge)

**Stage 3 made the adapter thin + modular.** Translation/isolation is split into
self-contained pieces (all `namespace aerpaw_platform`, none in AS2 core); the node
is now wiring only and holds no coordinate math and no robotics algorithm.

| Module (in `include/as2_platform_aerpaw/` + `src/`) | Stage-3 responsibility |
|---|---|
| `frame_conversions.hpp` (pure, `<cmath>` only) | **Coordinate-frame + unit translation**: ENU↔NED, ENU-yaw↔ArduPilot-heading, GPS→local-ENU, quat→yaw. ROS/socket-free, unit-tested. |
| `command_translation.{hpp,cpp}` | **Command translation** AS2→AERPAW: builds the runner JSON (`goto_ned`/`set_velocity`/arm/takeoff/land/kill/stop). |
| `state_translation.{hpp,cpp}` | **State translation** AERPAW→AS2: `TelemetryState` → `sensor_measurements/{odom,gps,imu}`. |
| `vehicle_identity.hpp` | **Vehicle + multi-UAV identification**: `drone{i}` ↔ ports `15760+i*2`. |
| `adapter_types.hpp` | **DT-vs-physical abstraction** (`PlatformBackend sitl/digital_twin/physical`) + `GeoHome`. |
| `ipc_bridge.{hpp,cpp}` | Transport + **connection/status**: typed inbound (`telemetry`/`status`/`error`), `vehicle_id`, link-freshness (`telemetryAgeSeconds`). |
| `aerpaw_platform.{hpp,cpp}` | Thin `as2::AerialPlatform` orchestrator: wires the above + **link watchdog** → `platform_info.connected`, logs AERPAW status/error, gates real-hardware kill-switch warning. |

- **`AerpawPlatform` overrides** delegate to the modules: `ownSendCommand`→`command::*`,
  `telemetryCallback`→`state::make*`, so the seam is one direction per file.
- **aerpawlib runner** — `aerpawlib_runner/aerpaw_as2_runner.py`: the *actual* DT /
  physical access point (aerpawlib reaches DT E-VMs `10.14.x.x:14550`, testbed
  `192.168.32.x:14550`); emits typed `telemetry`/`status`/`error` + `vehicle_id`, can
  autostart the AS2 subsystem (Stage 2).
- **Config**: `control_modes.yaml`, `platform_params.yaml` (`ipc_cmd/tel_port`,
  `takeoff_altitude`, `platform_backend`, `vehicle_id`, `link_timeout`).
- **Multi-UAV Mapping**: `launch/aerpaw_sitl.launch.py` + `as2_stack.launch.py`
  spawn N namespaced platforms, per-drone ports `+i*2`, per-drone `conn`/`vehicle_id`.
- **Physical-testbed helpers**: `deploy/cvm_setup.sh`, `deploy/qgc_tunnel.sh`,
  `deploy/run_aerpaw_experiment.sh` (Stage 2 AERPAW-first bring-up).
- **Tests**: `translation_gtest` (9: frames/command/state/identity/backend),
  `ipc_bridge_gtest` (2), `aerpaw_platform_gtest` (1). No vehicle / aerpawlib needed.

### 1.3 Experiment entry points — `as2_experiment_aerpaw_multiuav`

- `launch/sim_multirotor.launch.py` (Config A), `sim_gazebo.launch.py` (Config B),
  `aerpaw_digital_twin.launch.py` (Config C = `use_aerpaw:=true` DT or `false` SITL).
  Config C layers the full AS2 stack per drone (`aerpaw_digital_twin.launch.py:_as2_stack_nodes`)
  then runs the mission — **the same mission on all three backends is the proof the
  platform abstraction works.**
- `missions/triangle_formation.py`: 3 drones, threaded `DroneInterface` control.
- `config/three_drone_world.yaml`: DT/world layout.

## 2. Functional status (as determined in this workspace)

- **Compiles / built**: `install_jazzy/as2_platform_aerpaw/lib/as2_platform_aerpaw/as2_platform_aerpaw_node`
  is present; `install_jazzy/as2_experiment_aerpaw_multiuav` launch+missions installed.
  **Verified in this workspace (Stages 3–14)**: `as2_platform_aerpaw` rebuilds clean on the
  conda `jazzy` ROS env (`colcon build` OK) and its **34 functional gtests pass**
  (`translation_gtest` 28, `ipc_bridge_gtest` 3, `aerpaw_platform_gtest` 3). Plus **19 Python
  tests** in `as2_experiment_aerpaw_multiuav` (closed-loop, multi-UAV, layering incl.
  environment-agnostic + ROS-2-boundary guards).
- **Documented pass levels** (`docs/README.md §Status`): Level-1 build+unit tests on Humble;
  Config A flies; Config B on Gazebo Harmonic/Jazzy; Config C **SITL endpoints validated**.
- **Still unverifiable in *this* shell**: **`aerpawlib` is not importable** here, so the
  runner itself and a real DT/SITL end-to-end flight can't be executed. Runtime flight
  verification needs an AERPAW VM or local `aerpawlib`+SITL per
  [`E2E_MANUAL.md`](E2E_MANUAL.md) §2.2–2.3. (`colcon test` also reports pre-existing
  `ament_lint`/`uncrustify`/`flake8` failures across the package — cosmetic lint debt, not
  functional.)
- **Known limitations (already coded/documented)**: kill-switch = disarm+stop, not an
  irreversible motor cut (`AerpawPlatform::ownKillSwitch` + runner `kill`, which also logs
  a warning when `platform_backend=physical`); SPEED mode depends on aerpawlib
  `set_velocity` marked *NOT SUPPORTED* by the copter filter.
- **Telemetry gaps handled explicitly, never fabricated (Stage 5)**: body angular rates,
  linear acceleration, GPS covariance and battery charge-state are not provided by
  aerpawlib/ArduPilot → marked *unavailable* (ROS covariance `−1` / `UNKNOWN`) in
  `state_translation.cpp`, not faked. The prior fabricated IMU (`accel.z=9.81`,
  zeroed-rates-as-measured) was removed. AERPAW flight mode/armable/ekf_ready are available
  but AS2 `PlatformInfo` has no field for them → logged on change only.

**Bottom line**: the *robotics control loop* of the target diagram (AS2 stack ↔ adapter ↔
aerpawlib ↔ UAV, incl. multi-UAV) is implemented and builds. The *RF / measurement* half of
the loop is not.

## 3. Gaps vs the target architecture diagram

The diagram's robotics column (`Motion Control / Localization / Trajectory / Planning /
Behaviors / Multi-UAV`) is fully served by AS2 + the adapter. The following diagram stages
have **no corresponding component yet**:

1. **Wireless / RF Infrastructure → AERPAW Measurements → Researcher Algorithm.**
   ~~Nothing in AS2 subscribes AERPAW RF/link metrics.~~ **Partially closed (Stage 10):**
   the adapter now *transports* AERPAW measurements (`measurement` IPC type →
   `<ns>/aerpaw/measurements` `std_msgs/String`), and the experiment layer exposes
   `TopicMeasurementSource` + `ClosedLoopRunner` + an injected `RobotPolicy` (see §10).
   Remaining work: a production AERPAW RF reader (OEO/data-collector client) feeding the
   runner's `--measure-checkpoint`; today it polls aerpawlib `checkpoint_check_string`
   and omits anything unavailable (never fabricated).
2. **Experiment Framework (AERPAW lifecycle).** Missions use plain `DroneInterface`; AERPAW
  's own experiment orchestration (node/resource reservation, radio config, experiment start/
   stop via aerpawlib's `ExperimentRunner`) is not driven from AS2. The runner is a
   `BasicRunner` (single-vehicle flight bridge), not an experiment coordinator.
3. **Experiment Data Collection.** No time-synced recording that pairs RF measurements with
   `self_localization/*`/`platform/*` under the AERPAW experiment's clock/metadata.
4. **Physical testbed end-to-end.** Only `deploy/` helpers exist; not validated against the
   real testbed (expected — needs AERPAW access).

## 4. Smallest set of NEW components to close the gaps (no AS2 duplication)

Keep everything in §1.1–1.3 as-is. Add the minimum:

- **N1 — `as2_platform_aerpaw` runner: promote `BasicRunner` → experiment-aware runner.**
  Extend `aerpaw_as2_runner.py` (and the IPC schema) rather than new code paths: add
  experiment lifecycle hooks + a measurement producer that pushes AERPAW RF/link metrics
  down the existing telemetry socket as a separate JSON `type`. *Reuses `IpcBridge`.*
- **N2 — `AerpawPlatform`: republish RF measurements as ROS topics.** In `aerpaw_platform.cpp`
  add a small publisher that turns the new measurement datagrams into ROS topics (e.g.
  `<ns>/aerpaw/rf/*`). Minimal new msg type(s) added to `as2_msgs` (or a tiny `aerpaw_msgs`),
  never forking existing types.
- **N3 — Experiment orchestrator (new small package `as2_aerpaw_experiment`), only if N1's
  lifecycle hooks prove insufficient.** Wraps aerpawlib experiment setup + exposes
  reserve/start/stop + resource handles as a ROS node/service so an AS2 mission can drive
  the AERPAW experiment. *Prefer extending the runner (N1); create this only when the
  single-vehicle runner cannot host the multi-resource coordinator cleanly.*
- **N4 — Data recorder (new node, may live in N2/N3 package).** Time-syncs + bags
  `aerpaw/rf/*` with AS2 `self_localization/*`/`platform/*`. Reuse existing rosbag/`as2_cli`
  tooling rather than writing a new logger.
- **N5 — Mission + launch updates in `as2_experiment_aerpaw_multiuav`.** Add a Config-C
  "RF-conditioned" mission consuming N2/N4 to demonstrate the closed research loop. No new
  mission package.

**Explicitly do NOT build**: another platform/control/state/multi-uav mechanism (AS2 owns
these), a duplicate MAVLink bridge (the adapter+runner own this), or a new experiment
framework (AERPAW owns this — we only add a thin driver).

## 5. Data-flow (as-is), mapped to the diagram

```
AERPAW (owns testbed + RF + experiment)
  aerpawlib runner (aerpaw_as2_runner.py) ── MAVLink ──► UAV (DT/SITL/real E-VM)
        ▲   JSON/UDP cmd ◄── IpcBridge ──┐         telemetry (GPS/NED) ─┐
        │   [RF metrics: NOT YET fed]    │                             ▼
   AERPAW–AS2 Adapter (as2_platform_aerpaw)                    telemetryCallback → sensor_measurements/*
        AerpawPlatform: Command/State translation + platform abstraction   │
        │  AS2 pose/twist cmds (ENU odom)                                   ▼
        └──────────────────────►  AS2 stack (per drone{i} namespace)  state_estimator → self_localization/*
                                   controller + behaviors + python_api  ▲
                                   Researcher mission (triangle_formation.py)  │ cmds
   MISSING LOOP: AERPAW RF measurements ──(N1/N2)──► aerpaw/rf/* ──► researcher mission
```

## 6. Control-plane ownership (Stage 2 — AERPAW is the top-level experiment platform)

**Rule:** AeroStack2 is a *robotics subsystem required by the AERPAW experiment*.
AERPAW owns the experiment and the testbed; AS2 provides reusable robotics
capability. The control/supervision direction is always **AERPAW → AS2**, never
the reverse.

### 6.1 Responsibility matrix

| Responsibility | Owner | Where it lives |
|---|---|---|
| Experiment lifecycle (create/start/stop/teardown) | **AERPAW** | AERPAW experiment controller runs `aerpawlib --script aerpaw_as2_runner.py`; local bring-up: `as2_platform_aerpaw/deploy/run_aerpaw_experiment.sh` |
| Experiment configuration | **AERPAW** | aerpawlib CLI args (`--conn`, `--vehicle`, env) + `deploy/cvm_setup.sh` |
| Digital Twin execution | **AERPAW** | aerpawlib (`--no-aerpaw-environment` = off; DT virtual E-VM `10.14.x.x:14550`) |
| Physical testbed execution | **AERPAW** | aerpawlib vs real E-VM `192.168.32.x:14550` + `deploy/qgc_tunnel.sh` |
| Wireless / RF infrastructure | **AERPAW** | (measurement feedback into AS2 = gap N1/N2, §4) |
| UAV / testbed resource reservation | **AERPAW** | OEO console / vehicle profile → `conn_droneN` (aerpawlib side) |
| Experiment deployment | **AERPAW** | `deploy/cvm_setup.sh`, `run_aerpaw_experiment.sh` |
| Experiment data collection | **AERPAW** | (+ AS2 recorder N4, §4) |
| --- | | |
| State estimation / localization | **AS2** | `as2_state_estimator` (reused) |
| Motion control / trajectory / planning | **AS2** | `as2_motion_controller`, `as2_behaviors_*` (reused) |
| Behaviors / multi-UAV robotics capability | **AS2** | `as2_behaviors_*`, `as2_python_api` (reused) |
| Vehicle command/state translation (adapter) | **AS2** | `as2_platform_aerpaw` `AerpawPlatform` (the seam) |
| Researcher robotics algorithm (mission) | **AS2** | `as2_experiment_aerpaw_multiuav/missions/*` |

### 6.2 AS2 must NOT (and currently does not) do these

Starting/managing the AERPAW experiment, replacing AERPAW's experiment framework,
its Digital Twin, its wireless infrastructure, or its testbed management.

**Verified**: `grep -rniE "aerpawlib" as2_core as2_state_estimator as2_motion_controller
as2_behaviors as2_python_api as2_behavior_tree as2_msgs` returns nothing — no AS2
*library* package imports or drives aerpawlib. The only aerpawlib touchpoint is the
AERPAW-side runner + the adapter's localhost UDP socket.

### 6.3 How the subsystem is started (production path)

`deploy/run_aerpaw_experiment.sh` (run as the AERPAW experiment) does, in order:
1. reach/reserve the vehicle MAVLink endpoints (AERPAW),
2. `ros2 launch as2_platform_aerpaw as2_stack.launch.py num_drones:=N` — **AS2
   robotics subsystem only** (platform + estimator + controller + behaviors, **no
   aerpawlib**), as a child process,
3. launch the per-drone `aerpawlib` bridge runners (AERPAW↔AS2 adapter),
4. run the AS2 mission,
5. `trap` teardown → stops the AS2 subsystem when the experiment ends.

Single-drone alternative: the runner's `--as2-subsystem [--as2-subsystem-cmd …]`
flag makes the AERPAW-run runner spawn+teardown `as2_stack.launch.py` for its own
namespace (same AERPAW→AS2 direction).

### 6.4 Dev harness (non-production, inverted on purpose)

`aerpaw_platform.launch.py`, `aerpaw_sitl.launch.py` and Config-C
`aerpaw_digital_twin.launch.py` let a local `ros2 launch` spawn `aerpawlib` so
SITL/Config-A/B/C can be validated from a single command with no AERPAW access.
These are **developer harnesses** (each carries a Stage-2 banner); they are not the
ownership model for a real experiment.

### 6.5 Vehicle ↔ platform mapping (Stage 4 — one AERPAW UAV = one AS2 platform)

Each AERPAW UAV is represented by exactly one `as2::AerialPlatform` (the
`AerpawPlatform` adapter node); AS2's normal abstraction (`DroneInterface`,
behaviours) drives it — **no bespoke per-vehicle API**. Uniqueness + multi-UAV
coexistence are the responsibility of `vehicle_identity.hpp`:

- `VehicleIdentity{ ns=drone{i}, vehicle_id, aerpaw_conn, mavlink_sysid,
  cmd_port=15760+i*2, tel_port=15761+i*2, backend }` — one per AERPAW UAV.
- `build_vehicle_map(N, ids, conns, backend)` builds the fleet map; ids/namespace
  default to `drone{i}` and are overridable (`vehicle_ids:=` in launch, real AERPAW
  names).
- `validate_unique(map)` rejects any namespace/id/port collision.
- The AERPAW-side **hardware-unique id** (MAVLink system id) is reported by the
  runner in telemetry (`mavlink_sysid`) and adopted by the platform, which logs the
  `AERPAW vehicle -> AS2 platform` mapping once.

`as2_stack.launch.py` / `aerpaw_sitl.launch.py` / `run_aerpaw_experiment.sh` each
spawn N of these from the map, so N AERPAW UAVs are live simultaneously.
**Tested**: `VehicleMap.FleetOfThreeIsUnique` / `…Collision` (mapping) and
`AerpawPlatformGTest.MultiplePlatformsCoexist` (3 namespaced platform nodes,
distinct IPC sockets, in one process).

## 7. State / telemetry path (Stage 5 — expose real AERPAW state, never fabricate)

`AERPAW UAV → aerpawlib Drone (vehicle iface) → IpcBridge (UDP-JSON) →
state_translation → ROS 2 sensor_measurements/* → AS2 raw_odometry`.

The set of fields published is exactly what aerpawlib `Drone` actually exposes
(introspected, not assumed):

| AERPAW source (real) | AS2 output | status |
|---|---|---|
| `position` (lat/lon/alt) | `odom.pose.position` (ENU) + `gps` | available |
| `attitude` (NED FRD euler, rad) | `odom.pose.orientation` + `imu.orientation` (ENU/FLU quat) | available |
| `velocity` (NED) | `odom.twist.linear` (body frame, per AS2 Odometry contract) | available |
| `gps.fix_type` | `gps.status` (FIX / NO_FIX) | available |
| `battery.{voltage,current,level}` | `sensor_measurements/battery` (BatteryState, `percentage=level/100`) | available (NEW) |
| `armed/mode/armable/ekf_ready/connected` | `platform_info.connected` (watchdog) + flight-state log | available (mode/armable/ekf: log-only, no PlatformInfo field) |
| — (not provided) | `imu.angular_velocity`, `imu.linear_acceleration`, `odom.twist.angular`, gps covariance, battery charge-state | **UNAVAILABLE → covariance `−1` / `UNKNOWN`, value 0; never fabricated** |

Correctness fixes: removed the previous **fabricated** IMU (`accel.z=9.81`, zeroed
rates-as-measured) and the **identity** odom orientation (which silently discarded
AERPAW heading). Orientation is now a verified NED→ENU quaternion; a units bug
(roll/pitch in radians vs yaw in degrees on the wire) was unified by sending an
explicit `*_rad` attitude triple. `header.stamp` uses the AS2 clock; the AERPAW
sample time is carried as `ts`.

**Tested** (`translation_gtest`): `NedAttitudeToEnuQuat`,
`WorldVecToBodyPreservesMagnitude`, `OdometryOrientationFromAttitudeNotIdentity`,
`OdometryTwistIsBodyFrameAndRatesMarkedUnknown`,
`ImuOrientationPresentRatesAndAccelMarkedUnavailable`, `GpsCarriesRealFixQuality`,
`BatteryFromAerpaw`.

## 8. Command path (Stage 6 — AS2 → AERPAW, only what AERPAW can execute)

`AeroStack2 → ROS 2 (actuator_command/* + platform services) → AerpawPlatform
(ownSendCommand/ownSetArmingState/ownTakeoff/…) → command_translation (ENU→NED,
mode→cmd) → IpcBridge (UDP-JSON) → aerpaw_as2_runner (aerpawlib Drone) → UAV`.

`command_capability.hpp` (pure, unit-tested) is the single source of truth for what
the AERPAW side can ACTUALLY run — derived from aerpawlib's own declarations and the
E-VM command filter (both introspected, not guessed):

| AS2 mode / command | AERPAW primitive | SITL | DT/Physical |
|---|---|:--:|:--:|
| arm / takeoff / land / position / yaw-angle | `set_armed`/`takeoff`/`land`/`goto_coordinates`(DO_REPOSITION) | ✅ | ✅ |
| hover / stop | SITL velocity-0; testbed **reposition-to-current** (LOITER is filter-banned) | ✅ | ✅ |
| speed / velocity | `set_velocity` (aerpawlib `[NOT SUPPORTED]`) | ✅ | ❌ `UNSUPPORTED_ON_TESTBED` |
| trajectory / attitude / body-rates / yaw-rate | (none) | ❌ | ❌ `UNSUPPORTED_ALWAYS` |

**Explicit unsupported handling** (never silently dropped / sent to an incapable
vehicle): `ownSetPlatformControlMode` consults the matrix and **returns false** so the
AS2 set-platform-mode service reports failure upstream, and `report_unsupported()` logs
each distinct rejection once with reason + backend; `ownSendCommand` re-guards per cycle.
This is why hover/stop on the testbed use the supported reposition rather than the
banned `action.hold()` (which severs the link).

**Tested** (`translation_gtest`): `CommandCapability.*` (lifecycle/position,
velocity-testbed-only, offboard-unsupported-always, the mode matrix) and
`CommandTranslation.HoverAtRepositionForm`.

## 9. Experiment logic vs platform integration (Stage 9)

Two strictly separated layers joined by a single seam. Full contract:
[`LAYERING.md`](LAYERING.md).

| Layer | Package | Owns | Never contains |
|---|---|---|---|
| Platform integration | `as2_platform_aerpaw` | AERPAW↔AS2 comm, state/command translation, platform abstraction, vehicle-id *mechanism* | mission, RF metrics, decision/optimisation, coordination policy |
| Experiment logic | `as2_experiment_aerpaw_multiuav` (`experiment/`) | researcher algorithm, objectives, decision, **RF/network metrics**, multi-UAV **coordination policy** | adapter/MAVLink internals, UDP wire |

- The experiment layer talks to vehicles ONLY through `experiment/interfaces.PlatformInterface`
  (over `as2_python_api.DroneInterface`); RF/network data via `experiment/rf_metrics`
  (a `MeasurementSource`; AERPAW feed is future work → `UnavailableRFSource`, never
  fabricated).
- `missions/triangle_formation.py` is now a thin runner over `coordination` +
  `decision`; it does not mention the adapter at all.
- **Dependency direction enforced** one-way (experiment → AS2 → adapter) by
  `as2_experiment_aerpaw_multiuav/test/test_layering.py` (runs under `colcon test`):
  experiment code may not import `ipc_bridge`/`aerpaw_platform`/`aerpaw_as2_runner`/
  `aerpawlib`/`as2_platform_aerpaw` or open the UDP IPC ports; the adapter may not import
  the experiment package. (Adapter grep + this test both confirm no leakage.)

## 10. Wireless-driven closed loop (Stage 10)

The central research cycle, with robotics and wireless operating as ONE experiment:

```
UAV moves → wireless env changes → AERPAW measures RF/network
   → aerpaw_as2_runner --measure-checkpoint polls aerpawlib checkpoint_check_string
   → {"type":"measurement", metrics:{sinr_db,rssi_dbm,throughput_mbps,packet_loss,...}}  (UDP JSON)
   → IpcBridge (transport) → AerpawPlatform publishes <ns>/aerpaw/measurements (std_msgs/String)
   → experiment: TopicMeasurementSource → RobotPolicy.decide(states, metrics) → RobotCommand
   → PlatformInterface (AS2 behaviour/controller) → UAV moves → ...
```

**Division of responsibility (no algorithm in the integration layer):**

| Piece | Layer | Role |
|---|---|---|
| `measurement` IPC type + `<ns>/aerpaw/measurements` topic | Platform integration | **transport only** — forwards AERPAW metrics verbatim (opaque JSON), no interpretation |
| `rf_metrics.MeasurementSource` (Protocol) + `TopicMeasurementSource` / `CallbackMeasurementSource` / `UnavailableRFSource` | Experiment | the interface **different experiments implement/consume** to read AERPAW metrics |
| `rf_metrics.RFMetrics` (`sinr_db, rssi_dbm, throughput_mbps, packet_loss, channel, …`) | Experiment | wireless metric model; `None` = unavailable, never fabricated |
| `closed_loop.RobotPolicy` (Protocol) + `RobotCommand` | Experiment | injectable research algorithm → per-vehicle action |
| `closed_loop.ClosedLoopRunner` | Experiment | generic sense→decide→act cycle (holds no algorithm) |
| `policies.SignalAwareWaypointPolicy`, `StaticHoverPolicy` | Experiment | **examples** only; swap freely |

- Different experiments plug in by providing a `RobotPolicy` (and optionally a custom
  `MeasurementSource`); neither the adapter nor the runner changes.
- Honesty preserved: with no live RF feed the runner emits nothing and policies fall back
  to geometry (`test_closed_loop.test_unavailable_metrics_fall_back_no_fabrication`).
- Boundary kept: the closed loop lives entirely in the experiment package; it consumes
  measurements via the standard `std_msgs/String` topic, so the Stage-9 layering test
  still passes (no adapter/aerpawlib import).
- **Tested**: `test_closed_loop` (6: good/bad link, unavailable-fallback, multi-vehicle,
  command kinds, run-loop bounds) + `ipc_bridge.MeasurementIsTransportedVerbatim`.
- **Demo**: `missions/wireless_closed_loop.py`.

## 11. Online trajectory updates (Stage 11 — updated references during flight)

Requirement: experiments whose desired trajectory changes mid-flight (current state +
live wireless metric → new reference → AS2 → UAV); NOT restricted to preplanned static
trajectories. Two changes make the command path live-updatable while respecting AERPAW's
primitive (DO_REPOSITION = a *target*, not a fast setpoint; offboard velocity/trajectory
is SITL-only per §8):

- **Adapter `ownSendCommand` (POSITION):** a change-gate + keepalive. A new `goto` is
  forwarded only when the reference actually moves (`command::position_reference_changed`,
  pure+tested: > `position_update_min_distance` m or > `position_update_min_yaw` rad) or
  every `position_keepalive` s. Fresh in-flight references flow through immediately,
  without flooding DO_REPOSITION. A control-mode change forces the next send.
- **Runner `aerpaw_as2_runner.py`:** a non-blocking, **latest-wins** movement worker. The
  UDP recv loop no longer awaits arrival — `goto`/`set_velocity`/takeoff/land go to a
  single worker; `_pending_move` coalesces to the newest and preempts an in-flight leg —
  so new references, telemetry, measurements and kill/stop are always accepted en route.

Honest limit: aerpawlib gates each new `goto_coordinates` on the vehicle being "ready"
(leg arrival), so a retarget is applied at the earliest the autopilot accepts a new
reposition — updates are continuous/latest-wins, not instantaneous mid-leg snaps. True
high-rate trajectory streaming needs offboard (SITL). The §10 closed loop + this path
let a researcher re-issue waypoint/velocity references every cycle.

**Tested**: `CommandTranslation.PositionReferenceChangedGate` (adapter gate);
`test_closed_loop.test_online_reference_updates_each_step` +
`…test_rf_change_alters_next_reference` (updated references reach the platform).

## 12. Multi-UAV experiments (Stage 12)

Multiple AERPAW UAVs are first-class, independent entities — one adapter + one
`as2::AerialPlatform` each — joining into one shared ROS 2 / AS2 multi-UAV layer:

```
AERPAW ── UAV1 ── Adapter1 (ns drone0) ─┐
       ── UAV2 ── Adapter2 (ns drone1) ─┼─► ROS 2 (per-ns topics) ─► AS2 ─► coordination
       ── UAV3 ── Adapter3 (ns drone2) ─┘
```

**Isolation guarantees (verified on the live ROS graph by
`aerpaw_platform_gtest.MultiUavNamespaceIsolation`):**
- **Unique vehicle IDs** + namespace `drone{i}` (`vehicle_identity.hpp`, §6.5).
- **Independent state interfaces:** `/drone{i}/sensor_measurements/odom` is published
  *only* by the `drone{i}` platform node (asserted via publisher node-namespace).
- **Independent command interfaces:** `/drone{i}/actuator_command/pose` is subscribed
  *only* by `drone{i}` — a command for one vehicle can never be heard by another
  (**no cross-control**).
- **Disjoint topic namespaces** across vehicles; **independent IPC ports** (`15760+i*2`)
  and MAVLink conns per vehicle (physical isolation too).

**Shared multi-UAV state (where required):**
- AS2-native: coordination nodes subscribe to each `<ns>/self_localization/pose`
  (e.g. `as2_behaviors_swarm_flocking`) — reused, not reimplemented.
- Experiment-layer: `experiment/fleet.py.FleetState` aggregates every vehicle's
  `VehicleState` (+ RF metrics) into one snapshot for a `RobotPolicy`, so a single
  decision tick sees the whole fleet.

**Coordination substrate = existing AS2 + the §10/§11 closed loop:** `ClosedLoopRunner`
already passes the *full* states+metrics dict to `RobotPolicy.decide`, so any policy can
be fleet-wide. Example policies (`experiment/policies.py`):
`FormationPolicy` (leader-follower), `CollisionAvoidancePolicy` (reactive separation over
shared state), `CoverageAssignmentPolicy` (distributed task assignment). These are
illustrative — swap in cooperative planning / coverage / comm-aware coordination without
touching the adapter.

**Future experiment families enabled by construction:** formation control, cooperative
trajectory planning, collision avoidance, coverage, distributed planning, and
communication-aware coordination (FleetState + RF metrics feed the policy). Demo:
`missions/multi_uav_coordination.py`.

**Tested:** `MultiUavNamespaceIsolation` (adapter/ROS graph), `test_multiuav` (FleetState
geometry, formation offsets, collision separation, coverage assignment, no-cross-control).

## 13. Digital Twin ↔ Physical Testbed compatibility (Stage 13)

**AERPAW already provides the abstraction; we mirror it, not duplicate it.** Introspected
fact: aerpawlib has exactly ONE environment distinction — whether the AERPAW OEO forward
server is reachable (`AerpawPlatform` / `ping_forward_server`, surfaced as
`--no-aerpaw-environment`). It does **not** distinguish Digital Twin from physical anywhere
in the `Drone` API; the only difference is which E-VM address `--conn` points at
(`10.14.x.x` vs `192.168.32.x`), invisible to the robotics integration.

```
Experiment -> AeroStack2 robotics -> AERPAW adapter -> { aerpawlib Drone (identical API) }
                                                        ├─ Digital Twin   (AERPAW env)
                                                        └─ Physical testbed (AERPAW env)
```

So environment-specific differences are isolated **below the common interface**: the
experiment, the AS2 robotics stack, and the adapter are byte-identical for DT and physical;
only `CONNS` (endpoints) + a safety label differ.

Implementation (adapter_types.hpp / command_capability.hpp):
- `PlatformBackend{SITL, DIGITAL_TWIN, PHYSICAL}` is a **label that mirrors AERPAW's env
  notion** — not a second execution path. `is_aerpaw_environment(b)` (DT or physical) is
  the single capability gate (velocity/offboard blocked in the AERPAW env; works in SITL),
  matching AERPAW's own boundary.
- **DT ≡ physical for capability** — `for_control_mode(DT,m,y) == for_control_mode(PHYSICAL,m,y)`
  for every mode (asserted). The only DT-vs-physical difference is `is_real_hardware`
  (physical adds a kill-switch safety **warning**) — no command changes.
- Switching env is pure config: `as2_stack.launch.py platform_backend:=digital_twin|physical`
  (default derived from `use_aerpaw`); `run_aerpaw_experiment.sh AERPAW_BACKEND=physical
  CONNS="<192.168.32.x endpoints>"`. Same experiment, same code.

Guaranteed environment-agnostic experiment: `test_layering.test_experiment_is_environment_agnostic`
asserts the experiment layer/missions contain no branch on backend/environment identifiers.

**Tested**: `CommandCapability.DigitalTwinEqualsPhysicalTestbed` (adapter parity),
`test_experiment_is_environment_agnostic` (experiment layer).

## 14. ROS 2 kept internal to the robotics layer (Stage 14)

Middleware topology and the intended boundary:

```
AERPAW  <->  Integration Adapter (as2_platform_aerpaw)  <->  ROS 2  <->  AeroStack2
   (no ROS)      (the single ROS<->AERPAW seam)          (AS2 native)
```

**Where ROS 2 lives (correct):** the AeroStack2 components (platform, estimator,
controller, behaviours) and the adapter's ROS-facing side (publishers/subscriptions,
`sensor_measurements/*`, `actuator_command/*`, the `<ns>/aerpaw/measurements` passthrough).
The adapter IS the boundary — that is appropriate.

**Where ROS 2 is kept OUT (enforced):**
- **AERPAW side (`aerpaw_as2_runner.py`)** — the only coupling to the robotics layer is the
  **ROS-free UDP/JSON integration interface** (the clean interface). Its optional subsystem
  bring-up is an *opaque* `--as2-subsystem-cmd` the runner starts but never interprets; the
  previous `ros2 launch as2_platform_aerpaw …` literal was removed. Bring-up normally lives
  in the AERPAW orchestrator (a launcher), not the bridge.
- **Pure experiment core** (`closed_loop`, `fleet`, `coordination`, `decision`, `policies`) —
  stdlib only, no `rclpy`/ROS-msg imports, so policies are portable/testable without ROS.
  ROS appears only in the two **boundary implementations** (`interfaces.DroneInterfacePlatform`
  over `as2_python_api`, `rf_metrics.TopicMeasurementSource`), and only as *lazy,
  function-scoped* imports behind the clean `PlatformInterface` / `MeasurementSource`
  interfaces.

**Clean integration interfaces** (what the experiment / AERPAW side depend on, not ROS):
`PlatformInterface` (vehicle ops), `MeasurementSource`/`RFMetrics` (wireless data),
`RobotPolicy`/`RobotCommand` (decisions), and the UDP/JSON contract on the AERPAW side.

**Tested (CI-enforced):** `test_layering.test_aerpaw_side_runner_is_ros_free` (runner has
no `ros2`/`rclpy`/`as2_*`/msg references in code) and
`test_layering.test_experiment_core_is_ros_free` (pure core has none; boundary impls import
ROS lazily, never at module top level).
