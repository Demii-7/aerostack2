# as2_platform_aerpaw

Aerial platform bridging [AeroStack2](https://github.com/AERPAW/aerostack2) to ArduPilot
vehicles via [aerpawlib](https://github.com/AERPAW/aerpawlib). A C++
`as2::AerialPlatform` node talks to a Python **aerpawlib runner** over localhost UDP — one
JSON object per datagram — so the same AS2 mission runs on the **Real AERPAW Digital
Twin**, **SITL**, or (with the same node) the standard AS2 simulators.

> Install / build / unit‑test: [`docs/E2E_MANUAL.md`](../../docs/E2E_MANUAL.md) §3.
> Run the example mission (Config A/B/C): [`as2_experiment_aerpaw_multiuav`](../../as2_experiment_aerpaw_multiuav/README.md).
> Requirements: [`docs/run_and_test_requirements.txt`](../../docs/run_and_test_requirements.txt).
> This page is the **bridge reference**.

## Ownership (AERPAW is top-level)

**AERPAW owns the experiment** (lifecycle, config, Digital Twin, physical testbed,
RF, resources, deployment, data collection). **AeroStack2 is a robotics subsystem**
that AERPAW starts. The supervision direction is AERPAW → AS2; AS2 never launches
or manages AERPAW. Production bring-up:
[`deploy/run_aerpaw_experiment.sh`](deploy/run_aerpaw_experiment.sh) (starts the AS2
subsystem + the aerpawlib runners + the mission, and tears AS2 down on exit). The
`ros2 launch` files below are **developer/SITL harnesses** (they launch aerpawlib
from AS2 for local testing without AERPAW access). See
[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) §6.

## AS2 robotics subsystem (started by AERPAW)

```bash
ros2 launch as2_platform_aerpaw as2_stack.launch.py num_drones:=3 use_aerpaw:=true
```

Starts ONLY the AS2 side per drone (platform adapter + state estimator + motion
controller + behaviors) — **no aerpawlib**. AERPAW is expected to run the
`aerpaw_as2_runner.py` for each drone and provide its MAVLink `--conn`.

## Bring up one drone (dev smoke test)

```bash
ros2 launch as2_platform_aerpaw aerpaw_platform.launch.py \
  namespace:=drone0 conn:=udpin://127.0.0.1:14550 use_aerpaw:=false

ros2 topic echo /drone0/self_localization/pose     # should stream once telemetry arrives
```

N drones (dev harness): `aerpaw_sitl.launch.py num_drones:=3` (adds the runner + per‑drone ports).

## IPC contract

| Item | Value |
|------|-------|
| Transport | localhost UDP, connectionless, **one JSON object per datagram** |
| Command port | `ipc_cmd_port` (15760) — the **runner binds it**, C++ sends |
| Telemetry port | `ipc_tel_port` (15761) — **C++ binds it**, runner sends |
| Per drone | offset `+i*2` → drone0 15760/15761, drone1 15762/15763, … |

## Vehicle ↔ platform mapping (one AERPAW UAV = one AS2 platform)

Every AERPAW UAV in the experiment is represented by exactly one
`as2::AerialPlatform` instance (no per-vehicle custom API — AS2's normal platform
abstraction + `DroneInterface` drive it). `vehicle_identity.hpp` is the single
source of truth for the fleet mapping:

```
AERPAW UAV i  ──  aerpaw_conn (MAVLink)  ──  runner (vehicle_id, mavlink_sysid)
      │                                              │ UDP IPC (cmd_port/tel_port)
      ▼                                              ▼
AS2 platform  ns=drone{i}   id=vehicle_id (or drone{i})   sysid (adopted from telemetry)
```

| Field | Meaning | Unique by |
|-------|---------|-----------|
| `ns` / ROS node | the AS2 platform identity | `drone{i}` |
| `vehicle_id` | AERPAW-facing name (OEO node/alias); optional via `vehicle_ids:=` | default `drone{i}`, or explicit |
| `mavlink_sysid` | AERPAW hardware id, adopted from telemetry | the AERPAW side |
| `cmd_port` / `tel_port` | IPC socket pair reaching exactly this platform | `+i*2` |

`build_vehicle_map(N, ids, conns, backend)` produces the map and
`validate_unique(map)` fails on any namespace/id/port collision, so a multi-UAV
fleet is provably unambiguous. The node logs the established mapping once:
`Mapped AERPAW vehicle id=... sysid=... -> AS2 platform ns=...`.

Bring-up (AERPAW owns lifecycle, starts this subsystem):

```bash
ros2 launch as2_platform_aerpaw as2_stack.launch.py \
  num_drones:=3 use_aerpaw:=true vehicle_ids:="uav-north uav-east uav-south"
```

The example mission reaches each vehicle through the standard API
(`DroneInterface("drone0" | "drone1" | "drone2")`), never a bespoke one.

**Multi-UAV isolation (Stage 12):** each vehicle's state feed and command input are
exclusively its own — `/drone{i}/sensor_measurements/odom` is published only by
`drone{i}`, and `/drone{i}/actuator_command/pose` is subscribed only by `drone{i}`, so
commands never cross vehicles (verified by `aerpaw_platform_gtest.MultiUavNamespaceIsolation`).
Shared fleet state for coordination (formation / collision avoidance / coverage / swarm)
lives in the experiment layer (`as2_experiment_aerpaw_multiuav/experiment/fleet.py`) or in
AS2's own `as2_behaviors_swarm_flocking`; the adapter stays per-vehicle. See
[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) §12 and
[`docs/AS2_CAPABILITIES.md`](../../docs/AS2_CAPABILITIES.md).

### Runner command map (`cmd` → aerpawlib)

| Command | Action |
|---------|--------|
| `arm` / `disarm` | `drone.set_armed(True/False)` |
| `takeoff` / `land` | `drone.takeoff(h)` / `drone.land()` |
| `set_velocity` | `drone.set_velocity(VectorNED(vx, vy, vz))` — **SITL only** (testbed blocks it) |
| `goto_ned` | `drone.goto_coordinates(home + VectorNED(n,e,d), target_heading=…)` |
| `kill` | `set_armed(False)` + `stop_velocity()` (see *Limitations*) |

## Command path (Stage 6 — AS2 → AERPAW)

`AeroStack2 (behaviours/controller) → ROS 2 actuator_command/* + platform services
→ AerpawPlatform (ownSendCommand / ownSetArmingState / ownTakeoff / …) →
command_translation (ENU→NED, mode→cmd) → IpcBridge (UDP-JSON) →
aerpaw_as2_runner (aerpawlib Drone) → UAV`.

The adapter only sends commands **AERPAW can actually execute**. The capability
matrix is a single source of truth in `command_capability.hpp` (pure + tested):

| AS2 command / control mode | AERPAW execution | SITL | Digital Twin / Physical |
|----------------------------|------------------|:----:|:-----------------------:|
| Arm / disarm | `set_armed` | ✅ | ✅ |
| Takeoff / Land | `takeoff` / `land` | ✅ | ✅ |
| Position (`POSITION`) | `goto_coordinates` (DO_REPOSITION) | ✅ | ✅ |
| Yaw / orientation (`YAW_ANGLE`) | goto `target_heading` | ✅ | ✅ |
| Hover (`HOVER`) / Stop | SITL: velocity-0; testbed: reposition-to-current | ✅ | ✅* |
| Velocity (`SPEED`, `SPEED_IN_A_PLANE`) | `set_velocity` | ✅ | ❌ filter-blocked |
| Trajectory (`TRAJECTORY`) | none | ❌ | ❌ |
| Attitude / body-rates | none | ❌ | ❌ |
| Yaw **rate** (`YAW_SPEED`) | none (offboard) | ❌ | ❌ |

\* on the testbed, LOITER/`action.hold` is **filter-banned** (severs the link), so
HOVER/STOP are implemented as a throttled **reposition-to-current** using the
supported `goto_coordinates`, not a banned primitive.

**Unsupported commands are handled explicitly, never silently dropped:**
`ownSetPlatformControlMode` checks the matrix and **returns false** (so the AS2
set-platform-mode service reports failure to the behaviour), and
`report_unsupported()` logs each distinct rejection once with the reason + backend.
`ownSendCommand` re-guards per cycle and never emits a `set_velocity`/trajectory the
vehicle cannot run.

### Online reference updates (Stage 11)

The POSITION path supports **changing the trajectory during flight**, not just static
preplanned paths:

- `ownSendCommand` forwards a `goto` only when the commanded reference **changes**
  (`command::position_reference_changed`, > `position_update_min_distance` m /
  `position_update_min_yaw` rad) or every `position_keepalive` s — so new waypoints flow
  through immediately without flooding DO_REPOSITION (a target, not a fast setpoint).
- The runner (`aerpaw_as2_runner.py`) executes movement in a **non-blocking latest-wins
  worker**, so its command loop never stalls on arrival — a fresh reference (or
  kill/stop/measurement) is accepted mid-leg.
- Params: `position_update_min_distance`, `position_update_min_yaw`, `position_keepalive`.
- True high-rate velocity/trajectory streaming needs offboard (SITL only — the testbed
  filter blocks it, §Command path). Combined with the Stage-10 closed loop, a researcher
  re-issues references every cycle.

## Digital Twin ↔ Physical Testbed (same code)

AERPAW's only environment distinction (per aerpawlib) is *AERPAW env vs not* (OEO forward
server reachable; CLI `--no-aerpaw-environment`). **Digital Twin and physical are the same
AERPAW environment** — the `Drone` API, the runner, the adapter and AS2 are identical for
them; the only difference is which E-VM `--conn` points at (`10.14.x.x` vs `192.168.32.x`).

The adapter mirrors that (does not invent a second abstraction): `PlatformBackend` is just
a label; capabilities gate on `is_aerpaw_environment()` so **DT ≡ physical** (tested), and
`is_real_hardware()` only adds a kill-safety warning on physical. Switch environments with
pure config — `platform_backend:=physical` (launch) / `AERPAW_BACKEND=physical` +
`CONNS=...` (`run_aerpaw_experiment.sh`); no code change. See
[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) §13.

### Inbound message types (runner → platform, on the telemetry port)

One JSON datagram, discriminated by `type` (default `telemetry` → backward
compatible). Every message carries `vehicle_id` + `ts`.

| `type` | Fields | Adapter handling |
|--------|--------|------------------|
| `telemetry` | position/GPS, NED velocity, full attitude (rad), gps fix/sats, battery, armed/mode/armable/ekf, connected, vehicle_id, mavlink_sysid, ts | → `sensor_measurements/*`; feeds link watchdog + flight-state log |
| `status` | `message` | logged INFO (lifecycle/report) |
| `error` | `command`, `message` | logged ERROR (failed cmd) |
| `measurement` | `vehicle_id`, `metrics` (RF/wireless JSON), `ts` | **transport only**: republished verbatim to `<ns>/aerpaw/measurements` (`std_msgs/String`); NOT interpreted here |

> The `measurement` type is the Stage-10 wireless→robot loop plumbing. The adapter is a
> dumb pipe: it forwards the AERPAW RF/wireless payload onto a standard ROS topic. The
> *interpretation* and any decision algorithm live in the experiment layer
> (`as2_experiment_aerpaw_multiuav/experiment/` — `TopicMeasurementSource`,
> `RobotPolicy`, `ClosedLoopRunner`). No research algorithm is hard-coded in the adapter.
> The runner emits these when started with `--measure-checkpoint "sinr_db=radio/snr,..."`
> (polls aerpawlib `checkpoint_check_string`; unavailable metrics are omitted, never faked).

## Frames

Frame + unit math is isolated in `frame_conversions.hpp` (pure, unit-tested).

- AS2 odom is **ENU** (x=E, y=N, z=U); the base class delivers commands already expressed
  in the odom frame, so no extra yaw rotation is applied.
- ENU→NED is a pure axis permutation: `north=+y`, `east=+x`, `down=-z`.
- ArduPilot heading (deg from North, clockwise): `heading = fmod(90 - yaw_enu, 360)`.
- Attitude: AERPAW gives MAVSDK **NED/FRD** euler (radians) → AS2 **ENU/FLU** quaternion
  (`ned_euler_to_enu_quat`: `roll=roll`, `pitch=-pitch`, `yaw=90°-yaw_ned`).
- `POSITION` targets are absolute offsets from the drone's **fixed home**
  (`home_lat`/`home_lon`, captured on first telemetry) → stable, non‑drifting waypoints.

## Telemetry / state path (Stage 5 — expose what AERPAW actually provides)

Flow: AERPAW UAV → `aerpaw_as2_runner.py` (aerpawlib `Drone` = vehicle interface)
→ `IpcBridge` (UDP-JSON) → `AerpawPlatform` (`state_translation`) → ROS 2
`sensor_measurements/*` → AS2 `raw_odometry` estimator.

| Topic | Type | From (aerpawlib) | Availability |
|-------|------|------------------|--------------|
| `sensor_measurements/odom` | `nav_msgs/Odometry` | position→ENU, attitude→orientation, velocity→**body** twist | pose+linear real; **angular velocity unavailable** (cov `−1`) |
| `sensor_measurements/imu` | `sensor_msgs/Imu` | attitude→`orientation` | **gyro+accel unavailable**: value 0, covariance `−1` (ROS "not provided") — never faked |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | lat/lon/alt + `gps.fix_type`→`status` | `position_covariance_type = UNKNOWN` |
| `sensor_measurements/battery` | `sensor_msgs/BatteryState` | `battery.{voltage,current,level}` | `level/100 → percentage`; status/health `UNKNOWN` (not given) |
| `platform/info` (base) | `as2_msgs/PlatformInfo` | armed/offboard/state-machine | `connected` from the link watchdog (telemetry freshness) |

**Unavailable → explicit, never fabricated:** body rates, linear acceleration, GPS
covariance, battery charge-state. AERPAW also gives **flight mode / armable /
ekf_ready**, but AS2 `PlatformInfo` has no field for them, so they are **logged on
change** (`AERPAW flight state: mode=… armed=… armable=… ekf_ready=…`) rather than
invented into a message that cannot carry them. The estimator falls back to what it
supports (`raw_odometry` uses odom pose+twist; the `-1` covariances mark IMU rates/
accel as unknown, not zero-with-confidence).

**Timestamp:** `header.stamp` = AS2 node clock (consistent for estimator/TF); the raw
AERPAW sample time is carried as `ts` in the IPC message for cross-reference.

## Control modes (`config/control_modes.yaml`)

| Mode | Value | Used by |
|------|-------|---------|
| HOVER | `0b00010000` | idle / safety |
| SPEED (yaw speed, local FLU) | `0b01000100` | `pid_speed` velocity |
| POSITION (yaw angle, ENU) | `0b01100000` | `go_to` |

## Fleet configuration (`config/fleet.yaml`) — Stage 18

Change the **number of UAVs** and all platform config in one data file — no source
edits. `launch/fleet_config.py::load_fleet` resolves it (and rejects duplicate
namespace/port/id).

```yaml
backend: digital_twin      # sitl | digital_twin | physical
namespace_prefix: drone
cmd_port_base: 15760
tel_port_base: 15761
port_stride: 2
frames: {earth: earth, map: map, odom: odom, base: base_link}
uav_defaults: {takeoff_altitude: 25.0, link_timeout: 1.0, position_keepalive: 0.5}
vehicles:                  # length == number of UAVs
  - id: uav-north
  - id: uav-east
  - id: uav-south          # per-vehicle: conn, namespace, cmd_port, backend, params...
```

Use it:

```bash
ros2 launch as2_platform_aerpaw as2_stack.launch.py fleet_config:=config/fleet.yaml
# ad-hoc overrides (precedence: arg > use_aerpaw > fleet.yaml):
ros2 launch as2_platform_aerpaw as2_stack.launch.py num_drones:=2 platform_backend:=physical
FLEET_CONFIG=$PWD/config/fleet.yaml ./deploy/run_aerpaw_experiment.sh   # AERPAW orchestrator
```

Per-node params live in `config/platform_params.yaml`; frames are also `as2::Node`
params (`earth_frame_id`, `odom_frame_id`, `base_frame_id`, …).

## Configuration (`config/platform_params.yaml`)

```yaml
cmd_freq: 100.0          # command loop rate (Hz)
info_freq: 10.0          # platform-info publish rate (Hz)
ipc_cmd_port: 15760      # UDP command port
ipc_tel_port: 15761      # UDP telemetry port
takeoff_altitude: 25.0   # AERPAW minimum (≥ 25 m)
```

## Launch arguments

`aerpaw_platform.launch.py` (single drone):

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `drone0` | ROS 2 drone namespace |
| `conn` | `udpin://127.0.0.1:14550` | MAVLink connection string |
| `cmd_port` / `tel_port` | `15760` / `15761` | IPC UDP ports |
| `use_sim_time` | `false` | Use simulation clock |
| `use_aerpaw` | `false` | `true` inside an AERPAW E‑VM |

`aerpaw_sitl.launch.py` (N drones): adds `num_drones` (default `3`) and `conn_drone<i>`
(per‑drone connection; ports auto‑increment by `i*2`).

`as2_stack.launch.py` (N drones, **AS2 subsystem only — production path started by
AERPAW**): `num_drones` (default `1`), `use_sim_time`. Launches no aerpawlib.

The AERPAW-run `aerpaw_as2_runner.py` accepts `--as2-subsystem` (+ optional
`--as2-subsystem-cmd`) so a single runner can itself start/stop
`as2_stack.launch.py` as its AERPAW-owned child (AERPAW→AS2).

## Unit tests

```bash
colcon test --packages-select as2_platform_aerpaw && colcon test-result --verbose
```

`ipc_bridge_gtest` (UDP round‑trip) + `aerpaw_platform_gtest` (node constructs; 3
platforms coexist) + `translation_gtest` (frames/attitude, command, state incl.
IMU/GPS/battery availability + fleet map). No vehicle or aerpawlib needed.

## Adapter modules (integration + translation only)

The adapter is deliberately **thin and modular**: the node wires the AS2 lifecycle
to the transport + isolated translation helpers, and holds **no robotics
algorithm** (control law, planning, estimation live in AeroStack2, not here).

| Stage‑3 responsibility | Module |
|------------------------|--------|
| Vehicle identification + multi-UAV mapping | `vehicle_identity.hpp` (+ `vehicle_id`/`platform_backend` params) |
| Coordinate-frame translation + unit conversion | `frame_conversions.hpp` (pure, ROS-free, unit-tested): ENU↔NED, heading↔yaw, NED-euler→ENU-quat, world→body, GPS→ENU |
| AERPAW representation / backend abstraction | `adapter_types.hpp` (`PlatformBackend`, `GeoHome`) |
| Command capability gate (per AS2 env) | `command_capability.hpp` (mirrors AERPAW's env distinction) |
| Vehicle **command** translation (AS2→AERPAW) | `command_translation.{hpp,cpp}` |
| Vehicle **state** translation (AERPAW→AS2) | `state_translation.{hpp,cpp}` — odom/imu/gps/**battery**; marks unavailable fields (no fabrication) |
| Connection / lifecycle + error/status propagation | `aerpaw_platform.cpp` (link watchdog → `platform_info.connected`, status/error reports) over `ipc_bridge.{hpp,cpp}` |
| Wiring / orchestration (no logic) | `aerpaw_platform.cpp` |

**Isolation guarantee**: every AERPAW concept stays inside this package
(`namespace aerpaw_platform`). Nothing AERPAW-specific is added to `as2_core` or
`as2_msgs`; the adapter only speaks the standard `sensor_measurements/*` +
`as2::AerialPlatform` contract. `translation_gtest` covers the pure math with no
vehicle, ROS graph or aerpawlib.

**ROS 2 boundary (Stage 14)**: this package is the *single* seam where AERPAW meets
ROS 2 (`AERPAW ↔ adapter ↔ ROS 2 ↔ AeroStack2`). The AERPAW-side runner
(`aerpaw_as2_runner.py`) carries **no** ROS 2 — it only speaks the ROS-free UDP/JSON
interface, and its subsystem bring-up is an opaque command it never interprets. The
pure experiment core is likewise ROS-free. Enforced by
`as2_experiment_aerpaw_multiuav/test/test_layering.py` (`*_is_ros_free`).

## Package layout

```
src/ + include/          aerpaw_platform (node) + ipc_bridge (UDP JSON)
  command_translation.*  AS2 cmd -> AERPAW JSON
  state_translation.*    AERPAW telemetry -> AS2 sensor msgs
  frame_conversions.hpp  ENU<->NED, yaw<->heading, NED-euler->ENU quat, world->body, GPS->ENU (pure)
  vehicle_identity.hpp   drone{i} <-> ports 15760+i*2  (multi-UAV map)
  adapter_types.hpp      PlatformBackend (sitl/dt/physical), GeoHome
aerpawlib_runner/        aerpaw_as2_runner.py (AERPAW-side bridge; can start AS2 subsystem)
config/                  control_modes.yaml, platform_params.yaml, fleet.yaml (Stage 18)
launch/                  as2_stack.launch.py (AS2 subsystem, fleet-config driven)
                         fleet_config.py     (declarative fleet loader: config/fleet.yaml)
                         aerpaw_platform/sitl.launch.py (dev harness)
deploy/                  cvm_setup.sh, qgc_tunnel.sh, run_aerpaw_experiment.sh (AERPAW-first)
tests/                   ipc_bridge_gtest, aerpaw_platform_gtest, translation_gtest
```

## Safe failure handling (Stage 17)

Malformed or stale data never silently reaches AeroStack2 (`telemetry_guard.hpp` + node
gates):

- **Invalid telemetry** (NaN / out-of-range / (0,0) no-fix / non-finite attitude) →
  **dropped** (throttled WARN + count); never published as odom/gps/imu; home origin only
  latched from valid data.
- **Stale/frozen telemetry** → published only when the link is fresh AND the packet `ts`
  advanced; the adapter **stops re-emitting** the last sample (the link watchdog flips
  `platform_info.connected=false`).
- **Malformed command** (non-finite pose/twist) → dropped, service/command returns false.
- **Unsupported command** → rejected at `set_platform_control_mode` (see *Command path*).
- **Adapter startup failure** (IPC port busy) → `RCLCPP_FATAL` + throw (node exits; no
  half-dead "up" platform). **AeroStack2 startup failure** → `run_aerpaw_experiment.sh`
  aborts rather than launching vehicles into a dead robotics layer.
- **Missing/stale wireless measurement** → `TopicMeasurementSource(max_age_s)` reports it
  *unavailable*, so the policy degrades to geometry (never acts on frozen RF).

Mission-level failsafe (RTL/land) stays in AeroStack2, fed by truthful
`platform_info.connected`. Unknown stays unknown — nothing is fabricated to "recover".

## Limitations

- **Telemetry not provided by AERPAW** (never fabricated): body angular rates,
  linear acceleration, GPS covariance and battery charge-state are marked
  *unavailable* (covariance `−1` / `UNKNOWN`). Flight mode/armable/ekf_ready exist on
  the AERPAW side but AS2 `PlatformInfo` cannot carry them → logged on change only.
  See *Telemetry / state path* above.
- **Kill switch** = disarm + stop‑velocity — the strongest aerpawlib action; **not** an
  irreversible emergency motor‑stop. Keep a safety pilot on real hardware.
- **SPEED mode** depends on aerpawlib `set_velocity`, marked `[NOT SUPPORTED]` by the
  current copter filter — validate on Config A before relying on it.

Broader troubleshooting: [`docs/E2E_MANUAL.md`](../../docs/E2E_MANUAL.md) §6.
