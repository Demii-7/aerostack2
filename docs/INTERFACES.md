# Integration Interfaces (Stage 15)

Explicit, **versioned** contract for the three seams of the AERPAW ↔ AeroStack2
integration. This is the source of truth for what crosses each boundary; code and this
table are kept in sync (several are asserted by tests, noted per section).

```
AERPAW  ==IF-1==>  Adapter  ==IF-2==>  AeroStack2  ==IF-3==>  Experiment algorithm
 (UDP/JSON)        (ROS 2)              (as2_python_api / topics)
```

> Frames, units, and sign conventions referenced throughout are defined in
> [`COORDINATE_FRAMES.md`](COORDINATE_FRAMES.md).

## Interface registry & versioning

| ID | Boundary | Mechanism | Version | Where defined |
|----|----------|-----------|---------|---------------|
| **IF-1** | AERPAW ↔ Adapter | localhost UDP, one JSON object/datagram | **v1** | `ipc_bridge.hpp` `kProtocolVersion`, `aerpaw_as2_runner.PROTOCOL_VERSION`, `command_translation.cpp` |
| **IF-2** | Adapter ↔ AeroStack2 | ROS 2 topics/services/actions (as2_core names) | tracks AS2 release | `as2_core/include/as2_core/names/*`, `aerpaw_platform.cpp` |
| **IF-3** | AERPAW ↔ Experiment | Python Protocols/dataclasses (+ one ROS passthrough topic) | **v1** | `experiment/interfaces.py`, `rf_metrics.py`, `closed_loop.py` |

**Policy:** MAJOR = incompatible (removed/renamed/re-typed field, changed semantics);
MINOR = additive/optional. IF-1 bumps the integer in **both** `ipc_bridge.hpp` and
`aerpaw_as2_runner.py` (guarded by `test_layering.test_protocol_version_consistent`);
the adapter logs a one-shot warning if a runner speaks a different IF-1 version.

---

## IF-1 · AERPAW ↔ Adapter (UDP/JSON, v1)

**Transport:** connectionless localhost UDP, one JSON object per datagram.
**Ports:** command port (runner binds, adapter→AERPAW commands) + telemetry port (adapter
binds, AERPAW→adapter data), per-drone offset `+i*2` → `drone0` `15760/15761`, `drone1`
`15762/15763`, … (`vehicle_identity.hpp`).
**Envelope (inbound AERPAW→adapter):** `type` (default `telemetry`), `proto` (int),
`vehicle_id` (str), `ts` (float, epoch s).

### Vehicle IDs
| Field | Meaning |
|-------|---------|
| `vehicle_id` | AERPAW-side name (OEO node/alias); echoed by the runner (`--vehicle-id`) |
| `mavlink_sysid` | AERPAW hardware-unique MAVLink system id (adopted by the platform) |
| ROS namespace `drone{i}` | the AS2 platform identity (1 UAV ↔ 1 platform) |

### Commands (adapter → AERPAW) — `command_translation.cpp`
| `cmd` | Fields | Units | Maps to (aerpawlib) |
|-------|--------|-------|---------------------|
| `arm` / `disarm` | — | — | `set_armed(True/False)` |
| `takeoff` | `alt` | m (rel) | `takeoff(altitude=alt)` |
| `land` | — | — | `land()` |
| `set_velocity` | `vx,vy,vz` | m/s **NED** | `set_velocity(VectorNED)` — **SITL only** (testbed filter blocks) |
| `goto_ned` | `north,east,down` (m NED), `heading` (deg from North CW), `home_lat,home_lon` (deg) | m/deg | `goto_coordinates(home+VectorNED, target_heading)` |
| `kill` | — | — | disarm (+ stop) — not irreversible |
| `stop` | — | — | SITL stop; testbed uses `goto_ned` to current pose |

### State messages (AERPAW → adapter) — `type:"telemetry"`
| Field(s) | Meaning | Availability |
|----------|---------|--------------|
| `lat,lon,alt_msl,rel_alt` | WGS-84 pos + MSL/relative alt (m) | always |
| `vx_ned,vy_ned,vz_ned` | NED velocity (m/s) | always |
| `roll_rad,pitch_rad,yaw_ned_rad` | attitude, MAVSDK NED/FRD euler (rad) — authoritative | always |
| `roll,pitch` (rad), `yaw` (deg) | legacy keys (back-compat) | always |
| `gps_fix_type` (0/1 none,2 2D,3 3D), `gps_satellites` | GNSS quality | when aerpawlib provides |
| `battery_voltage,current,level` (level 0–100 %) | power | when aerpawlib provides |
| `armed,armable,ekf_ready,connected` (bool), `mode` (str) | flight/lifecycle state | when provided |
| `vehicle_id, mavlink_sysid, ts, proto` | ids / timestamp / version | always |

Unavailable fields are **omitted, not zeroed** (the adapter maps absence to ROS
"unknown": covariance −1 / `UNKNOWN`; see ARCHITECTURE §7).

### Lifecycle / status / error (AERPAW → adapter)
| `type` | Fields | Adapter handling |
|--------|--------|------------------|
| `status` | `message` | logged INFO |
| `error` | `command`, `message` | logged ERROR (command rejected by AERPAW) |
Link lifecycle is derived by the adapter from telemetry freshness (`link_timeout`):
no packet → `platform_info.connected=false`.

### Wireless measurement passthrough (AERPAW → adapter → ROS)
| `type` | Fields | Handling |
|--------|--------|----------|
| `measurement` | `metrics` (object, e.g. `sinr_db,rssi_dbm,throughput_mbps,packet_loss,channel`) | forwarded **verbatim** (transport only) to IF-2 `<ns>/aerpaw/measurements` |

---

## IF-2 · Adapter ↔ AeroStack2 (ROS 2)

All names are **relative** and thus namespaced per vehicle (`/drone{i}/…`) — this is what
gives independent per-UAV state/command interfaces with no cross-control (verified by
`aerpaw_platform_gtest.MultiUavNamespaceIsolation`). QoS: `SensorDataQoS` for sensor/
command topics.

### Platform state — adapter PUBLISHES
| Topic | Type | Notes |
|-------|------|-------|
| `sensor_measurements/odom` | `nav_msgs/Odometry` | ENU pose+body twist; rates cov −1 (unknown) |
| `sensor_measurements/imu` | `sensor_msgs/Imu` | orientation from attitude; gyro/accel cov −1 |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | lat/lon/alt + fix status |
| `sensor_measurements/battery` | `sensor_msgs/BatteryState` | voltage/current/`percentage`=level/100 |
| `platform/info` | `as2_msgs/PlatformInfo` | `connected`(watchdog)/`armed`/`offboard`/control mode |
| `aerpaw/measurements` | `std_msgs/String` | IF-1 measurement passthrough (JSON) |
The AS2 state estimator (`raw_odometry`) turns `sensor_measurements/*` into
`self_localization/{pose,twist,odom}` (adapter does not estimate).

### Motion / trajectory commands — adapter SUBSCRIBES
| Topic | Type | Control mode | AERPAW execution |
|-------|------|--------------|------------------|
| `actuator_command/pose` | `geometry_msgs/PoseStamped` | POSITION (yaw ANGLE), odom/ENU | `goto_ned` ✅ (all envs) |
| `actuator_command/twist` | `geometry_msgs/TwistStamped` | SPEED, odom/ENU | `set_velocity` — SITL only |
| `actuator_command/trajectory` | `as2_msgs/TrajectorySetpoints` | TRAJECTORY | ❌ rejected (offboard; capability gate) |
| `actuator_command/thrust` | `as2_msgs/Thrust` | — | ❌ rejected |

### Behavior requests (services the platform exposes)
`set_arming_state`, `set_offboard_mode`, `set_platform_control_mode`,
`platform_takeoff`, `platform_land`, `platform/list_control_modes`,
`platform/state_machine_event` (+ `alert_event` topic for KILL/EMERGENCY). Declared
control modes: `HOVER`, `SPEED`(yaw SPEED), `POSITION`(yaw ANGLE) —
`config/control_modes.yaml`. Unsupported mode requests are **rejected at
`set_platform_control_mode`** (service returns success=false; see ARCHITECTURE §8/§11).

### Status feedback
`platform/info` (connection/arming/control mode), `controller/info` (AS2 controller).

---

## IF-3 · AERPAW ↔ Experiment Algorithm (Python, v1)

Clean, ROS-free Python interfaces the experiment code depends on; ROS 2 is hidden behind
`PlatformInterface`/`TopicMeasurementSource` implementations (guarded by
`test_experiment_core_is_ros_free`).

### UAV state — `interfaces.VehicleState`
`vehicle_id`, `x,y,z` (m, AS2 ENU/odom), `yaw` (rad, CCW, 0=East), `battery_fraction`
(0–1, optional), `connected`, `mode`.

### Wireless measurements — `rf_metrics`
- `RFMetrics`: `vehicle_id, sinr_db, rssi_dbm, throughput_mbps, packet_loss, channel,
  timestamp` (all Optional → `None` = not measured) + `extra`; `.available`.
- **`MeasurementSource`** (Protocol): `poll(vehicle_ids) -> dict[id, RFMetrics]`.
  Implementations: `UnavailableRFSource` (no feed), `CallbackMeasurementSource`
  (custom/sim), `TopicMeasurementSource` (reads IF-2 `aerpaw/measurements`),
  `from_json()` parses the IF-1 metrics object.
- **Timestamps:** `RFMetrics.timestamp` ← IF-1 `ts` (epoch s); per-sample.

### Vehicle operations — `interfaces.PlatformInterface` (Protocol)
`arm`, `offboard`, `takeoff(height,speed)`, `land(speed)`,
`go_to(x,y,z,speed,yaw?)`, `follow_path(points,speed)`, `hover`, `get_state()`, `close()`.
Implemented by `DroneInterfacePlatform` over `as2_python_api` (AS2 behaviours); the
experiment never touches ROS/adapter directly.

### Algorithm outputs — `closed_loop`
- `RobotCommand`: `kind` ∈ {`takeoff,land,go_to,hover,follow_path`}, `x,y,z,yaw,speed,
  path,height`.
- **`RobotPolicy`** (Protocol): `decide(states: dict[id,VehicleState],
  metrics: dict[id,RFMetrics]) -> dict[id, RobotCommand]` — sees the **whole fleet**
  (shared multi-UAV state; `fleet.FleetState` for geometry queries) each tick.
- `ClosedLoopRunner`: `sense (get_state + poll) → decide → act`; `run(duration, period)`
  — the wireless→robot closed loop (online updates per §11; latest-wins).

### Experiment events
- Per-step `(states, metrics, commands)` (the closed-loop tick).
- AERPAW flight-state changes surfaced by the adapter (`mode/armed/armable/ekf_ready`,
  ARCHITECTURE §7); wireless metrics via `MeasurementSource`.

---

## Traceability
- IF-1 fields ↔ `ipc_bridge.cpp` (inbound) + `command_translation.cpp` (outbound) +
  `aerpaw_as2_runner.py`.
- IF-2 names ↔ `as2_core/names/topics.hpp`,`services.hpp` + `aerpaw_platform.cpp`.
- IF-3 ↔ `experiment/{interfaces,rf_metrics,closed_loop,fleet}.py`.
- Enforced by tests: `test_protocol_version_consistent` (IF-1 version),
  `MultiUavNamespaceIsolation` (IF-2 namespacing), `test_experiment_core_is_ros_free`
  (IF-3 ROS containment), `MeasurementIsTransportedVerbatim` (IF-1→IF-2 passthrough).
