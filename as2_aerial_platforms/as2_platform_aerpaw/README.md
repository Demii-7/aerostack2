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

## Bring up one drone (smoke test)

```bash
ros2 launch as2_platform_aerpaw aerpaw_platform.launch.py \
  namespace:=drone0 conn:=udpin://127.0.0.1:14550 use_aerpaw:=false

ros2 topic echo /drone0/self_localization/pose     # should stream once telemetry arrives
```

N drones: `aerpaw_sitl.launch.py num_drones:=3` (adds the runner + per‑drone ports).

## IPC contract

| Item | Value |
|------|-------|
| Transport | localhost UDP, connectionless, **one JSON object per datagram** |
| Command port | `ipc_cmd_port` (15760) — the **runner binds it**, C++ sends |
| Telemetry port | `ipc_tel_port` (15761) — **C++ binds it**, runner sends |
| Per drone | offset `+i*2` → drone0 15760/15761, drone1 15762/15763, … |

### Runner command map (`cmd` → aerpawlib)

| Command | Action |
|---------|--------|
| `arm` / `disarm` | `drone.set_armed(True/False)` |
| `takeoff` / `land` | `drone.takeoff(h)` / `drone.land()` |
| `set_velocity` | `drone.set_velocity(VectorNED(vx, vy, vz))` |
| `goto_ned` | `drone.goto_coordinates(home + VectorNED(n,e,d), target_heading=…)` |
| `kill` | `set_armed(False)` + `stop_velocity()` (see *Limitations*) |

## Frames

- AS2 odom is **ENU** (x=E, y=N, z=U); the base class delivers commands already expressed
  in the odom frame, so no extra yaw rotation is applied.
- ENU→NED is a pure axis permutation: `north=+y`, `east=+x`, `down=-z`.
- ArduPilot heading (deg from North, clockwise): `heading = fmod(90 - yaw_enu, 360)`.
- `POSITION` targets are absolute offsets from the drone's **fixed home**
  (`home_lat`/`home_lon`, captured on first telemetry) → stable, non‑drifting waypoints.

## Telemetry (published by this node; consumed by the state estimator)

| Topic | Type | Source |
|-------|------|--------|
| `sensor_measurements/odom` | `nav_msgs/Odometry` | flat‑earth ENU near home |
| `sensor_measurements/imu` | `sensor_msgs/Imu` | orientation from ArduPilot heading |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | raw lat/lon/alt |

The estimator turns these into `self_localization/*`.

## Control modes (`config/control_modes.yaml`)

| Mode | Value | Used by |
|------|-------|---------|
| HOVER | `0b00010000` | idle / safety |
| SPEED (yaw speed, local FLU) | `0b01000100` | `pid_speed` velocity |
| POSITION (yaw angle, ENU) | `0b01100000` | `go_to` |

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

## Unit tests

```bash
colcon test --packages-select as2_platform_aerpaw && colcon test-result --verbose
```

`ipc_bridge_gtest` (UDP round‑trip) + `aerpaw_platform_gtest` (node constructs with the
shipped configs). No vehicle or aerpawlib needed.

## Package layout

```
src/ + include/          aerpaw_platform (node) + ipc_bridge (UDP JSON)
aerpawlib_runner/        aerpaw_as2_runner.py (BasicRunner bridge)
config/                  control_modes.yaml, platform_params.yaml
launch/                  aerpaw_platform.launch.py, aerpaw_sitl.launch.py
tests/                   ipc_bridge_gtest, aerpaw_platform_gtest
```

## Limitations

- **Kill switch** = disarm + stop‑velocity — the strongest aerpawlib action; **not** an
  irreversible emergency motor‑stop. Keep a safety pilot on real hardware.
- **SPEED mode** depends on aerpawlib `set_velocity`, marked `[NOT SUPPORTED]` by the
  current copter filter — validate on Config A before relying on it.

Broader troubleshooting: [`docs/E2E_MANUAL.md`](../../docs/E2E_MANUAL.md) §6.
