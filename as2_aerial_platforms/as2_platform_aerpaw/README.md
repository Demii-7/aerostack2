# as2_platform_aerpaw

Aerial platform that bridges [AeroStack2](https://github.com/AERPAW/aerostack2) to
ArduPilot vehicles via [aerpawlib](https://github.com/AERPAW/aerpawlib). A C++ ROS 2
node talks to a small Python **aerpawlib runner** over localhost UDP (one JSON object
per datagram); the runner drives the real vehicle, Digital Twin, or SITL through the
aerpawlib `Drone` API and streams telemetry back.

**Same workflow, three targets** — swap one flag:

| Target | `use_aerpaw` | `conn` |
|--------|--------------|--------|
| Local SITL / Digital Twin | `false` | `udpin://127.0.0.1:14550` |
| Real AERPAW vehicle (E-VM) | `true` | AERPAW-assigned `udpin://<ip>:<port>` |

---

## Quick start

Three phases, ~10 min first run. New to this? Start on the **Digital Twin / SITL** —
identical commands, no hardware.

### Phase 1 — Install & build  · your machine

1. **Install the Python bridge (one-time).**
   ```bash
   git clone https://github.com/AERPAW/aerpawlib.git
   pip install -e "aerpawlib[sitl,dev]"
   aerpawlib-setup-sitl          # first time only
   ```
2. **Build the platform in your ROS 2 workspace.**
   ```bash
   colcon build --packages-select as2_platform_aerpaw
   source install/setup.bash
   ```

   > Tip: the `aerpawlib` runner must be importable by the launch (`aerpawlib` on
   > `PATH`, and its own Python env not shadowed by ROS's `PYTHONPATH`).

### Phase 2 — Bring up one drone  · SITL / Digital Twin

1. **Launch the platform + runner for `drone0`.**
   ```bash
   ros2 launch as2_platform_aerpaw aerpaw_platform.launch.py \
     namespace:=drone0 \
     conn:=udpin://127.0.0.1:14550 \
     use_aerpaw:=false
   ```
2. **Verify it's alive** — position should stream from the vehicle.
   ```bash
   ros2 topic echo /drone0/self_localization/pose
   ```

   > Tip: AERPAW requires a minimum takeoff altitude of **25 m** (set by
   > `takeoff_altitude` in `config/platform_params.yaml`).

### Phase 3 — Run a multi-drone mission  · E-VM / Digital Twin

1. **Launch the full stack** — brings up N platforms + runners + state estimator +
   controller + behaviors + the mission script in one call.
   ```bash
   ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
     num_drones:=3 use_aerpaw:=false
   ```
   For real vehicles, add the per-drone endpoints and flip the flag:
   ```bash
   ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
     num_drones:=3 use_aerpaw:=true \
     conn_drone0:=udpin://10.14.X.X:14550 \
     conn_drone1:=udpin://10.14.X.Y:14550
   ```
2. **Stop cleanly.**
   ```bash
   # Ctrl+C in the launch terminal
   ros2 lifecycle set /drone0/platform shutdown
   ```

   > Tip: QGroundControl can be used read-only to watch the drones; AS2 does the
   > commanding, the safety pilot keeps final authority.

---

## Launch arguments

`aerpaw_platform.launch.py` (single drone):

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `drone0` | ROS 2 drone namespace |
| `conn` | `udpin://127.0.0.1:14550` | MAVLink connection string |
| `cmd_port` | `15760` | IPC command UDP port (C++ → Python) |
| `tel_port` | `15761` | IPC telemetry UDP port (Python → C++) |
| `use_sim_time` | `false` | Use the simulation clock |
| `use_aerpaw` | `false` | `true` inside an AERPAW E-VM |

`aerpaw_sitl.launch.py` (N drones) adds `num_drones` (default `3`) and `conn_drone<i>`
(per-drone connection; ports auto-increment by `i*2`).

## Telemetry

The platform publishes raw sensor data; the AS2 state estimator turns it into
localization:

| Topic | Type | Notes |
|-------|------|-------|
| `sensor_measurements/odom` | `nav_msgs/Odometry` | flat-earth ENU near home |
| `sensor_measurements/imu` | `sensor_msgs/Imu` | orientation from ArduPilot heading |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | raw lat/lon/alt |

**Frames:** AS2 odom is ENU (x=E, y=N, z=U); ArduPilot heading is degrees from North,
clockwise. The bridge converts with `heading = fmod(90 - yaw_enu, 360)` and a pure axis
permutation for NED — no extra yaw rotation, because commands arrive already expressed
in the odom frame.

## Safety

`ownKillSwitch()` maps to `set_armed(False)` + `stop_velocity()` — the strongest action
aerpawlib exposes. This is **not** an irreversible motor-stop, so keep a physical safety
pilot on real hardware.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| No `/drone0/self_localization/pose` | Check the `conn` string and that a vehicle/SITL is actually sending MAVLink. UDP here is connectionless — no "wait for connect". |
| SITL won't connect | Run `aerpawlib-setup-sitl`; make sure SITL is sending to the same port `conn` binds. |
| Port conflict (N drones) | `aerpaw_sitl.launch.py` increments `cmd/tel` by `i*2`; otherwise pass distinct `cmd_port`/`tel_port`. |
| `aerpawlib` import errors | The runner uses its own Python env; ensure it isn't shadowed by the ROS env's `PYTHONPATH`. |

## Layout

```
as2_platform_aerpaw/
├── config/            control_modes.yaml, platform_params.yaml
├── src/ + include/    C++ platform node + IPC bridge
├── aerpawlib_runner/  aerpaw_as2_runner.py (Python ↔ vehicle bridge)
├── launch/            aerpaw_platform.launch.py, aerpaw_sitl.launch.py
└── tests/             ipc_bridge_gtest, aerpaw_platform_gtest
```
