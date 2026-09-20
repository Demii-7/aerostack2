# as2_platform_aerpaw

Aerial platform that bridges the AeroStack2 framework to the AERPAW Digital Twin via
[aerpawlib](https://github.com/AERPAW/aerpawlib) (ArduPilot/MAVLink).

The C++ ROS 2 node communicates with a Python aerpawlib runner over localhost
UDP sockets using one JSON object per datagram.  The Python runner translates
commands into aerpawlib `Drone` API calls and streams telemetry back.

## IPC architecture

Commands flow C++ → Python over the command port (`15760`); the Python runner
binds it and C++ only sends to it.  Telemetry flows Python → C++ over the
telemetry port (`15761`), which the C++ bridge binds.  UDP is connectionless —
each datagram carries exactly one JSON object.  Per-drone ports increment by
`i*2` (see `aerpaw_sitl.launch.py`).

## Frame conventions

- **AS2 odom frame**: ENU (x=East, y=North, z=Up).
- **ArduPilot heading**: degrees from North, clockwise (0=N, 90=E).
- **Conversion (C++)**: `heading_aerpaw = fmod(90 - yaw_enu_deg, 360)`.
- The twist/pose commands arrive in ENU (odom) frame via `setCommandPoseFrameId(getOdomFrameId())`.
  The NED conversion is a pure axis permutation (`north=y_enu`, `east=x_enu`, `down=-z_enu`)
  — no yaw rotation needed because the base class already converts to odom.

## Telemetry topics

The platform publishes raw sensor data; the AS2 state estimator produces
localisation from these:

| Topic | Type | Source |
|-------|------|--------|
| `sensor_measurements/odom` | `nav_msgs/Odometry` | Flat-earth ENU near home |
| `sensor_measurements/imu` | `sensor_msgs/Imu` | Orientation from ArduPilot heading |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | Raw lat/lon/alt |

## Kill switch

`ownKillSwitch()` maps to `set_armed(False)` + `stop_velocity()` — the strongest
action available through aerpawlib.  This is **not** an irreversible emergency motor
stop (AS2 kill-switch semantics).  Documented limitation for real-hardware safety.

## Package layout

```
as2_aerial_platforms/as2_platform_aerpaw/
├── config/
│   ├── control_modes.yaml        # HOVER / SPEED(yaw-speed) / POSITION(yaw-angle)
│   └── platform_params.yaml      # default IPC ports, takeoff_altitude
├── include/as2_platform_aerpaw/
│   ├── aerpaw_platform.hpp
│   └── ipc_bridge.hpp
├── src/
│   ├── aerpaw_platform.cpp
│   ├── ipc_bridge.cpp
│   └── aerpaw_platform_node.cpp
├── aerpawlib_runner/
│   └── aerpaw_as2_runner.py      # Python bridge script (BasicRunner + @entrypoint)
├── launch/
│   ├── aerpaw_platform.launch.py  # single drone
│   └── aerpaw_sitl.launch.py      # N drones on SITL / AERPAW DT
└── tests/
    ├── ipc_bridge_gtest.cpp
    └── aerpaw_platform_gtest.cpp
```

## Build

```bash
colcon build --packages-select as2_platform_aerpaw
source install/setup.bash
```

**Dependencies** (outside the colcon workspace):

```bash
git clone https://github.com/AERPAW/aerpawlib.git
pip install -e "aerpawlib[sitl,dev]"
aerpawlib-setup-sitl    # first time only
```

## Launch arguments

### `aerpaw_platform.launch.py` (single drone)

| Argument       | Default                       | Description                              |
|----------------|-------------------------------|------------------------------------------|
| `namespace`    | `drone0`                      | ROS 2 drone namespace                    |
| `conn`         | `udpin://127.0.0.1:14550`    | MAVLink connection string                |
| `cmd_port`     | `15760`                       | IPC command UDP port                     |
| `tel_port`     | `15761`                       | IPC telemetry UDP port                   |
| `use_sim_time` | `false`                       | Use simulation clock                     |
| `use_aerpaw`   | `false`                       | Set `true` inside AERPAW environment     |

### `aerpaw_sitl.launch.py` (N drones)

Same as above plus:

| Argument        | Default | Description                  |
|-----------------|---------|------------------------------|
| `num_drones`    | `3`     | Number of drones to spawn    |
| `conn_drone<i>` | (auto)  | Per-drone connection string  |

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Runner does not connect | The platform node and aerpawlib runner use connectionless UDP — no "wait for connect" step. Ensure ports match on both sides. |
| aerpawlib errors / SITL connection refused | Run `aerpawlib-setup-sitl` and verify `aerpawlib --help` works. |
| Port conflict | Change `cmd_port` / `tel_port` for each drone (e.g. 15760+i\*2); `aerpaw_sitl.launch.py` does this automatically. |
