# as2_experiment_aerpaw_multiuav

Multi-UAV triangle formation experiment that validates all three AS2 backends:

| Config | Launch file | Backend |
|--------|------------|---------|
| **A**  | `sim_multirotor.launch.py` | AS2 Multirotor Simulator |
| **B**  | `sim_gazebo.launch.py`     | Gazebo Harmonic           |
| **C**  | `aerpaw_digital_twin.launch.py` | AERPAW Digital Twin / SITL |

All configs launch the full AS2 stack per drone: platform node, state estimator,
motion controller, and behaviour servers (arm, offboard, takeoff, land, go_to).

## Mission

Three drones (`drone0` is the leader, `drone1` and `drone2` are followers at
±10 m lateral offset) fly a 50 m square at 25 m altitude, then land.
Uses the synchronous `DroneInterface` API with threading for concurrency.

See `missions/triangle_formation.py`.

## Prerequisites

```bash
colcon build --packages-select as2_platform_aerpaw as2_experiment_aerpaw_multiuav
source install/setup.bash
```

For Config C only:

```bash
pip install -e "aerpawlib[sitl,dev]"
aerpawlib-setup-sitl   # first time only
```

## Config A — Multirotor Simulator

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

## Config B — Gazebo

Create a simulation YAML defining 3 drones (see `as2_gazebo_assets` docs), then:

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_gazebo.launch.py \
    simulation_config_file:=/path/to/three_drone_world.yaml
```

## Config C — AERPAW Digital Twin / SITL

SITL (local validation):

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=false
```

AERPAW Digital Twin:

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=true \
    conn_drone0:=udpin://10.14.X.X:14550 \
    conn_drone1:=udpin://10.14.X.Y:14550 \
    conn_drone2:=udpin://10.14.X.Z:14550
```

## Monitoring

```bash
ros2 topic echo /drone0/sensor_measurements/odom
ros2 topic echo /drone0/sensor_measurements/imu
ros2 run rviz2 rviz2  # add TF + Odometry per namespace
```

The state estimator consumes `sensor_measurements/*` and produces
`self_localization/*` (pose, twist, odom).

## Notes

- **Minimum takeoff altitude**: 25 m (AERPAW testbed requires ≥ 20 m; configurable in `platform_params.yaml`).
- **No upstream modifications**: everything lives in new packages only.
- Code style: `ament_clang_format` (C++), `ament_flake8`/`ament_pep257` (Python).
