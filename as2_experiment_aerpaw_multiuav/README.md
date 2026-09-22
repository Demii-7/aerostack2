# as2_experiment_aerpaw_multiuav

Example multi‑UAV experiment (3‑drone triangle/square formation) that exercises all
three AeroStack2 backends through the same code path.

> Prerequisites: [`docs/run_and_test_requirements.txt`](../docs/run_and_test_requirements.txt)
> · full workflow / build / setup: [`docs/E2E_MANUAL.md`](../docs/E2E_MANUAL.md)
> · bridge reference: [`as2_platform_aerpaw`](../as2_aerial_platforms/as2_platform_aerpaw/README.md)
>
> **This page owns the run commands below.**

## Mission

`drone0` is the leader; `drone1`/`drone2` follow at a ±10 m lateral offset. All three
arm → take off to 25 m → fly a 50 m square → land, using the synchronous `DroneInterface`
API with one thread per drone. See `missions/triangle_formation.py`.

## Build the experiment

```bash
colcon build --packages-up-to as2_experiment_aerpaw_multiuav
source install/setup.bash
```

## Configs

| Config | Backend | Launch file | Extra deps |
|--------|---------|-------------|-----------|
| **A** | AS2 Multirotor Simulator | `sim_multirotor.launch.py` | none |
| **B** | Gazebo Harmonic | `sim_gazebo.launch.py` | `as2_gazebo_assets`, `as2_platform_gazebo`, Gazebo Harmonic (**Jazzy**) |
| **C** | AERPAW Digital Twin / SITL | `aerpaw_digital_twin.launch.py` | aerpawlib (+ SITL, or a running AERPAW experiment) |

### A — Multirotor Simulator

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

### B — Gazebo

Needs a 3‑drone world YAML (an example ships at `config/three_drone_world.yaml`):

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_gazebo.launch.py \
  simulation_config_file:=$(pwd)/as2_experiment_aerpaw_multiuav/config/three_drone_world.yaml
```

> Run on **ROS 2 Jazzy / Gazebo Harmonic**. Humble's bundled Fortress lacks the platform
> SPEED‑mode support, so B does not fully fly there.

### C — SITL / Digital Twin

Local SITL (start 3 ArduCopter SITL instances sending to UDP 14550 / 14560 / 14570 first):

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py use_aerpaw:=false
```

Real AERPAW vehicles (per‑drone MAVLink endpoints):

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
  use_aerpaw:=true \
  conn_drone0:=udpin://10.14.X.X:14550 \
  conn_drone1:=udpin://10.14.X.Y:14550 \
  conn_drone2:=udpin://10.14.X.Z:14550
```

`aerpaw_digital_twin.launch.py` brings up, per drone: the AERPAW platform + runner, state
estimator, motion controller, and arm/offboard/takeoff/land/go_to behaviours, then the
mission script.

## Monitor

```bash
ros2 topic echo /drone0/self_localization/pose      # estimator output
ros2 topic echo /drone0/sensor_measurements/odom    # raw platform telemetry
ros2 run rviz2 rviz2                                 # add TF + Odometry per namespace
```

## Notes

- **Minimum takeoff altitude 25 m** (AERPAW requirement; set in the platform
  `platform_params.yaml`).
- All code lives in these new packages — no upstream AS2 modifications.
- Style: `ament_clang_format` (C++), `ament_flake8`/`ament_pep257` (Python).
