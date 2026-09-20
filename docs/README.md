# AERPAW Multi-UAV Platform — README

Bridge that lets any AeroStack2 mission run unchanged on the **AERPAW Digital
Twin** / SITL, in addition to the standard AS2 Multirotor Simulator and Gazebo
backends.

```
PLAN.md                                          original task specification
HOST.md                                          Chameleon host access notes
docs/run_and_test_requirements.txt               full prerequisite checklist
docs/E2E_MANUAL.md                               end-to-end manual (Quick Start +
                                                 parts on AERPAW, Aerostack2, the
                                                 platform, your own experiment, runs)
docs/README.md                                   this document

as2_aerial_platforms/as2_platform_aerpaw/
    src/aerpaw_platform.cpp                      C++ AerialPlatform node
    src/ipc_bridge.cpp(.hpp)                     UDP JSON IPC (command + telemetry)
    include/as2_platform_aerpaw/aerpaw_platform.hpp
    aerpawlib_runner/aerpaw_as2_runner.py        Python aerpawlib runner
    config/control_modes.yaml, platform_params.yaml
    launch/aerpaw_platform.launch.py, aerpaw_sitl.launch.py
    tests/ipc_bridge_gtest.cpp, aerpaw_platform_gtest.cpp
    README.md

as2_experiment_aerpaw_multiuav/
    missions/triangle_formation.py               3-drone square mission
    launch/sim_multirotor.launch.py              Config A
    launch/sim_gazebo.launch.py                  Config B
    launch/aerpaw_digital_twin.launch.py         Config C
    README.md
```

## Architecture (very short)

- The C++ `AerpawPlatform` node implements `as2::AerialPlatform`. Commands are
  converted to the odom (ENU) frame by the AS2 base class and sent as one JSON
  object per UDP datagram; telemetry is received the same way.
- The Python runner (`aerpaw_as2_runner.py`) binds the command port and
  translates commands into aerpawlib `Drone` API calls; it streams telemetry
  (odom/IMU/GPS) back to C++, which republishes raw `sensor_measurements/*`.
- A separate AS2 state estimator produces `self_localization/*` from those raw
  topics; the experiment launches wire the full stack per drone.

See each package README for details.

---

## How to use

Full prerequisite checklist: `docs/run_and_test_requirements.txt`.

### 1. Build

Requires ROS 2 Humble on Ubuntu 22.04 (or the repo's pixi humble env):

```bash
colcon build --packages-up-to as2_platform_aerpaw
source install/setup.bash
```

### 2. Unit test (no simulator, no aerpawlib)

```bash
colcon test --packages-select as2_platform_aerpaw
colcon test-result --verbose
```

This runs `ipc_bridge_gtest` (UDP round-trip) and `aerpaw_platform_gtest`
(node construction with the shipped configs).

### 3. Run

**Config A — Multirotor Simulator (no extra deps)**
```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

**Config B — Gazebo Harmonic**
Needs `as2_gazebo_assets` + `as2_platform_gazebo` + gz stack, and a 3-drone
world YAML:
```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_gazebo.launch.py \
    simulation_config_file:=$(pwd)/your_three_drone_world.yaml
```

**Config C — AERPAW DT / SITL**
Needs aerpawlib + SITL on the target machine:
```bash
pip install -e "aerpawlib[sitl,dev]" && aerpawlib-setup-sitl
# local SITL:
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py use_aerpaw:=false
# real Digital Twin (per-drone MAVLink URLs):
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=true \
    conn_drone0:=udpin://10.14.X.X:14550 conn_drone1:=udpin://10.14.X.Y:14550 conn_drone2:=udpin://10.14.X.Z:14550
```

**Monitor** (`/drone0/sensor_measurements/{odom,imu,gps}`, `self_localization/*`):
```bash
ros2 run rviz2 rviz2
```

---

## What is left to implement / verify

Done and working-deadline items are ticked; the rest are the remaining steps.

- [x] Packages written: C++ platform, Python runner, multi-UAV experiment
- [x] Code implements POSITION/SPEED/HOVER; ENU<->ArduPilot heading conversion;
      fixed-home absolute POSITION target; `sensor_measurements/*` telemetry
- [x] Mission uses the synchronous AS2 API (threading, no async/await misuse)
- [x] Launch files wire full stack (state estimator + motion controller +
      arm/offboard/takeoff/land/go_to behaviors)
- [ ] **Compile & run gtests on a ROS 2 Humble host** — not done here (no ROS
      in this environment; run the two commands in §2 above). This is the #1
      remaining action.
- [ ] Transform unit tests (SPEED yaw rotation at 0°/90°, POSITION with a fixed
      home) — worth adding as gtests once the build works.
- [ ] Runtime validation of all three configs (A quickest; B needs a 3-drone
      Gazebo world YAML; C needs SITL and, ideally, AERPAW hardware access).
- [ ] Confirm actual behavior of SPEED mode: aerpawlib v2 `set_velocity` is
      marked `[NOT SUPPORTED]` by the current copter filter; validate on
      Config A/SITL and adjust if needed.
- [ ] (Optional) True AS2 emergency-stop semantics: current `kill` maps to
      disarm + stop_velocity — no irreversible motor-stop API in aerpawlib.
- [x] (Optional) Move package under `as2_aerial_platforms/` to match repo
      convention (DONE — now at `as2_aerial_platforms/as2_platform_aerpaw/`).

## Known limitations

- kill = disarm + stop velocity (no true emergency motor stop)
- SPEED-mode behavior depends on aerpawlib copter filter
- Config C requires the 25 m minimum AERPAW takeoff altitude (default
  `takeoff_altitude: 25.0` in `platform_params.yaml`)