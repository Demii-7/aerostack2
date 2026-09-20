# End-to-End Manual: AERPAW + Aerostack2

Run Aerostack2 (AS2) multi-UAV experiments on the AERPAW Digital Twin and the
built-in simulators through the `as2_platform_aerpaw` bridge.

This manual takes you from a fresh machine to a flying multi-vehicle experiment,
covering in order:

1. **AERPAW** — aerpawlib, local SITL, and Digital Twin access
2. **Aerostack2** — ROS 2 Humble, workspace build, platform package
3. **The platform** — what the `as2_platform_aerpaw` bridge does, how to configure it
4. **Your own experiment** — create an AS2 mission and the launch files to run it
5. **End-to-end runs** — the three configurations, including on the real DT

A **Quick Start** is at the top. Details follow.

---

## 0. Quick Start

Prerequisite one-liner (Ubuntu 22.04, ROS 2 Humble already sourced):

```bash
# 1) Install deps + build the platform and its AS2 dependencies
sudo apt install nlohmann-json3-dev
colcon build --packages-up-to as2_platform_aerpaw
source install/setup.bash

# 2) Run the unit tests (no simulator, no aerpawlib)
colcon test --packages-select as2_platform_aerpaw
colcon test-result --verbose

# 3) Fly the 3-drone mission on the AS2 Multirotor Simulator (no extra deps)
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

For the AERPAW path (SITL / Digital Twin) instead of the built-in simulator,
continue reading — Parts 1 and 5 (Config C).

---

## 1. Architecture in one picture

```
                   AS2 WORLD                          AERPAW WORLD
                                                     
  Your mission (as2_python_api / launch files)
        |
        v
  Aerostack2 framework
     - behaviours (go_to, takeoff, land, arm, offboard)
     - motion controller (pid_speed)
     - state estimator (raw_odometry / ground_truth)
        |                                       ^
        v                                       |
  as2_platform_aerpaw (C++ AerialPlatform) ------+
     = UDP JSON bridge, one JSON object per datagram
        |  cmd-port 15760+i*2   /   tel-port 15761+i*2
        v
  aerpaw_as2_runner.py  (Python, aerpawlib v2, BasicRunner)
        |
        v
  ArduPilot vehicle --MAVLink-->  [ AS2 Simulator | Gazebo | AERPAW DT | SITL ]
```

*Commands* flow C++ -> runner -> ArduPilot. *Telemetry* flows back and is
published by the C++ node as raw `sensor_measurements/*`; the AS2 state
estimator turns it into `self_localization/*`, which the motion controller and
behaviours consume. That is why the *same* mission runs unchanged on every
backend.

---

## 1. Part 1 — Set up the AERPAW side

### 1.1 Get an AERPAW account and console access

1. Request access at the AERPAW portal:
   - Console: https://console.aerpaw.org
   - User manual: https://sites.google.com/ncsu.edu/aerpaw-user-manual/
     (start with the "Hello AERPAW!" sample experiment)
2. Have the PI create an **experiment** and add you as an experimenter.
3. Within the experiment, reserve the nodes you need:
   - **Portable node(s)** = vehicles (each UAV/rover runs on a portable node)
   - Fixed nodes = communication sites (not needed for pure flight tests)
4. Record the addresses you will use for MAVLink later:
   - Real testbed: the vehicle control script connects to the message-filter
     endpoint of the vehicle E-VM (commonly something like `192.168.32.X:14550`).
   - Digital Twin: the emulation endpoints are **virtual** addresses you read
     from the console / vehicle E-VM (e.g. `10.14.X.X:14550`).
   These strings become `conn_droneN` in Section 5.4.

### 1.2 Install aerpawlib locally (development + SITL)

```bash
git clone https://github.com/AERPAW/aerpawlib.git
cd aerpawlib
pip install -e "aerpawlib[sitl,dev]"   # editable, plus ArduPilot SITL tooling
aerpawlib-setup-sitl                   # first time only: builds ArduPilot SITL
```

> On AERPAW C-VMs aerpawlib is preinstalled; conda/venv users may need the
> `pip install` above inside the active environment.

### 1.3 Verify SITL + aerpawlib standalone (before any AS2)

a) Start ArduPilot SITL (Copter) on the default udpin://127.0.0.1:14550
   (`aerpawlib-setup-sitl` installs tooling; or run `sim_vehicle.py` yourself).

b) Write `/tmp/hello.py`:

```python
from aerpawlib.v2 import Drone, VectorNED, BasicRunner, entrypoint

class Hello(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=25)   # 25 m minimum on AERPAW
        await drone.goto_coordinates(drone.position + VectorNED(10, 0))
        await drone.land()
```

c) Run it:

```bash
aerpawlib --api-version v2 --script /tmp/hello.py \
          --conn udpin://127.0.0.1:14550 --vehicle drone --no-aerpaw-environment
```

`--no-aerpaw-environment` is mandatory **outside** the AERPAW testbed. Full API
reference: https://aerpaw.github.io/aerpawlib

### 1.4 What "launching on AERPAW" means later

Each vehicle has its own MAVLink endpoint. Aerostack2 runs *one*
`as2_platform_aerpaw` node + *one* runner per vehicle, so N drones need N
distinct connection strings (Section 5.4).

---

## 2. Part 2 — Set up Aerostack2

### 2.1 Install ROS 2 Humble (Ubuntu 22.04)

Choose one:

- **Official apt** (recommended): https://docs.ros.org/en/humble/Installation.html
- **conda / robostack** (self-contained env; used in our CI):
  ```bash
  conda create -n humble ros-humble-desktop colcon-common-extensions \
      cmake nlohmann_json gtest -c robostack -c conda-forge
  conda activate humble
  ```
- **pixi** (the repo ships a `pixi.toml` with a `humble` environment):
  ```bash
  curl -fsSL https://pixi.sh/install.sh | sh
  pixi install
  pixi run -e humble bash
  ```

Verify:

```bash
ros2 --version && printenv ROS_DISTRO   # expect humble
```

### 2.2 Get the workspace and build

```bash
git clone https://github.com/aerostack2/aerostack2.git
cd aerostack2
# this repository already contains as2_platform_aerpaw and
# as2_experiment_aerpaw_multiuav (docs/README.md lists them)

# Resolve dependency repos listed in aerostack2.repos, then if using src layout:
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-up-to as2_platform_aerpaw --event-handlers console_direct+
source install/setup.bash
```

The platform depends on `as2_msgs`, `as2_interfaces`, `as2_core`, and
`as2_python_api`; the experiment additionally needs the simulator, estimator,
controller, and behaviour packages. `--packages-up-to` builds them automatically.

### 2.3 Build and unit-test the platform

```bash
colcon build --packages-select as2_platform_aerpaw as2_experiment_aerpaw_multiuav
source install/setup.bash

colcon test --packages-select as2_platform_aerpaw
colcon test-result --verbose
```

Expected tests: `ipc_bridge_gtest` (UDP JSON round-trip on localhost) and
`aerpaw_platform_gtest` (node construction with the shipped YAML configs).
Neither needs aerpawlib or a vehicle.

---

## 3. Part 3 — Understand the platform bridge

### 3.1 Packages

```
as2_platform_aerpaw/                       the AERPAW platform
  src/aerpaw_platform.cpp                  AerpawPlatform : as2::AerialPlatform
  src/ipc_bridge.cpp / ipc_bridge.hpp      UDP JSON IPC (send command, recv telemetry)
  src/aerpaw_platform_node.cpp             executable entry point
  aerpawlib_runner/aerpaw_as2_runner.py    Python BasicRunner (bridge API)
  config/control_modes.yaml                advertised control modes
  config/platform_params.yaml              ports, frequencies, takeoff altitude
  launch/aerpaw_platform.launch.py         single drone, platform only
  launch/aerpaw_sitl.launch.py             N drones on SITL or DT (platform only)
  tests/                                   gtests

as2_experiment_aerpaw_multiuav/            the 3-drone experiment
  missions/triangle_formation.py           AS2 Python API mission
  launch/sim_multirotor.launch.py          Config A
  launch/sim_gazebo.launch.py              Config B
  launch/aerpaw_digital_twin.launch.py     Config C
```

### 3.2 IPC contract

| Item          | Value                                                        |
|---------------|--------------------------------------------------------------|
| Transport     | local UDP, connectionless                                    |
| Framing       | one JSON object per datagram (no newline framing)            |
| Command port  | `ipc_cmd_port` (default 15760) — **the runner binds it**, C++ only sends |
| Telemetry port| `ipc_tel_port` (default 15761) — **the C++ bridge binds it**, runner sends |
| Per drone     | offset `i*2` -> drone0: 15760/15761, drone1: 15762/15763, ... |

### 3.3 Frames and headings

- AS2 works in the ENU odom frame; the base class converts commands to the odom
  frame (TF-based, so heading is already applied).
- ENU -> NED conversion is then a pure axis permutation:
  `north=+y`, `east=+x`, `down=-z`.
- ArduPilot heading (degrees from North, clockwise) from ENU yaw:
  `heading = fmod(90 - yaw_enu_deg, 360)`.
- POSITION targets are offsets from the platform's **fixed home**
  (`home_lat`/`home_lon` captured at first telemetry), so the mission is
  absolute and stable.

### 3.4 Control modes (`control_modes.yaml`)

| Mode                       | Value          | Notes                    |
|----------------------------|----------------|--------------------------|
| HOVER                      | `0b00010000`   |                          |
| SPEED (yaw speed, local FLU)| `0b01000100`  |                          |
| POSITION (yaw angle, ENU)  | `0b01100000`   | used by `go_to`          |

### 3.5 Telemetry topics

Published by the platform node in namespace `/droneN`; consumed by the state
estimator:

| Topic                                 | Type                     |
|---------------------------------------|--------------------------|
| `sensor_measurements/odom`            | `nav_msgs/Odometry`      |
| `sensor_measurements/imu`             | `sensor_msgs/Imu`        |
| `sensor_measurements/gps`             | `sensor_msgs/NavSatFix`  |

The estimator then publishes `/droneN/self_localization/*`.

### 3.6 Config parameters (`platform_params.yaml`)

```yaml
cmd_freq: 100.0          # Hz of the command loop
info_freq: 10.0          # Hz of platform info publish
ipc_cmd_port: 15760      # UDP command port
ipc_tel_port: 15761      # UDP telemetry port
takeoff_altitude: 25.0   # minimum 25 m on the AERPAW DT
```

### 3.7 The Python runner

`aerpaw_as2_runner.py` is a normal aerpawlib v2 `BasicRunner`. It exposes a
bridge API understood by the C++ node:

| C++ command (`cmd`) | Runner action |
|---------------------|---------------|
| `arm` / `disarm`    | `drone.set_armed(True/False)` |
| `takeoff`           | `drone.takeoff(altitude=...)` |
| `land`              | `drone.land()` |
| `set_velocity`      | `drone.set_velocity(VectorNED(vx, vy, vz))` |
| `goto_ned`          | `drone.goto_coordinates(Coordinate(home) + VectorNED(north, east, down), target_heading=...)` |
| `kill`              | `drone.set_armed(False)` + `stop_velocity()` (no true motor-stop; see Section 6) |

You normally never touch the runner — AS2 (through the C++ node) drives it.

### 3.8 Launches

- `aerpaw_platform.launch.py` — one platform node, raw (no stack, no mission).
  Useful for wiring checks.
- `aerpaw_sitl.launch.py` — N platform nodes + runners for SITL/DT.
- The experiment launches (Part 5) reuse the sitl launch and add the full stack.

---

## 4. Part 4 — Create your own AS2 experiment

### 4.1 Scaffold an ament_python package

```bash
mkdir -p as2_my_experiment/{launch,missions,resource}
cd as2_my_experiment
echo as2_my_experiment > resource/as2_my_experiment
```

`package.xml` (exec deps: `as2_python_api`, `as2_platform_aerpaw`, `rclpy`).
`setup.py` must list **every** launch/mission file in `data_files`, or the
installed share directory will miss them:

```python
from setuptools import setup
pkg = 'as2_my_experiment'
setup(
    name=pkg, version='1.1.3', packages=[], zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', ['launch/my_mission.launch.py']),
        ('share/' + pkg + '/missions', ['missions/my_mission.py']),
    ],
    install_requires=['setuptools'])
```

### 4.2 Write a mission with the AS2 Python API

`missions/my_mission.py` (synchronous `DroneInterface`):

```python
import rclpy
from as2_python_api.drone_interface import DroneInterface

def run_mission():
    rclpy.init()
    drone = DroneInterface("drone0", verbose=True)
    drone.arm()                                  # arm motors
    drone.takeoff(25.0, 3.0)                     # height, speed
    drone.offboard()                             # enable offboard control
    drone.go_to(50.0, 0.0, 25.0, speed=3.0)      # offsets in ENU near home
    drone.follow_path([[50, 50, 25], [0, 50, 25], [0, 0, 25]], 5)
    drone.land(0.5)
    drone.shutdown()

run_mission()
```

Useful `DroneInterface` methods: `arm`, `disarm`, `takeoff`, `land`, `offboard`,
`idle`, `go_to`, `follow_path`, `go_to.go_to`, and service-based behaviour calls.
For **multi-vehicle** work, instantiate one `DroneInterface("droneN")` per drone
and run them in parallel threads (calls are blocking) — see
`triangle_formation.py`.

> Frame caveat: coordinates follow the platform odom convention; offsets are
> relative to home (Section 3.3). On the Multirotor Simulator / SITL the origin
> is the drone's initial home.

### 4.3 Write the launch file (full-stack pattern)

Reuse the per-drone pattern from
`as2_experiment_aerpaw_multiuav/launch/aerpaw_digital_twin.launch.py`. Per
namespace `droneN`:

```
droneN/
  platform            (as2_platform_aerpaw_node)          <- the bridge
  state_estimator     (as2_state_estimator_node, plugin raw_odometry | ground_truth)
  motion_controller   (as2_motion_controller_node, plugin pid_speed)
  arm_behavior, offboard_behavior                        (as2_behaviors_platform)
  go_to_behavior_node, takeoff_behavior_node, land_behavior_node
                                                         (as2_behaviors_motion)
```

Then add `ExecuteProcess(["python3", "<installed share>/missions/my_mission.py"])`.
Keep a `use_sim_time` switch: `true` for Gazebo, `false` otherwise.

### 4.4 Build and install

```bash
cd /path/to/workspace
colcon build --packages-select as2_my_experiment
source install/setup.bash
```

Your experiment can now be launched exactly like the three reference configs
(Section 5), swapping the mission and the number of drones.

---

## 5. Part 5 — Run end-to-end

Build everything once:

```bash
colcon build --packages-up-to as2_experiment_aerpaw_multiuav
source install/setup.bash
```

### 5.1 Config A — AS2 Multirotor Simulator (no extra deps)

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

Expect: 3 platforms + estimators (raw_odometry) + controllers + behaviours; the
mission takes off to 25 m and flies a 50 m square.

### 5.2 Config B — Gazebo Harmonic

Requires `as2_gazebo_assets`, `as2_platform_gazebo` and the Gazebo Harmonic ROS
integration, plus a **3-drone simulation YAML**:

```bash
ros2 launch as2_experiment_aerpaw_multiuav sim_gazebo.launch.py \
    simulation_config_file:=/path/to/three_drone_world.yaml use_sim_time:=true
```

### 5.3 Config C — local SITL (same code path as the DT)

```bash
# aerpawlib + SITL installed (Part 1), then:
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=false
```

The launch starts 3 runners on the default SITL endpoints
`udpin://127.0.0.1:14550`, `...:14560`, `...:14570`. Point your own SITL
instances at those, or the runners wait for a MAVLink link.

### 5.4 Config C — AERPAW Digital Twin / testbed

1. Start your AERPAW experiment (Part 1) so each vehicle endpoint is up.
2. Grab the MAVLink connection string for **each** vehicle from the console /
   vehicle E-VM (Section 1.1).
3. Launch with the per-drone strings:

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=true \
    conn_drone0:=udpin://10.14.X.X:14550 \
    conn_drone1:=udpin://10.14.X.Y:14550 \
    conn_drone2:=udpin://10.14.X.Z:14550
```

`use_aerpaw:=true` omits `--no-aerpaw-environment`, so aerpawlib uses the
testbed profile and the 25 m geofence applies. The mission ends with `land()`.

### 5.5 Monitoring

```bash
ros2 run rviz2 rviz2
ros2 topic echo /drone0/sensor_measurements/odom
ros2 topic echo /drone0/self_localization/pose     # estimator output
ros2 topic list | grep drone0
```

Quick checks: `ros2 node list` (9+ nodes per drone namespace) and the mission
stdout (`[triangle] Flying to waypoint...`) in the launch terminal.

---

## 6. Troubleshooting and limitations

| Symptom | Likely cause / fix |
|---------|--------------------|
| `ModuleNotFoundError: aerpawlib` | Install in the *active* env: `pip install -e "aerpawlib[sitl,dev]"` |
| Runner starts then exits | Running outside the testbed with `use_aerpaw:=true`; use `use_aerpaw:=false` |
| Mission stuck at arming/takeoff (Config C) | No MAVLink link; start SITL/DT followers on the `conn_droneN` endpoints |
| `go_to` never reaches target / oscillates | State estimator not publishing `self_localization/*`; controller not `pid_speed` |
| POSITION targets jump or rotate | Wrong frame assumption; keep ENU odom (Section 3.3) |
| Telemetry empty (`sensor_measurements/*`) | Runner must bind `ipc_cmd_port`, C++ binds `ipc_tel_port`; ports offset +`i*2` per drone |
| Launch: file not found in `share/...` | Missing `data_files` entry in `setup.py` (Section 4.1) |
| `kill` doesn't stop motors instantly | Known limitation: no true AS2 emergency stop in aerpawlib |

**Known limitations**

- `kill` maps to disarm + stop-velocity — the strongest action aerpawlib offers;
  it is not an irreversible emergency motor-stop.
- aerpawlib v2 `set_velocity` is marked `[NOT SUPPORTED]` by the current copter
  filter — validate SPEED mode on the simulator before relying on it.
- AERPAW geofence mandates >= 25 m takeoff (default is already 25.0).
- Config B needs its own 3-drone Gazebo world YAML.
- The C++ build and gtests require a real ROS 2 Humble host (not a bare Python
  environment).

---

## 7. References

- Aerostack2: https://aerostack2.github.io (wiki + tutorials)
- This work: the repository `docs/` directory — `docs/README.md`,
  `docs/run_and_test_requirements.txt`, and the package READMEs
- aerpawlib: https://github.com/AERPAW/aerpawlib · API: https://aerpaw.github.io/aerpawlib
- AERPAW portal: https://console.aerpaw.org · user manual:
  https://sites.google.com/ncsu.edu/aerpaw-user-manual/