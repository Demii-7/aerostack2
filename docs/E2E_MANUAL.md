# End‑to‑End Manual — AERPAW + AeroStack2

From a fresh machine to a flying multi‑UAV experiment through the
`as2_platform_aerpaw` bridge.

**Scope of the doc set** (this file is the *how‑to*; details live in their owner):
- Prerequisites → [`run_and_test_requirements.txt`](run_and_test_requirements.txt)
- Bridge internals (IPC, frames, topics, modes, params) → [`as2_platform_aerpaw/README.md`](../as2_aerial_platforms/as2_platform_aerpaw/README.md)
- The example mission + **per‑config run commands** → [`as2_experiment_aerpaw_multiuav/README.md`](../as2_experiment_aerpaw_multiuav/README.md)
- **AERPAW VM bootstrap + QGC tunnel scripts** → [`deploy/`](../as2_aerial_platforms/as2_platform_aerpaw/deploy/README.md) (`cvm_setup.sh`, `qgc_tunnel.sh`)

---

## 0. Quick Start

On Ubuntu 22.04 with ROS 2 Humble already sourced (see
[`run_and_test_requirements.txt`](run_and_test_requirements.txt) for everything needed):

```bash
colcon build --packages-up-to as2_platform_aerpaw   # build + AS2 deps
source install/setup.bash
ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py num_drones:=3
```

That flies the 3‑drone mission on the Multirotor Simulator (no extra deps). For Gazebo,
the AERPAW Digital Twin, or SITL, use the run commands in the
[experiment README](../as2_experiment_aerpaw_multiuav/README.md) after the setup below.

---

## 1. How it works (30 seconds)

AS2 commands an `as2::AerialPlatform` in the ENU odom frame. This platform is the
`as2_platform_aerpaw` C++ node: it serialises each command as one JSON datagram over a
localhost UDP socket to a Python **aerpawlib runner**, which drives an ArduPilot vehicle
(Real DT / SITL / Gazebo) via MAVLink and streams telemetry back. The platform republishes
raw `sensor_measurements/*`; the AS2 state estimator turns those into `self_localization/*`.
Because only the platform changes, **the same mission runs on every backend**.

Full contract (ports, frames, topics, control modes, runner command map) is the
[platform README](../as2_aerial_platforms/as2_platform_aerpaw/README.md).

---

## 2. Part 1 — Set up the AERPAW side

### 2.1 AERPAW access (real testbed / Digital Twin)

1. Request access — console: https://console.aerpaw.org · user manual:
   https://sites.google.com/ncsu.edu/aerpaw-user-manual/ (try "Hello AERPAW!").
2. Have the PI create an **experiment** and add you as an experimenter.
3. Reserve nodes: **portable node(s)** = vehicles; fixed nodes = radio sites (not
   needed for pure flight).
4. Note each vehicle's MAVLink endpoint (real testbed ≈ `192.168.32.X:14550`;
   Digital Twin uses the **virtual** E‑VM addresses, e.g. `10.14.X.X:14550`).
   These become `conn_droneN` in the run commands.

### 2.2 aerpawlib + local SITL (development)

```bash
git clone https://github.com/AERPAW/aerpawlib.git
cd aerpawlib && pip install -e "aerpawlib[sitl,dev]"
aerpawlib-setup-sitl        # first time only: builds ArduPilot SITL
```

### 2.3 Prove SITL + aerpawlib before touching AS2

Start a Copter SITL sending to `udp:127.0.0.1:14550`, then fly it with a bare runner:

```python
# /tmp/hello.py
from aerpawlib.v2 import Drone, VectorNED, BasicRunner, entrypoint

class Hello(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=25)          # 25 m minimum on AERPAW
        await drone.goto_coordinates(drone.position + VectorNED(10, 0))
        await drone.land()
```

```bash
aerpawlib --api-version v2 --script /tmp/hello.py \
          --conn udpin://127.0.0.1:14550 --vehicle drone --no-aerpaw-environment
```

`--no-aerpaw-environment` is required **outside** the testbed. API: https://aerpaw.github.io/aerpawlib

---

## 3. Part 2 — Set up and build Aerostack2

### 3.1 ROS 2 Humble (pick one)

- **apt** (recommended): https://docs.ros.org/en/humble/Installation.html
- **conda / robostack**: `conda create -n humble ros-humble-desktop colcon-common-extensions cmake nlohmann_json gtest -c robostack-humble -c conda-forge`
- **pixi** (repo `pixi.toml`, `humble` env): `pixi install`

### 3.2 Build the workspace

```bash
colcon build --packages-up-to as2_platform_aerpaw   # pulls as2_msgs, as2_core, as2_python_api…
source install/setup.bash
```

### 3.3 Unit‑test the platform (no simulator, no aerpawlib)

```bash
colcon test --packages-select as2_platform_aerpaw
colcon test-result --verbose
```

Runs `ipc_bridge_gtest` (UDP JSON round‑trip) and `aerpaw_platform_gtest` (node builds
with the shipped configs).

---

## 4. Part 3 — Create your own AS2 experiment

1. **Scaffold** an `ament_python` package; `package.xml` exec‑deps: `as2_python_api`,
   `as2_platform_aerpaw`, `rclpy`. In `setup.py` list **every** launch/mission file under
   `data_files` (`<share>/launch`, `<share>/missions`), else the installed share tree
   misses them.
2. **Mission** with the synchronous `DroneInterface` API — one instance per drone
   (`"droneN"`) run in parallel threads, since calls block. See
   `../as2_experiment_aerpaw_multiuav/missions/triangle_formation.py` for the pattern.
3. **Launch** the full stack per namespace (`platform`, `state_estimator`,
   `motion_controller`, arm/offboard + go_to/takeoff/land behaviours), then
   `ExecuteProcess(["python3", "<share>/missions/my_mission.py"])`. Keep a `use_sim_time`
   switch (`true` for Gazebo). Copy `../as2_experiment_aerpaw_multiuav/launch/` as a base.
4. **Build** `colcon build --packages-select as2_my_experiment`.

Reference bridge args/config live in the
[platform README](../as2_aerial_platforms/as2_platform_aerpaw/README.md).

---

## 5. Part 4 — Run it end‑to‑end

Use the copy‑paste commands in the
[experiment README](../as2_experiment_aerpaw_multiuav/README.md) (Config A / B / C, SITL
and Digital Twin). What to expect:

- **A — Multirotor Simulator:** 3 platforms + raw_odometry estimator + `pid_speed`
  controller + behaviours; takeoff to 25 m and the 50 m square.
- **B — Gazebo:** needs `as2_gazebo_assets` + `as2_platform_gazebo` + **Gazebo
  Harmonic** and a 3‑drone world YAML (an example ships at
  `../as2_experiment_aerpaw_multiuav/config/three_drone_world.yaml`). On Humble's
  Fortress the platform's SPEED mode is unsupported — run B on **ROS 2 Jazzy**.
- **C — SITL / Digital Twin:** one runner + MAVLink endpoint per drone; watch
  `/droneN/self_localization/pose` stream, then the mission runs.

---

## 6. Troubleshooting & limitations

| Symptom | Cause / fix |
|---------|-------------|
| `ModuleNotFoundError: aerpawlib` | Install into the **active** env (§2.2). |
| Runner starts then exits | Using `use_aerpaw:=true` outside the testbed → set `false`. |
| Config C stuck at arm/takeoff | No MAVLink link — start SITL/DT on the `conn_droneN` endpoints. |
| `go_to` never reaches / oscillates | No `self_localization/*` (estimator down) or controller not `pid_speed`. |
| Empty `sensor_measurements/*` | IPC port mismatch (see platform README); per‑drone offset `+i*2`. |
| Launch: file not found in `share/` | Missing `data_files` entry in `setup.py` (§4.1). |

**Limitations**

- `kill` = disarm + stop‑velocity (aerpawlib has no irreversible motor‑stop).
- aerpawlib v2 `set_velocity` is `[NOT SUPPORTED]` by the current copter filter — validate
  SPEED mode on Config A before relying on it (affects Config C velocity).
- AERPAW mandates ≥ 25 m takeoff (default already `takeoff_altitude: 25.0`).
- Config B targets **Gazebo Harmonic** (ROS 2 Jazzy); Humble ships Fortress.

---

## 7. References

- Aerostack2: https://aerostack2.github.io · aerpawlib: https://github.com/AERPAW/aerpawlib
  (API https://aerpaw.github.io/aerpawlib) · AERPAW console: https://console.aerpaw.org
- Docs index: [`README.md`](README.md) · requirements: [`run_and_test_requirements.txt`](run_and_test_requirements.txt)
