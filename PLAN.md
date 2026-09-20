# OpenCode Agent Instructions: AeroStack2 AERPAW Platform (`as2_platform_aerpaw`)

## What you are building

An AeroStack2 **aerial platform** package called `as2_platform_aerpaw` that bridges the AS2
framework to **aerpawlib** (AERPAW's Python vehicle control library, which drives ArduPilot
vehicles over MAVLink). Once built, any AS2 mission — written in Python or C++ — can run
unchanged on:

1. AS2's built-in **Multirotor Simulator** (existing platform, used as a baseline)
2. **Gazebo** (existing platform, used as a baseline)
3. The **AERPAW Digital Twin** via `as2_platform_aerpaw` (what you are building)
4. Real AERPAW hardware via the same platform

You will also write a multi-UAV test experiment that validates all three targets.

---

## Step 0 — Read the source before writing any code

**This step is mandatory. Do not skip it.**

### 0a. AS2 Aerial Platform API

The official tutorial lives at:
https://aerostack2.github.io/development/tutorials/aerial_platform/index.html

The base class is `as2::AerialPlatform` (a `rclcpp::Node` subclass) in:
`aerostack2/as2_core/include/as2_core/aerial_platform.hpp`

Every platform **must** override these virtual methods:

| Method | Description |
|---|---|
| `configureSensors()` | Declare/setup platform-side sensor publishers. Called once by base after construction. |
| `ownSetArmingState(bool)` | Arm or disarm the vehicle. |
| `ownSetOffboardControl(bool)` | Enter/exit offboard (guided) mode. |
| `ownSetPlatformControlMode(const as2_msgs::msg::ControlMode&)` | Accept and store the active control mode. |
| `ownSendCommand()` | Translate the current command msg into a hardware command and send it. |
| `ownKillSwitch()` | Immediate motor stop (irreversible). |
| `ownStopPlatform()` | Emergency hover (stop obeying commands, hold position best-effort). |
| `ownTakeoff()` | (Optional) Hardware-assisted takeoff. Return false if unsupported. |
| `ownLand()` | (Optional) Hardware-assisted land. Return false if unsupported. |

The base class provides these **protected attributes** that arrive pre-filled before
`ownSendCommand()` is called:

| Attribute | Type | Source topic |
|---|---|---|
| `command_pose_msg_` | `geometry_msgs::msg::PoseStamped` | `actuator_command/pose` |
| `command_twist_msg_` | `geometry_msgs::msg::TwistStamped` | `actuator_command/twist` |
| `command_thrust_msg_` | `as2_msgs::msg::Thrust` | `actuator_command/thrust` |
| `platform_info_msg_` | `as2_msgs::msg::PlatformInfo` | (read-only, internal state) |

Study the **Gazebo platform** implementation as your primary reference — it is the simplest
complete example and is covered in the official tutorial. Then also read `as2_platform_mavlink`
(ArduPilot/MAVROS back-end, architecturally closest to what you are building).

The control mode bitmask encoding is in `control_modes.yaml` next to each platform. You will
need to pick the modes that aerpawlib can support and list only those. Example (from Gazebo):

```yaml
available_modes:
    - 0b00010000  # HOVER
    - 0b01000100  # SPEED with yaw SPEED in LOCAL_FLU_FRAME
```

### 0b. aerpawlib

```bash
git clone https://github.com/aerpaw/aerpawlib.git
pip install -e "aerpawlib[sitl,dev]"
aerpawlib-setup-sitl   # builds ArduPilot SITL binary (first time only)
```

Key things to read in the `aerpawlib/` Python package:

- **`v2/vehicle.py`** — `Drone` class: `takeoff(altitude)`, `land()`,
  `goto_coordinates(coord)`, `set_velocity(vned)`, `.position` (a `Coordinate`),
  `.velocity`, `.heading`, `.armed`
- **`v2/runner.py`** — `BasicRunner`, `StatefulRunner`, `@entrypoint` decorator
- **`v2/util.py`** — `VectorNED`, `Coordinate` types
- **`zmq_helper.py`** — multi-vehicle ZMQ proxy (`--zmq-proxy-server` / `--zmq-identifier`)
- **`examples/`** — especially multi-vehicle ZMQ examples
- **`configs/`** — JSON config files used with `--config`

CLI usage reminder:
```bash
# Single drone, local SITL
aerpawlib --api-version v2 --script my_mission.py \
  --conn udpin://127.0.0.1:14550 --vehicle drone --no-aerpaw-environment

# Multi-drone: start proxy first, then each drone with --zmq-identifier
aerpawlib-run-proxy
aerpawlib --api-version v2 --script multi_mission.py \
  --conn udpin://127.0.0.1:14550 --vehicle drone \
  --zmq-identifier drone0 --zmq-proxy-server localhost \
  --no-aerpaw-environment
```

---

## Step 1 — Repository setup

```bash
# Fork https://github.com/aerostack2/aerostack2 on GitHub, then:
git clone https://github.com/<YOUR_ORG>/aerostack2.git
cd aerostack2
git remote add upstream https://github.com/aerostack2/aerostack2.git

# Create the new platform package inside the fork
ros2 pkg create as2_platform_aerpaw \
  --build-type ament_cmake \
  --dependencies rclcpp as2_core as2_msgs geometry_msgs nav_msgs sensor_msgs \
  --destination-directory .

# Clone aerpawlib alongside (outside the ROS workspace)
cd ..
git clone https://github.com/aerpaw/aerpawlib.git
pip install -e "aerpawlib[sitl,dev]"
aerpawlib-setup-sitl
```

---

## Step 2 — IPC bridge design

The AS2 platform node is **C++ / ROS2**. aerpawlib is **Python / asyncio**. Keep them in
separate processes connected by a thin IPC layer — do not embed Python in the C++ process.

**Recommended IPC:** a pair of localhost UDP sockets (one per direction) using newline-
delimited JSON. The C++ node binds; the Python runner connects.

**Commands (C++ → Python), one JSON line per command:**
```json
{"cmd": "arm"}
{"cmd": "disarm"}
{"cmd": "takeoff", "alt": 25.0}
{"cmd": "land"}
{"cmd": "set_velocity", "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0}
{"cmd": "goto_ned", "north": 0.0, "east": 0.0, "down": -25.0, "heading": 0.0}
{"cmd": "kill"}
{"cmd": "stop"}
```

**Telemetry (Python → C++), streamed at ~20 Hz:**
```json
{
  "lat": 35.727, "lon": -78.698, "alt_msl": 85.0, "rel_alt": 25.1,
  "vx_ned": 0.1, "vy_ned": 0.0, "vz_ned": 0.0,
  "roll": 0.0, "pitch": 0.01, "yaw": 1.57,
  "armed": true, "mode": "GUIDED", "ts": 1234567890.123
}
```

---

## Step 3 — Package structure

```
as2_platform_aerpaw/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── control_modes.yaml          # supported AS2 control modes
│   └── platform_params.yaml        # IPC ports, timeouts
├── include/as2_platform_aerpaw/
│   ├── aerpaw_platform.hpp         # AerpawPlatform class declaration
│   └── ipc_bridge.hpp              # UDP bridge to Python runner
├── src/
│   ├── aerpaw_platform.cpp
│   ├── ipc_bridge.cpp
│   └── main.cpp
├── aerpawlib_runner/
│   └── aerpaw_as2_runner.py        # aerpawlib script that speaks IPC
├── launch/
│   ├── aerpaw_platform.launch.py   # single drone
│   └── aerpaw_sitl.launch.py       # spawns SITL + platform for N drones
└── README.md
```

---

## Step 4 — Implement `AerpawPlatform`

### 4a. Header (`aerpaw_platform.hpp`)

```cpp
#pragma once
#include <as2_core/aerial_platform.hpp>
#include <as2_msgs/msg/control_mode.hpp>
#include <memory>
#include "ipc_bridge.hpp"

class AerpawPlatform : public as2::AerialPlatform {
public:
  explicit AerpawPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // --- Mandatory overrides (as2::AerialPlatform) ---
  void configureSensors() override;
  bool ownSetArmingState(bool arm) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & mode) override;
  bool ownSendCommand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

  // --- Optional overrides ---
  bool ownTakeoff() override;   // delegate to aerpawlib
  bool ownLand() override;      // delegate to aerpawlib

private:
  std::unique_ptr<IpcBridge> bridge_;
  as2_msgs::msg::ControlMode control_in_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  void publishTelemetry();  // reads bridge state → publishes odom, imu topics
};
```

### 4b. Constructor

```cpp
AerpawPlatform::AerpawPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  this->declare_parameter<int>("ipc_cmd_port", 15760);
  this->declare_parameter<int>("ipc_tel_port", 15761);

  int cmd_port = this->get_parameter("ipc_cmd_port").as_int();
  int tel_port = this->get_parameter("ipc_tel_port").as_int();

  bridge_ = std::make_unique<IpcBridge>(cmd_port, tel_port);
  bridge_->start();  // binds sockets, waits for Python runner to connect

  // Publish telemetry at 20 Hz
  telemetry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&AerpawPlatform::publishTelemetry, this));
}
```

### 4c. Implement each override

Follow the Gazebo platform pattern exactly. Key points:

**`configureSensors()`** — for now, no extra sensors beyond what the base class handles:
```cpp
void AerpawPlatform::configureSensors() {
  // No additional sensors configured here.
  // Odometry and IMU are published in publishTelemetry().
}
```

**`ownSetArmingState(bool arm)`** — send arm/disarm command over IPC:
```cpp
bool AerpawPlatform::ownSetArmingState(bool arm) {
  nlohmann::json cmd;
  cmd["cmd"] = arm ? "arm" : "disarm";
  return bridge_->sendCommand(cmd);
}
```

**`ownSetOffboardControl(bool offboard)`** — aerpawlib handles offboard internally when
`goto_coordinates` / `set_velocity` are called, so simply return true:
```cpp
bool AerpawPlatform::ownSetOffboardControl(bool offboard) {
  return true;  // aerpawlib manages GUIDED mode internally
}
```

**`ownSetPlatformControlMode()`** — store and acknowledge (validation is done by base class):
```cpp
bool AerpawPlatform::ownSetPlatformControlMode(
  const as2_msgs::msg::ControlMode & mode)
{
  control_in_ = mode;
  return true;
}
```

**`ownSendCommand()`** — translate AS2 control mode → IPC command. Mirror Gazebo pattern:
```cpp
bool AerpawPlatform::ownSendCommand() {
  if (control_in_.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    nlohmann::json cmd;
    cmd["cmd"]      = "set_velocity";
    cmd["vx"] = cmd["vy"] = cmd["vz"] = cmd["yaw_rate"] = 0.0;
    return bridge_->sendCommand(cmd);
  }

  if (control_in_.control_mode == as2_msgs::msg::ControlMode::SPEED) {
    // command_twist_msg_ is in LOCAL_FLU frame; convert to NED for aerpawlib
    // FLU→NED: x_ned=+y_flu (forward), y_ned=+x_flu, z_ned=-z_flu
    nlohmann::json cmd;
    cmd["cmd"]      = "set_velocity";
    cmd["vx"]       =  command_twist_msg_.twist.linear.y;
    cmd["vy"]       =  command_twist_msg_.twist.linear.x;
    cmd["vz"]       = -command_twist_msg_.twist.linear.z;
    cmd["yaw_rate"] =  command_twist_msg_.twist.angular.z;
    return bridge_->sendCommand(cmd);
  }

  if (control_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    // command_pose_msg_ in ENU/global frame; convert to NED for aerpawlib
    // ENU→NED for position: north=+y_enu, east=+x_enu, down=-z_enu
    nlohmann::json cmd;
    cmd["cmd"]     = "goto_ned";
    cmd["north"]   =  command_pose_msg_.pose.position.y;
    cmd["east"]    =  command_pose_msg_.pose.position.x;
    cmd["down"]    = -command_pose_msg_.pose.position.z;
    // derive heading from pose quaternion or set to 0
    cmd["heading"] = 0.0;
    return bridge_->sendCommand(cmd);
  }

  RCLCPP_WARN(this->get_logger(), "Unsupported control mode — command ignored");
  return false;
}
```

> **Frame note:** Verify the exact frame AS2 uses for each control mode against
> `as2_platform_mavlink`'s `ownSendCommand()`. The FLU→NED transform above is a starting
> point; adjust if the behaviour controller publishes in a different frame.

**`ownKillSwitch()`** and **`ownStopPlatform()`**:
```cpp
void AerpawPlatform::ownKillSwitch() {
  bridge_->sendCommand({{"cmd", "kill"}});
}

void AerpawPlatform::ownStopPlatform() {
  // Hover: send zero velocity (same as Gazebo platform pattern)
  nlohmann::json cmd;
  cmd["cmd"] = "set_velocity";
  cmd["vx"] = cmd["vy"] = cmd["vz"] = cmd["yaw_rate"] = 0.0;
  bridge_->sendCommand(cmd);
}
```

**`ownTakeoff()` and `ownLand()`** — delegate to aerpawlib (it supports them natively):
```cpp
bool AerpawPlatform::ownTakeoff() {
  nlohmann::json cmd;
  cmd["cmd"] = "takeoff";
  cmd["alt"] = 25.0;  // minimum AERPAW altitude
  return bridge_->sendCommand(cmd);
}

bool AerpawPlatform::ownLand() {
  return bridge_->sendCommand({{"cmd", "land"}});
}
```

### 4d. `publishTelemetry()`

Read the latest telemetry snapshot from `IpcBridge`, then publish:
- `nav_msgs::msg::Odometry` on `self_localization/odom`
- `sensor_msgs::msg::Imu` on `sensor_measurements/imu`

Use `as2::sensors::Odom` and `as2::sensors::Imu` helpers from `as2_core` if available,
otherwise publish directly. Convert lat/lon/alt → local ENU using the home position as origin
(store home at first telemetry packet when `armed=false`).

---

## Step 5 — `aerpaw_as2_runner.py`

This Python script is the aerpawlib side of the bridge. It must:

1. Connect to the C++ IPC sockets (one for receiving commands, one for sending telemetry)
2. Use aerpawlib v2 API to control a `Drone` object
3. Receive command JSON → call the appropriate `Drone` method
4. Stream telemetry JSON at 20 Hz

```python
#!/usr/bin/env python3
"""
aerpaw_as2_runner.py

Aerpawlib runner that bridges to the AS2 AERPAW platform node via UDP/JSON IPC.
Launch via as2_platform_aerpaw launch files, or manually:

  python aerpaw_as2_runner.py \
    --conn udpin://127.0.0.1:14550 \
    --cmd-port 15760 --tel-port 15761 \
    --no-aerpaw-environment
"""
import asyncio, json, socket, argparse, time
from aerpawlib.v2 import Drone, VectorNED, BasicRunner, entrypoint

CMD_PORT = 15760
TEL_PORT = 15761

class AS2Runner(BasicRunner):
    def __init__(self, cmd_port: int, tel_port: int):
        super().__init__()
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind(("127.0.0.1", cmd_port))
        self._cmd_sock.setblocking(False)

        self._tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tel_target = ("127.0.0.1", tel_port)

    async def _recv_cmd(self) -> dict | None:
        loop = asyncio.get_event_loop()
        try:
            data, _ = await loop.run_in_executor(None, self._cmd_sock.recvfrom, 4096)
            return json.loads(data.decode())
        except BlockingIOError:
            return None

    async def _stream_telemetry(self, drone: Drone):
        while True:
            pos = drone.position
            tel = {
                "lat": pos.lat, "lon": pos.lon,
                "alt_msl": pos.alt, "rel_alt": pos.rel_alt,
                "vx_ned": drone.velocity.x,
                "vy_ned": drone.velocity.y,
                "vz_ned": drone.velocity.z,
                "roll": drone.attitude.roll,
                "pitch": drone.attitude.pitch,
                "yaw": drone.attitude.yaw,
                "armed": drone.armed,
                "mode": drone.mode,
                "ts": time.time(),
            }
            self._tel_sock.sendto(json.dumps(tel).encode(), self._tel_target)
            await asyncio.sleep(0.05)  # 20 Hz

    @entrypoint
    async def run(self, drone: Drone):
        tel_task = asyncio.create_task(self._stream_telemetry(drone))
        try:
            while True:
                cmd = await self._recv_cmd()
                if cmd is None:
                    await asyncio.sleep(0.01)
                    continue

                match cmd.get("cmd"):
                    case "arm":
                        await drone.set_armed(True)
                    case "disarm":
                        await drone.set_armed(False)
                    case "takeoff":
                        await drone.takeoff(altitude=cmd.get("alt", 25.0))
                    case "land":
                        await drone.land()
                    case "set_velocity":
                        v = VectorNED(cmd["vx"], cmd["vy"], cmd["vz"])
                        await drone.set_velocity(v, cmd.get("yaw_rate", 0.0))
                    case "goto_ned":
                        target = drone.position + VectorNED(
                            cmd["north"], cmd["east"], cmd["down"])
                        await drone.goto_coordinates(target)
                    case "kill":
                        await drone.set_armed(False)
                    case "stop":
                        await drone.set_velocity(VectorNED(0, 0, 0), 0.0)
        finally:
            tel_task.cancel()
```

> **Important:** Check actual aerpawlib v2 `Drone` API against the cloned source before
> finalising method names (e.g. `drone.attitude`, `drone.velocity`, `drone.set_armed` may
> differ). Adjust to match whatever the actual API exposes.

---

## Step 6 — `control_modes.yaml`

Choose the modes that aerpawlib can actually support. Start conservatively:

```yaml
available_modes:
    - 0b00010000  # HOVER
    - 0b01000100  # SPEED with yaw SPEED in LOCAL_FLU_FRAME
    - 0b01100001  # POSITION with yaw ANGLE in GLOBAL_ENU_FRAME
```

Add TRAJECTORY mode only if you implement trajectory tracking on top of `set_velocity`.

---

## Step 7 — Launch files

### `aerpaw_platform.launch.py` (single drone)

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns          = LaunchConfiguration("drone_id",   default="drone0")
    conn        = LaunchConfiguration("conn",        default="udpin://127.0.0.1:14550")
    use_aerpaw  = LaunchConfiguration("use_aerpaw", default="false")
    cmd_port    = LaunchConfiguration("cmd_port",   default="15760")
    tel_port    = LaunchConfiguration("tel_port",   default="15761")

    platform_node = Node(
        package="as2_platform_aerpaw",
        executable="as2_platform_aerpaw_node",
        namespace=ns,
        parameters=[{
            "ipc_cmd_port": cmd_port,
            "ipc_tel_port": tel_port,
            "control_modes_file": "$(find-pkg-share as2_platform_aerpaw)/config/control_modes.yaml",
        }],
    )

    no_aerpaw_flag = [] if use_aerpaw == "true" else ["--no-aerpaw-environment"]

    runner = ExecuteProcess(
        cmd=[
            "python3",
            "$(find-pkg-share as2_platform_aerpaw)/aerpawlib_runner/aerpaw_as2_runner.py",
            "--api-version", "v2",
            "--conn", conn,
            "--vehicle", "drone",
            "--cmd-port", cmd_port,
            "--tel-port", tel_port,
        ] + no_aerpaw_flag,
        output="screen",
    )

    return LaunchDescription([platform_node, runner])
```

### `aerpaw_sitl.launch.py` (N drones on SITL)

Parameterised by `num_drones` (default 3). For drone index `i`:
- ArduPilot SITL on ports `14550 + i*10`
- IPC cmd port `15760 + i*2`, tel port `15761 + i*2`
- ROS namespace `drone{i}`

Use `ExecuteProcess` to spawn each SITL instance and `OpaqueFunction` to generate N copies
of the platform node + runner.

---

## Step 8 — Multi-UAV test experiment

Create a second package: `as2_experiment_aerpaw_multiuav/`

```
as2_experiment_aerpaw_multiuav/
├── CMakeLists.txt  (or setup.py if pure Python)
├── package.xml
├── missions/
│   └── triangle_formation.py       # the mission script
├── launch/
│   ├── sim_multirotor.launch.py    # Config A: AS2 Multirotor Simulator
│   ├── sim_gazebo.launch.py        # Config B: Gazebo
│   └── aerpaw_digital_twin.launch.py  # Config C: AERPAW Digital Twin / SITL
└── README.md
```

### Mission: triangle formation square (`triangle_formation.py`)

Use the AS2 Python API (`as2_python_api`). Three drones: `drone0` (leader), `drone1`,
`drone2` (followers at ±10 m lateral offset from leader).

```python
"""
Triangle formation: leader flies a 50 m square at 25 m altitude.
Followers maintain a fixed NED lateral offset from the leader using velocity control.
"""
import asyncio
import rclpy
from as2_python_api.drone_interface import DroneInterface

ALTITUDE   = 25.0   # m (≥ 20 m required on AERPAW testbed)
SIDE       = 50.0   # m, square side length
SPEED      = 3.0    # m/s
OFFSETS    = [(0, 0), (10, 0), (-10, 0)]  # NE offsets per drone (metres)

async def run():
    rclpy.init()
    drones = [DroneInterface(f"drone{i}", verbose=True) for i in range(3)]

    # Arm and takeoff in parallel
    await asyncio.gather(*[d.arm() for d in drones])
    await asyncio.gather(*[
        d.takeoff.takeoff(height=ALTITUDE, speed=1.5) for d in drones
    ])

    # Square waypoints (ENU, relative to home)
    square = [(SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]

    for wp_north, wp_east in square:
        tasks = []
        for i, drone in enumerate(drones):
            off_n, off_e = OFFSETS[i]
            tasks.append(
                drone.go_to.go_to_point(
                    wp_north + off_n, wp_east + off_e, ALTITUDE,
                    speed=SPEED
                )
            )
        await asyncio.gather(*tasks)

    # Land in parallel
    await asyncio.gather(*[d.land.land(speed=0.5) for d in drones])
    await asyncio.gather(*[d.disarm() for d in drones])

    for d in drones:
        d.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(run())
```

> Adapt to the actual `as2_python_api` interface after reading its source. The pattern above
> follows the AS2 RSS24 demo; method names may differ slightly.

### Launch config A — Multirotor Simulator

Use the existing `as2_platform_multirotor_simulator`. No aerpawlib involved. Spin up 3 drone
instances with the Multirotor Simulator platform, then run the mission script.

### Launch config B — Gazebo

Use `as2_platform_gazebo` with Gazebo Harmonic. Spawn three drone models from
`as2_gazebo_assets`, then run the mission script.

### Launch config C — AERPAW Digital Twin / SITL

```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
  conn_drone0:=udpin://10.14.X.X:14550 \
  conn_drone1:=udpin://10.14.X.Y:14550 \
  conn_drone2:=udpin://10.14.X.Z:14550 \
  use_aerpaw:=true
```

For SITL validation (before real DT access):
```bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
  conn_drone0:=udpin://127.0.0.1:14550 \
  conn_drone1:=udpin://127.0.0.1:14560 \
  conn_drone2:=udpin://127.0.0.1:14570 \
  use_aerpaw:=false
```

---

## Step 9 — Tests

### Unit tests (`as2_platform_aerpaw/tests/`)

- IPC bridge: serialize every command type, parse back, verify round-trip
- Control mode mapping: mock `IpcBridge`, call `ownSendCommand()` for each supported mode,
  assert correct JSON command is produced (including FLU→NED frame conversion)
- Telemetry: feed a mock telemetry packet into `publishTelemetry()`, assert correct ROS
  message fields

```bash
colcon test --packages-select as2_platform_aerpaw
```

### SITL integration test

```bash
# Terminal 1: start 3 ArduPilot SITL instances
# (aerpawlib-setup-sitl creates the binary; run three with port offsets)
ardupilot-sitl --instance 0 --home 35.727,-78.698,60,0
ardupilot-sitl --instance 1 --home 35.727,-78.688,60,0
ardupilot-sitl --instance 2 --home 35.727,-78.678,60,0

# Terminal 2: run Config C pointing at SITL
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
  conn_drone0:=udpin://127.0.0.1:14550 \
  conn_drone1:=udpin://127.0.0.1:14560 \
  conn_drone2:=udpin://127.0.0.1:14570 \
  use_aerpaw:=false

# Terminal 3: run the mission
python3 missions/triangle_formation.py
```

Validation: all three drones arm, take off to 25 m, complete the square in formation, land.
Monitor with:
```bash
ros2 topic echo /drone0/self_localization/pose
ros2 run rviz2 rviz2  # add TF and Odometry displays per drone namespace
```

---

## Constraints and notes

- **ROS 2 distro:** Humble on Ubuntu 22.04. Do not use Galactic-only APIs.
- **aerpawlib API version:** always use `--api-version v2`.
- **Minimum takeoff altitude:** 25 m in all configs (AERPAW testbed requires ≥ 20 m).
- **Multi-vehicle ZMQ:** use `--zmq-proxy-server` + `--zmq-identifier droneN` when running
  on AERPAW Digital Twin with multiple drones coordinating via aerpawlib's ZMQ mechanism.
- **No modifications to upstream AS2 packages** — implement everything in the new packages
  `as2_platform_aerpaw` and `as2_experiment_aerpaw_multiuav`.
- **Code style:** `ament_clang_format` for C++, `ament_flake8` / `ament_pep8` for Python.
  CI workflow configs are in `.github/workflows/` of the upstream repo — replicate them.
- **Frame conventions:** AS2 uses FLU (Forward-Left-Up) body frame and ENU global frame.
  aerpawlib uses NED. All frame conversions belong in `ownSendCommand()` and
  `publishTelemetry()`.

---

## Deliverables checklist

- [ ] `as2_platform_aerpaw/` builds cleanly: `colcon build --packages-select as2_platform_aerpaw`
- [ ] `aerpaw_as2_runner.py` connects to aerpawlib, responds to all 8 command types
- [ ] IPC bridge unit tests passing
- [ ] `ownSendCommand()` frame conversion verified (FLU→NED for SPEED, ENU→NED for POSITION)
- [ ] Platform publishes `/droneX/self_localization/odom` and `/droneX/sensor_measurements/imu`
- [ ] `as2_experiment_aerpaw_multiuav/` builds and all three launch configs work
- [ ] Config A (Multirotor Simulator): 3-drone formation square completes successfully
- [ ] Config B (Gazebo): 3-drone formation square completes successfully
- [ ] Config C (SITL or Digital Twin): 3-drone formation square completes successfully
- [ ] `README.md` in each package covering build, launch args, and troubleshooting