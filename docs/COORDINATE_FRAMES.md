# Coordinate Frames & Conventions — AERPAW ↔ AeroStack2

> Stage 7 developer reference. The frames used by AERPAW, the UAV/autopilot, ROS 2
> and AeroStack2 are **not identical**; this page records every convention (axes,
> units, reference, orientation, velocity, yaw) and the single place the
> conversions live. Read this before touching any position/attitude/velocity math.

## 1. Golden rule

**Never assume two frames match. Every conversion goes through
`as2_platform_aerpaw/include/as2_platform_aerpaw/frame_conversions.hpp`** (pure,
no ROS/socket deps, unit-tested). Conversion math must not be inlined in the
platform node, behaviours, or launch — if you need a transform, add/reuse a named
function there. The runner (`aerpawlib_runner/aerpaw_as2_runner.py`) has the two
tiny AERPAW-side helpers (`heading_deg_from_yaw_ned_rad`,
`alt_msl_from_relative`, `goto_target_from_ned`) and otherwise relies on
aerpawlib's own `Coordinate`/`VectorNED` arithmetic; it is deliberately *not* a
second copy of the transform rules.

```
AERPAW UAV ─(MAVLink NED/FRD)→ aerpawlib Drone ─(UDP JSON)→ [ adapter ] ─(ROS 2 ENU/FLU)→ AeroStack2
                                                            frame_conversions.hpp
                                              ← the single conversion boundary →
```

## 2. Per-system conventions

| System | Position / world frame | Axes | Body frame | Orientation | Velocity | Yaw / heading | Units |
|---|---|---|---|---|---|---|---|
| **ArduPilot / MAVLink** (autopilot on the UAV) | global frame = **NED**, local = **FRD** | x=forward, y=right, z=**down** | FRD | Euler roll/pitch/yaw = rotation **about NED axes**, yaw = **heading** from North, **CW positive**; MAVSDK `attitude` returns it in **radians** | **NED** (north, east, down) | heading 0=North, 90=East | m, rad (also deg in COMMAND_LONG) |
| **aerpawlib `Drone`** (AERPAW vehicle interface) | `position` = WGS-84 lat/lon + **alt relative to home** (`alt+home_amsl`=MSL); `velocity` NED | NED | FRD | `attitude.{roll,pitch,yaw}` = MAVSDK euler **NED/FRD, radians**, `yaw` = heading | `velocity` = `VectorNED{north,east,down}` m/s | `heading` = degrees from North (CW) | lat/lon deg, alt m, vel m/s, att rad |
| **ROS 2 (REP 103 / 105)** | world/map = **ENU** (x=East, y=North, z=**Up**), gravity −z | ENU | body **FLU** (x=forward, y=left, z=up) | quaternion `(x,y,z,w)`, `setRPY` intrinsic **ZYX** | pose in world frame; `Odometry.twist` in **`child_frame_id` (body)** | yaw about **+z (up)**, **CCW positive**, 0 = facing **+x (East)** | m, rad, m/s, rad/s |
| **AeroStack2** | `earth`/`map` = **WGS84/REP105 ENU**; `odom` = local continuous **ENU** about a home origin; `base_link` = **FLU** | ENU (world), FLU (body) | FLU | quaternion, `getTransformation` uses `setRPY(roll,pitch,yaw)` | commands delivered in the frame the platform declares; AS2 sets platform pose/twist **command frame = `odom` (ENU)** | yaw = ENU yaw (CCW, 0=East) | m, rad |

**Reference frame for position.** The adapter defines `odom` as a **flat-earth local
ENU tangent plane whose origin is the vehicle GPS home** (`GeoHome` = lat/lon/alt_msl
captured on the first disarmed telemetry). So AS2 `odom` ENU (E,N,U) is metres from
that home. This matches the reference `as2_platform_multirotor_simulator`
(`sensor_measurements/odom` stamped in the **odom** frame, child `base_link`).

**Frame stamping (adapter output):**
- `sensor_measurements/odom` → `frame_id = <ns>/odom`, `child_frame_id = <ns>/base_link`.
- `sensor_measurements/gps` (NavSatFix) → `frame_id = <ns>/earth` (global WGS84).
- `sensor_measurements/imu` → `frame_id = <ns>/base_link` (body).
- `header.stamp` = AS2 node clock (see §5 on timestamps).

## 3. Orientation conventions in detail

AERPAW heading and AS2 yaw differ by both **zero** and **sign**:

- AS2/ROS yaw θ_enu: measured from **+x (East)**, **counter-clockwise**, radians.
- ArduPilot/aerpawlib heading ψ_ned: measured from **+y_north**, **clockwise**, so
  East = 90°.

Conversion (both directions, in `frame_conversions.hpp`):

```
heading_deg = fmod(90 - deg(yaw_enu) + 360, 360)      # enu_yaw_rad_to_heading_deg
yaw_enu_rad = rad(90 - heading_deg)                    # heading_deg_to_enu_yaw_rad
```

Full attitude (ArduPilot FRD euler → AS2 FLU quaternion): the NED→ENU axis change
(forward stays, right→left flips y, down→up flips z) gives

```
roll_enu  =  roll_ned
pitch_enu = -pitch_ned
yaw_enu   =  π/2 - yaw_ned
```

then `enu_euler_to_quat` (ZYX, matches `tf2::Quaternion::setRPY`) → `Quat{x,y,z,w}`.
Implemented by `ned_euler_to_enu_quat`. Verified: heading North ⇒ ENU yaw 90° ⇒ body +x
maps to world +y (North); heading East ⇒ ENU yaw 0 ⇒ body +x → world +x (East).

## 4. Velocity conventions

Two different velocity frames are used deliberately (each correct for its message):

- **State (AERPAW→AS2).** aerpawlib `velocity` is NED (earth). The adapter:
  `ned_to_enu` gives earth ENU, then `world_vec_to_body(orientation, enu_vel)` puts it
  in **body FLU**, because `nav_msgs/Odometry.twist` is defined in `child_frame_id`
  (REP 103) and AS2 `raw_odometry` republishes it stamped `base_link`.
- **Command (AS2→AERPAW).** AS2 delivers `actuator_command/twist` in the **odom (ENU
  earth)** frame (the adapter declares `setCommandTwistFrameId(odom)`).
  `speed_enu` maps ENU→NED (`enu_to_ned`, a pure axis permutation because it is
  earth-fixed) for aerpawlib `set_velocity(VectorNED)`. (Velocity is only executed in
  SITL; blocked by the AERPAW testbed filter — see `command_capability.hpp`.)

Linear position velocity NED↔ENU is `north=+y_enu, east=+x_enu, down=−z_enu` (and the
inverse). Body angular rates are **not** produced by AERPAW (never fabricated).

## 5. Position, altitude & timestamps

- GPS→local ENU (flat earth), `gps_to_local_enu`:
  `east = (lon−home_lon)·111412.84·cos(home_lat)`, `north = (lat−home_lat)·111132.92`,
  `up = rel_alt`. **Approximation**: locally flat (WGS-84 equirectangular); fine for a
  testbed area, not for global spans. Documented limitation.
- Altitude: aerpawlib `position.alt` is home-relative (up-positive). MSL =
  `alt + home_amsl` (`alt_msl_from_relative`). AS2 ENU z = `rel_alt` (up).
- Timestamps: message `header.stamp` is the AS2 node clock (keeps estimator/TF on one
  clock); the raw AERPAW sample time is carried as `ts` in the IPC message for
  cross-reference.

## 6. Function index (all in `frame_conversions.hpp`)

| Function | Direction / purpose |
|---|---|
| `enu_to_ned` / `ned_to_enu` | earth-frame ENU↔NED (position & velocity) |
| `enu_yaw_rad_to_heading_deg` / `heading_deg_to_enu_yaw_rad` | AS2 yaw ↔ ArduPilot heading |
| `quat_to_yaw_enu` | quaternion → ENU yaw |
| `enu_euler_to_quat` | ENU/FLU euler (ZYX) → quaternion |
| `ned_euler_to_enu_quat` | **AERPAW NED/FRD euler → AS2 ENU/FLU quaternion** |
| `quat_to_rot`, `world_vec_to_body` | rotate an earth (ENU) vector into body (FLU) |
| `gps_to_local_enu`, `meters_per_degree_lon` | WGS84 → local ENU about home |

Wire-format keys live in `command_translation` / `state_translation` and simply call
the above — they contain no independent transform math.

## 7. Test coverage

`translation_gtest` locks the conventions in: `EnuNedRoundTrip`,
`HeadingYawConversions`, `QuatYawMatchesHeading`, `NedAttitudeToEnuQuat`,
`WorldVecToBodyPreservesMagnitude`, `FrameRoundTrip.CommandAndStateAreInverses`,
`FrameRoundTrip.YawHeadingIsInverse`, and the `StateTranslation.*`/`CommandTranslation.*`
direction-specific cases.
