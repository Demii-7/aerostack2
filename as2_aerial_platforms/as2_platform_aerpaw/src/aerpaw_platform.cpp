// Copyright 2024 Universidad Politecnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file aerpaw_platform.cpp
 * @brief AERPAW<->AeroStack2 platform adapter node (thin orchestrator).
 *
 * Responsibility split (integration + translation ONLY):
 *   - this node: as2::AerialPlatform lifecycle wiring, timers, link watchdog,
 *     backend/vehicle-id params.
 *   - command_translation / state_translation: AS2 <-> AERPAW message mapping.
 *   - frame_conversions: coordinate-frame + unit conversion.
 *   - ipc_bridge: UDP-JSON transport to the AERPAW runner.
 * No robotics algorithm or raw frame math lives here.
 */

#include "aerpaw_platform.hpp"

namespace aerpaw_platform
{

AerpawPlatform::AerpawPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  const int cmd_port = this->getParameter<int>("ipc_cmd_port", identity::kDefaultCmdPortBase);
  const int tel_port = this->getParameter<int>("ipc_tel_port", identity::kDefaultTelPortBase);
  backend_ = backend_from_string(this->getParameter<std::string>("platform_backend", "sitl"));
  vehicle_id_ = this->getParameter<std::string>("vehicle_id", "");
  link_timeout_s_ = this->getParameter<double>("link_timeout", 1.0);
  position_update_min_distance_ = this->getParameter<double>("position_update_min_distance", 0.5);
  position_update_min_yaw_ = this->getParameter<double>("position_update_min_yaw", 0.1);
  position_keepalive_s_ = this->getParameter<double>("position_keepalive", 0.5);

  bridge_ = std::make_unique<IpcBridge>(cmd_port, tel_port);
  bridge_->start();
  // Adapter startup failure (e.g. telemetry port already bound): fail loudly so the
  // process exits instead of running half-dead (no fake "up" platform) (Stage 17).
  if (!bridge_->isRunning()) {
    RCLCPP_FATAL(this->get_logger(),
      "AERPAW adapter failed to start (IPC cmd=%d tel=%d unavailable); aborting",
      cmd_port, tel_port);
    throw std::runtime_error("AERPAW adapter IPC startup failed");
  }

  // Raw platform sensors -> sensor_measurements/*; the state estimator turns
  // these into self_localization/*. The adapter does no estimation itself.
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    as2_names::topics::sensor_measurements::odom, as2_names::topics::sensor_measurements::qos);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    as2_names::topics::sensor_measurements::imu, as2_names::topics::sensor_measurements::qos);
  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos);
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    as2_names::topics::sensor_measurements::battery, as2_names::topics::sensor_measurements::qos);
  // AERPAW wireless/RF metrics -> <ns>/aerpaw/measurements (std_msgs/String JSON).
  // Pure transport; parsed only in the experiment layer.
  measurement_pub_ = this->create_publisher<std_msgs::msg::String>(
    "aerpaw/measurements", rclcpp::SensorDataQoS());

  telemetry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&AerpawPlatform::telemetryCallback, this));
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz link + status check
    std::bind(&AerpawPlatform::watchdogCallback, this));

  RCLCPP_INFO(
    this->get_logger(), "AERPAW platform [%s] vehicle='%s' backend=%s cmd=%d tel=%d",
    this->get_namespace(), vehicle_id_.empty() ? "-" : vehicle_id_.c_str(),
    backend_name(backend_), cmd_port, tel_port);
}

void AerpawPlatform::configureSensors()
{
  // No extra sensors beyond the base class; telemetry publishes raw topics above.
}

bool AerpawPlatform::ownSetArmingState(bool state)
{
  return bridge_->sendCommand(command::arm(state));
}

bool AerpawPlatform::ownSetOffboardControl(bool offboard)
{
  // aerpawlib/ArduPilot has no separate offboard gate; commands are always offboard.
  (void)offboard;
  return true;
}

bool AerpawPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & control_in)
{
  using capability::for_control_mode;
  using capability::is_supported;

  const auto support = for_control_mode(backend_, control_in.control_mode, control_in.yaw_mode);
  if (!is_supported(support)) {
    report_unsupported(
      as2::control_mode::controlModeToString(control_in),
      capability::reason(backend_, support));
    return false;  // tell AS2 (via set-platform-mode service) the mode is unusable here
  }
  last_rejected_.clear();  // a supported mode was accepted

  RCLCPP_INFO(
    this->get_logger(), "Control mode: [%s]",
    as2::control_mode::controlModeToString(control_in).c_str());

  control_in_ = control_in;

  // Follow as2_platform_mavlink: commands arrive in the local ENU (odom) frame,
  // which the translation modules expect for their NED conversion.
  setCommandPoseFrameId(getOdomFrameId());
  setCommandTwistFrameId(getOdomFrameId());

  // A mode change is a new control context: force the next POSITION reference to be
  // sent even if it equals the last one.
  sent_pos_valid_ = false;
  return true;
}

bool AerpawPlatform::goto_due(const frames::Vec3 & cur, double yaw)
{
  const auto now = std::chrono::steady_clock::now();
  const bool changed = !sent_pos_valid_ ||
    command::position_reference_changed(
    last_sent_pos_, cur, last_sent_yaw_, yaw,
    position_update_min_distance_, position_update_min_yaw_);
  const bool keepalive_due = sent_pos_valid_ &&
    (now - last_goto_sent_) >= std::chrono::duration<double>(position_keepalive_s_);
  if (changed || keepalive_due) {
    last_sent_pos_ = cur;
    last_sent_yaw_ = yaw;
    last_goto_sent_ = now;
    sent_pos_valid_ = true;
    return true;
  }
  return false;
}

bool AerpawPlatform::ownSendCommand()
{
  using capability::for_control_mode;
  using capability::is_supported;

  // Re-check capability every cycle (defensive; mode should already be gated).
  if (!is_supported(for_control_mode(backend_, control_in_.control_mode, control_in_.yaw_mode))) {
    return false;
  }

  if (control_in_.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    if (backend_ == PlatformBackend::SITL) {
      return bridge_->sendCommand(command::hover());  // velocity-zero works in SITL
    }
    // Testbed: LOITER/offboard is filter-banned, so hover = reposition-to-current
    // via the supported primitive, throttled (DO_REPOSITION is not a fast setpoint).
    if (!has_last_pose_ || !hover_reposition_due()) {
      return true;
    }
    return bridge_->sendCommand(command::hover_at(last_pos_enu_, last_yaw_enu_, home_));
  }
  if (control_in_.control_mode == as2_msgs::msg::ControlMode::SPEED) {
    // Only reachable when velocity is supported (SITL); testbed blocked at mode-set.
    const frames::Vec3 v{
      command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y,
      command_twist_msg_.twist.linear.z};
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
      warn_bad_command("SPEED");
      return false;
    }
    return bridge_->sendCommand(command::speed_enu(command_twist_msg_));
  }
  if (control_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    // Online update: forward the reference only when it changed (or keepalive due),
    // so the researcher can retarget during flight without flooding DO_REPOSITION.
    const frames::Vec3 cur{
      command_pose_msg_.pose.position.x, command_pose_msg_.pose.position.y,
      command_pose_msg_.pose.position.z};
    const double yaw = frames::quat_to_yaw_enu(
      command_pose_msg_.pose.orientation.x, command_pose_msg_.pose.orientation.y,
      command_pose_msg_.pose.orientation.z, command_pose_msg_.pose.orientation.w);
    if (!guard::command_is_valid(cur, yaw)) {
      warn_bad_command("POSITION");  // never forward a malformed setpoint
      return false;
    }
    if (!goto_due(cur, yaw)) {
      return true;  // unchanged & within keepalive window: nothing new to send
    }
    return bridge_->sendCommand(command::goto_enu(command_pose_msg_, home_));
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000, "Unrouted control mode - command ignored");
  return false;
}

double AerpawPlatform::clock_now_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void AerpawPlatform::warn_bad_command(const char * what)
{
  const double now = clock_now_s();
  if (now - last_guard_warn_s_ > 5.0) {
    last_guard_warn_s_ = now;
    RCLCPP_WARN(this->get_logger(), "Dropping malformed %s command reference (non-finite)",
      what);
  }
}

void AerpawPlatform::ownKillSwitch()
{
  // AS2 kill-switch = immediate irreversible motor stop. ArduPilot via aerpawlib
  // has no true equivalent; disarm is the strongest available action. On real
  // hardware this is a documented safety limitation (see README Limitations).
  if (is_real_hardware(backend_)) {
    RCLCPP_ERROR(this->get_logger(), "KILL on PHYSICAL testbed = disarm only (see Limitations)");
  }
  bridge_->sendCommand(command::kill());
}

void AerpawPlatform::ownStopPlatform()
{
  // AS2 "stop" = hover as best as possible. On the testbed use the supported
  // reposition-to-current (never LOITER/offboard); in SITL the velocity hold works.
  if (backend_ == PlatformBackend::SITL) {
    bridge_->sendCommand(command::stop());
    return;
  }
  if (has_last_pose_) {
    bridge_->sendCommand(command::hover_at(last_pos_enu_, last_yaw_enu_, home_));
  } else {
    RCLCPP_WARN(this->get_logger(), "Stop requested before first telemetry - cannot reposition");
  }
}

bool AerpawPlatform::ownTakeoff()
{
  const double alt = this->getParameter<double>("takeoff_altitude", 25.0);
  return bridge_->sendCommand(command::takeoff(alt));
}

bool AerpawPlatform::ownLand()
{
  return bridge_->sendCommand(command::land());
}

void AerpawPlatform::telemetryCallback()
{
  const TelemetryState tel = bridge_->getTelemetry();
  if (tel.kind != InboundKind::TELEMETRY) {
    return;
  }

  // Interface version check (IF-1): warn once if the runner speaks a different
  // protocol major version than this adapter.
  if (!proto_checked_) {
    proto_checked_ = true;
    if (tel.proto != kProtocolVersion) {
      RCLCPP_WARN(this->get_logger(),
        "AERPAW wire protocol mismatch: runner=proto%d adapter=proto%d",
        tel.proto, kProtocolVersion);
    }
  }

  // STAGE 17 - never let malformed or stale AERPAW data reach AeroStack2.
  // (a) invalid sample -> drop (do not publish a garbage odom/gps/imu).
  if (!guard::telemetry_is_valid(tel)) {
    ++invalid_tel_count_;
    const double now = clock_now_s();
    if (now - last_guard_warn_s_ > 5.0) {
      last_guard_warn_s_ = now;
      RCLCPP_WARN(this->get_logger(),
        "Dropping invalid AERPAW telemetry (n=%llu): non-finite/out-of-range or no-fix",
        static_cast<unsigned long long>(invalid_tel_count_));
    }
    return;
  }
  // (b) stale / frozen sample -> drop (do NOT republish the last packet forever).
  const double age = bridge_->telemetryAgeSeconds();
  if (!guard::telemetry_is_fresh(age, link_timeout_s_, tel.ts, last_tel_ts_)) {
    if (age > link_timeout_s_) {
      ++stale_drop_count_;  // link gap; watchdog also flips connected=false
    }
    return;  // same-or-older ts: nothing new to publish this cycle
  }

  // Establish the ENU odom origin from the first disarmed telemetry (GPS home).
  if (!home_.set && !tel.armed) {
    home_.lat = tel.lat;
    home_.lon = tel.lon;
    home_.alt_msl = tel.alt_msl;
    home_.set = true;
  }
  // Adopt the AERPAW-side vehicle id if the node was launched without one.
  if (vehicle_id_.empty() && !tel.vehicle_id.empty()) {
    vehicle_id_ = tel.vehicle_id;
  }
  // Capture the AERPAW hardware-unique id (MAVLink system id) and log the mapping
  // AERPAW vehicle -> this AS2 platform exactly once (multi-UAV identification).
  if (tel.mavlink_sysid != 0) {
    aerpaw_sysid_ = tel.mavlink_sysid;
  }
  if (!identity_reported_ && !vehicle_id_.empty()) {
    identity_reported_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Mapped AERPAW vehicle id='%s' sysid=%d (%s) -> AS2 platform ns='%s' backend=%s",
      vehicle_id_.c_str(), aerpaw_sysid_, backend_name(backend_),
      this->get_namespace());
  }

  const auto stamp = this->now();

  nav_msgs::msg::Odometry odom = state::makeOdometry(
    tel, home_, getOdomFrameId(), getBaseFrameId());
  odom.header.stamp = stamp;
  odom_pub_->publish(odom);

  // Remember the current ENU pose + yaw so HOVER/STOP can reposition on the
  // testbed using the supported DO_REPOSITION primitive.
  last_pos_enu_ = frames::Vec3{
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
  last_yaw_enu_ = frames::quat_to_yaw_enu(
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  has_last_pose_ = true;
  last_tel_ts_ = tel.ts;  // accepted; a repeat of this ts is stale next cycle

  sensor_msgs::msg::NavSatFix gps = state::makeGps(tel, getEarthFrameId());
  gps.header.stamp = stamp;
  gps_pub_->publish(gps);

  sensor_msgs::msg::Imu imu = state::makeImu(tel, getBaseFrameId());
  imu.header.stamp = stamp;
  imu_pub_->publish(imu);

  if (tel.battery_valid) {
    sensor_msgs::msg::BatteryState battery = state::makeBattery(tel);
    battery.header.stamp = stamp;
    battery_pub_->publish(battery);
  }

  // Flight-state transitions (AERPAW mode/health). AS2 PlatformInfo has no
  // mode/ekf/armable fields, so these are surfaced via logs, not fabricated into
  // the platform message.
  if (tel.mode != last_mode_) {
    RCLCPP_INFO(
      this->get_logger(), "AERPAW flight state: mode='%s' armed=%d armable=%d ekf_ready=%d",
      tel.mode.c_str(), tel.armed ? 1 : 0, tel.armable ? 1 : 0, tel.ekf_ready ? 1 : 0);
    last_mode_ = tel.mode;
  }
}

void AerpawPlatform::watchdogCallback()
{
  // Link lifecycle: the base class hardcodes platform_info.connected = true, so
  // the adapter owns the real link state from telemetry freshness.
  const bool up = bridge_->telemetryAgeSeconds() <= link_timeout_s_;
  if (up != link_up_) {
    link_up_ = up;
    platform_info_msg_.connected = up;
    if (up) {
      RCLCPP_INFO(this->get_logger(), "AERPAW link UP");
    } else {
      RCLCPP_ERROR(this->get_logger(), "AERPAW link LOST (no telemetry for > %.2fs)",
        link_timeout_s_);
    }
  }

  // Transport AERPAW measurement payloads straight through to ROS (no interpretation).
  const AdapterMeasurement meas = bridge_->getMeasurement();
  if (meas.valid && meas.ts != last_measurement_ts_) {
    last_measurement_ts_ = meas.ts;
    std_msgs::msg::String out;
    out.data = meas.payload_json;
    measurement_pub_->publish(out);
  }

  // Error / status propagation from the AERPAW runner (deduped by timestamp).
  const AdapterEvent ev = bridge_->getEvent();
  if (ev.kind != InboundKind::NONE && ev.ts != last_event_ts_) {
    last_event_ts_ = ev.ts;
    if (ev.kind == InboundKind::ERROR) {
      RCLCPP_ERROR(this->get_logger(), "AERPAW error cmd='%s': %s",
        ev.command.c_str(), ev.message.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "AERPAW status: %s", ev.message.c_str());
    }
  }
}

void AerpawPlatform::report_unsupported(const std::string & what, const char * why)
{
  // Log each distinct rejection once (avoid 100 Hz spam); cleared on a good mode.
  if (last_rejected_ == what) {
    return;
  }
  last_rejected_ = what;
  RCLCPP_ERROR(
    this->get_logger(), "Command/mode '%s' not executed by AERPAW (%s) - backend=%s",
    what.c_str(), why, backend_name(backend_));
}

}  // namespace aerpaw_platform
