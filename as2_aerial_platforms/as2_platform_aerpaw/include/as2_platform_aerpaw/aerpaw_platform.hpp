// Copyright 2024 Universidad Politecnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
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
 * @file aerpaw_platform.hpp
 * @brief AERPAW Digital Twin aerial platform for AS2.
 */

#ifndef AS2_PLATFORM_AERPAW__AERPAW_PLATFORM_HPP_
#define AS2_PLATFORM_AERPAW__AERPAW_PLATFORM_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

#include "as2_msgs/msg/control_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

#include "adapter_types.hpp"
#include "command_capability.hpp"
#include "command_translation.hpp"
#include "frame_conversions.hpp"
#include "ipc_bridge.hpp"
#include "state_translation.hpp"
#include "telemetry_guard.hpp"
#include "vehicle_identity.hpp"

namespace aerpaw_platform
{

/**
 * @brief AERPAW<->AeroStack2 platform adapter.
 *
 * This node is deliberately THIN: it wires the as2::AerialPlatform lifecycle to
 * the transport (IpcBridge) and to the isolated translation modules
 * (command_translation / state_translation / frame_conversions). It holds no
 * robotics algorithm and no coordinate math of its own. AERPAW-specific concepts
 * (backend, home origin, vehicle id) are confined to adapter_types.hpp and never
 * leak into AeroStack2 core.
 *
 * LAYER BOUNDARY (Stage 9): this is PLATFORM INTEGRATION only - communication +
 * state/command translation + platform abstraction. Mission objectives, decision
 * making, optimisation and RF/network metrics belong to the EXPERIMENT layer
 * (as2_experiment_aerpaw_multiuav) and MUST NOT be added here. See
 * docs/LAYERING.md (enforced by as2_experiment_aerpaw_multiuav/test/test_layering.py).
 */
class AerpawPlatform : public as2::AerialPlatform
{
public:
  explicit AerpawPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AerpawPlatform() {}

  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;
  bool ownTakeoff() override;
  bool ownLand() override;

private:
  std::unique_ptr<IpcBridge> bridge_;
  as2_msgs::msg::ControlMode control_in_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  // Transport-only bridge of AERPAW wireless/RF metrics onto a standard ROS topic
  // (<ns>/aerpaw/measurements, std_msgs/String JSON). The adapter never interprets
  // them; the experiment layer consumes them via experiment/rf_metrics.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr measurement_pub_;

  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // AERPAW-side abstraction (isolated; see adapter_types.hpp).
  GeoHome home_;
  PlatformBackend backend_ = PlatformBackend::SITL;
  std::string vehicle_id_;
  int aerpaw_sysid_ = 0;        ///< Hardware-unique AERPAW id, adopted from telemetry.
  bool identity_reported_ = false;  ///< One-shot AERPAW<->AS2 platform mapping log.
  std::string last_mode_;       ///< Flight-state (mode) change detection.

  // Command-path state: last commanded/actual pose (for testbed hover) + logs.
  frames::Vec3 last_pos_enu_;
  double last_yaw_enu_ = 0.0;
  bool has_last_pose_ = false;
  std::chrono::steady_clock::time_point last_hover_reposition_{};
  bool hover_reposition_sent_ = false;
  std::string last_rejected_;   ///< Dedupe "unsupported command" error logs.

  // Online trajectory updates (Stage 11): forward a POSITION reference only when it
  // actually changes (or a keepalive elapses), so new in-flight references flow
  // through without flooding DO_REPOSITION (a *target*, not a fast setpoint).
  double position_update_min_distance_ = 0.5;   // m
  double position_update_min_yaw_ = 0.1;        // rad
  double position_keepalive_s_ = 0.5;           // s: resend same target at most this slow
  frames::Vec3 last_sent_pos_;
  double last_sent_yaw_ = 0.0;
  bool sent_pos_valid_ = false;
  std::chrono::steady_clock::time_point last_goto_sent_{};
  bool goto_due(const frames::Vec3 & cur, double yaw);

  // Throttle reposition-to-hover to ~2 Hz (DO_REPOSITION is not a 100 Hz setpoint).
  bool hover_reposition_due()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!hover_reposition_sent_ || (now - last_hover_reposition_) >= std::chrono::milliseconds(500)) {
      last_hover_reposition_ = now;
      hover_reposition_sent_ = true;
      return true;
    }
    return false;
  }
  void report_unsupported(const std::string & what, const char * why);

  // Connection / lifecycle state (link watchdog + status propagation).
  double link_timeout_s_ = 1.0;
  bool link_up_ = false;
  double last_event_ts_ = 0.0;  ///< Dedupe AERPAW status/error reports we logged.
  double last_measurement_ts_ = 0.0;  ///< Dedupe measurement payloads we republish.
  bool proto_checked_ = false;  ///< One-shot wire-protocol version check (IF-1).

  // Failure handling (Stage 17): drop invalid/stale telemetry, never republish.
  double last_tel_ts_ = 0.0;            ///< Last accepted telemetry timestamp.
  uint64_t invalid_tel_count_ = 0;      ///< Rejected invalid telemetry samples.
  uint64_t stale_drop_count_ = 0;       ///< Dropped stale/frozen telemetry samples.
  double last_guard_warn_s_ = 0.0;      ///< Throttle guard warning logs (steady clock).
  double clock_now_s();                 ///< Steady seconds for throttling.
  void warn_bad_command(const char * what);

  void telemetryCallback();
  void watchdogCallback();
};

}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__AERPAW_PLATFORM_HPP_
