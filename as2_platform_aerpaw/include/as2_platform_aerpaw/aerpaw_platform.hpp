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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

#include "as2_msgs/msg/control_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "ipc_bridge.hpp"

namespace aerpaw_platform
{

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

  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  double home_lat_ = 0.0;
  double home_lon_ = 0.0;
  double home_alt_ = 0.0;
  bool home_set_ = false;

  void telemetryCallback();
};

}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__AERPAW_PLATFORM_HPP_
