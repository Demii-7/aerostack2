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
// THIS IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * @brief AERPAW Digital Twin platform implementation.
 *
 * Frame conventions:
 *   AS2 odom frame: ENU (x=East, y=North, z=Up).
 *   ArduPilot heading: degrees from North, clockwise (0=N, 90=E).
 *   Conversion: heading_aerpaw = fmod(90 - yaw_enu_deg, 360).
 *
 * Telemetry published to sensor_measurements/* (raw platform data).
 * The AS2 state estimator node is responsible for producing
 * self_localization/* from these sensor topics.
 */

#include "aerpaw_platform.hpp"

#include <cmath>

#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/utils.h"

namespace aerpaw_platform
{

AerpawPlatform::AerpawPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  int cmd_port = this->getParameter<int>("ipc_cmd_port", 15760);
  int tel_port = this->getParameter<int>("ipc_tel_port", 15761);

  bridge_ = std::make_unique<IpcBridge>(cmd_port, tel_port);
  bridge_->start();

  // Publish raw platform sensors to sensor_measurements/*.
  // The state estimator produces self_localization/* from these.
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    as2_names::topics::sensor_measurements::odom, as2_names::topics::sensor_measurements::qos);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    as2_names::topics::sensor_measurements::imu, as2_names::topics::sensor_measurements::qos);

  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos);

  // Telemetry timer at 20 Hz
  telemetry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&AerpawPlatform::telemetryCallback, this));

  RCLCPP_INFO(this->get_logger(), "AERPAW platform started (cmd=%d, tel=%d)", cmd_port, tel_port);
}

void AerpawPlatform::configureSensors()
{
  // No additional sensors beyond base class
}

bool AerpawPlatform::ownSetArmingState(bool state)
{
  nlohmann::json cmd;
  cmd["cmd"] = state ? "arm" : "disarm";
  return bridge_->sendCommand(cmd);
}

bool AerpawPlatform::ownSetOffboardControl(bool offboard)
{
  return true;
}

bool AerpawPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & control_in)
{
  RCLCPP_INFO(
    this->get_logger(), "Control mode: [%s]",
    as2::control_mode::controlModeToString(control_in).c_str());

  control_in_ = control_in;

  // Follow as2_platform_mavlink: commands arrive in the local ENU (odom) frame,
  // which the ownSendCommand() NED conversions below expect.
  setCommandPoseFrameId(getOdomFrameId());
  setCommandTwistFrameId(getOdomFrameId());

  return true;
}

bool AerpawPlatform::ownSendCommand()
{
  if (control_in_.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    nlohmann::json cmd;
    cmd["cmd"] = "set_velocity";
    cmd["vx"] = 0.0;
    cmd["vy"] = 0.0;
    cmd["vz"] = 0.0;
    return bridge_->sendCommand(cmd);
  }

  if (control_in_.control_mode == as2_msgs::msg::ControlMode::SPEED) {
    // command_twist_msg_ arrives in ENU (odom) after base-class conversion.
    // ENU->NED: vx_ned=y_enu, vy_ned=x_enu, vz_ned=-z_enu.
    // This is a pure axis permutation (no heading needed) because the twist
    // is already in the earth-fixed odom frame, not the body FLU frame.
    nlohmann::json cmd;
    cmd["cmd"] = "set_velocity";
    cmd["vx"] = command_twist_msg_.twist.linear.y;   // north = y_enu
    cmd["vy"] = command_twist_msg_.twist.linear.x;   // east  = x_enu
    cmd["vz"] = -command_twist_msg_.twist.linear.z;  // down  = -z_enu
    return bridge_->sendCommand(cmd);
  }

  if (control_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    // command_pose_msg_ arrives in ENU (odom) after base-class conversion.
    // ENU->NED: north=+y, east=+x, down=-z.
    // Heading: AS2 ENU yaw (0=East, CCW) to ArduPilot heading (0=North, CW).
    nlohmann::json cmd;
    cmd["cmd"] = "goto_ned";
    cmd["north"] = command_pose_msg_.pose.position.y;
    cmd["east"] = command_pose_msg_.pose.position.x;
    cmd["down"] = -command_pose_msg_.pose.position.z;

    double yaw_enu_rad = tf2::getYaw(command_pose_msg_.pose.orientation);
    double yaw_enu_deg = yaw_enu_rad * 180.0 / M_PI;
    double heading_ap = fmod(90.0 - yaw_enu_deg + 360.0, 360.0);
    cmd["heading"] = heading_ap;

    // Include the platform's fixed home (ENU origin) so the runner builds
    // an absolute WGS-84 target instead of a relative offset.
    cmd["home_lat"] = home_lat_;
    cmd["home_lon"] = home_lon_;

    return bridge_->sendCommand(cmd);
  }

  RCLCPP_WARN(this->get_logger(), "Unsupported control mode - command ignored");
  return false;
}

void AerpawPlatform::ownKillSwitch()
{
  // AS2 defines ownKillSwitch() as an immediate, irreversible emergency motor
  // stop.  ArduPilot has no direct equivalent accessible via aerpawlib; the
  // strongest available action is disarm.  This is a known limitation for
  // real-hardware safety — use with caution.
  bridge_->sendCommand({{"cmd", "kill"}});
}

void AerpawPlatform::ownStopPlatform()
{
  nlohmann::json cmd;
  cmd["cmd"] = "stop";
  bridge_->sendCommand(cmd);
}

bool AerpawPlatform::ownTakeoff()
{
  // Read takeoff altitude from parameter (default 25 m; min safe value on DT).
  double alt = this->getParameter<double>("takeoff_altitude", 25.0);
  nlohmann::json cmd;
  cmd["cmd"] = "takeoff";
  cmd["alt"] = alt;
  return bridge_->sendCommand(cmd);
}

bool AerpawPlatform::ownLand()
{
  return bridge_->sendCommand({{"cmd", "land"}});
}

void AerpawPlatform::telemetryCallback()
{
  TelemetryState tel = bridge_->getTelemetry();

  if (!tel.connected) {return;}

  // Store home position on first telemetry when disarmed.
  // This establishes the ENU odom origin = GPS lat/lon at arm point.
  if (!tel.armed && !home_set_) {
    home_lat_ = tel.lat;
    home_lon_ = tel.lon;
    home_alt_ = tel.alt_msl;
    home_set_ = true;
  }

  auto stamp = this->now();

  // --- Publish odometry (sensor_measurements/odom) ---
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = getEarthFrameId();
  odom.child_frame_id = getBaseFrameId();

  // Convert lat/lon to local ENU (simplified flat-earth near home)
  const double lat_rad = home_lat_ * M_PI / 180.0;
  const double m_per_deg_lat = 111132.92;
  const double m_per_deg_lon = 111412.84 * cos(lat_rad);

  double east = (tel.lon - home_lon_) * m_per_deg_lon;
  double north = (tel.lat - home_lat_) * m_per_deg_lat;
  double up = tel.rel_alt;

  odom.pose.pose.position.x = east;
  odom.pose.pose.position.y = north;
  odom.pose.pose.position.z = up;

  // NED velocity to ENU
  odom.twist.twist.linear.x = tel.vy_ned;
  odom.twist.twist.linear.y = tel.vx_ned;
  odom.twist.twist.linear.z = -tel.vz_ned;

  odom_pub_->publish(odom);

  // --- Publish GPS (sensor_measurements/gps) ---
  sensor_msgs::msg::NavSatFix gps;
  gps.header.stamp = stamp;
  gps.header.frame_id = getEarthFrameId();
  gps.latitude = tel.lat;
  gps.longitude = tel.lon;
  gps.altitude = tel.alt_msl;
  gps_pub_->publish(gps);

  // --- Publish IMU (sensor_measurements/imu) ---
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = getBaseFrameId();

  // Convert ArduPilot heading (deg from North, CW) to ENU yaw (rad from East, CCW)
  double yaw_enu_rad = (90.0 - tel.yaw) * M_PI / 180.0;

  imu.orientation.z = sin(yaw_enu_rad / 2.0);
  imu.orientation.w = cos(yaw_enu_rad / 2.0);

  imu.angular_velocity.z = 0.0;  // No yaw rate from telemetry

  imu.linear_acceleration.z = 9.81;  // Approximate

  imu_pub_->publish(imu);
}

}  // namespace aerpaw_platform
