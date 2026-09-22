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
 * @file state_translation.cpp
 * @brief AERPAW telemetry -> AS2 sensor message builders.
 *
 * Availability policy (Stage 5 - never fabricate):
 *   AVAILABLE from AERPAW and published: position (GPS), velocity (NED), attitude
 *   (roll/pitch/yaw), altitude, GPS fix quality, battery (v/current/%), armed/
 *   mode/armable/ekf, connection.
 *   NOT AVAILABLE (ArduPilot/aerpawlib does not surface it): body angular velocity
 *   (rates) and linear acceleration. These are set to zero AND marked with a
 *   covariance of -1 (the ROS convention for "sensor does not provide this"), so
 *   downstream consumers see them as unknown rather than as a real measurement.
 */

#include "state_translation.hpp"

#include <cmath>

#include "frame_conversions.hpp"

namespace aerpaw_platform
{
namespace state
{

namespace
{
// ROS convention: a covariance diagonal of -1 marks "data not provided/unknown".
constexpr double kCovUnknown = -1.0;
}  // namespace

nav_msgs::msg::Odometry makeOdometry(
  const TelemetryState & tel, const GeoHome & home,
  const std::string & frame_id, const std::string & child_frame_id)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

  const frames::Vec3 enu_pos = frames::gps_to_local_enu(
    tel.lat, tel.lon, tel.rel_alt, home.lat, home.lon);
  odom.pose.pose.position.x = enu_pos.x;
  odom.pose.pose.position.y = enu_pos.y;
  odom.pose.pose.position.z = enu_pos.z;

  // Attitude (available): NED euler -> AS2 ENU/FLU quaternion.
  const frames::Quat q = frames::ned_euler_to_enu_quat(
    tel.roll_rad, tel.pitch_rad, tel.yaw_ned_rad);
  odom.pose.pose.orientation.x = q.x;
  odom.pose.pose.orientation.y = q.y;
  odom.pose.pose.orientation.z = q.z;
  odom.pose.pose.orientation.w = q.w;

  // Linear velocity: NED (earth) -> ENU (earth) -> body (AS2 Odometry.twist is in
  // the child/base frame, matching the reference multirotor platform).
  const frames::Vec3 enu_vel = frames::ned_to_enu(
    frames::Vec3{tel.vx_ned, tel.vy_ned, tel.vz_ned});
  const frames::Vec3 body_vel = frames::world_vec_to_body(q, enu_vel);
  odom.twist.twist.linear.x = body_vel.x;
  odom.twist.twist.linear.y = body_vel.y;
  odom.twist.twist.linear.z = body_vel.z;

  // Angular velocity NOT available from AERPAW -> zero + "unknown" covariance.
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom.twist.covariance[0] = kCovUnknown;  // roll rate unknown
  odom.twist.covariance[7] = kCovUnknown;  // pitch rate unknown
  odom.twist.covariance[15] = kCovUnknown;  // yaw rate unknown
  return odom;
}

sensor_msgs::msg::NavSatFix makeGps(const TelemetryState & tel, const std::string & frame_id)
{
  sensor_msgs::msg::NavSatFix gps;
  gps.header.frame_id = frame_id;
  gps.latitude = tel.lat;
  gps.longitude = tel.lon;
  gps.altitude = tel.alt_msl;

  // Real GNSS fix quality from AERPAW (no fix / 2D / 3D).
  gps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  if (tel.gps_fix_type >= 2) {
    gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  // Position covariance not provided by AERPAW -> "unknown".
  gps.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  return gps;
}

sensor_msgs::msg::Imu makeImu(const TelemetryState & tel, const std::string & base_frame_id)
{
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = base_frame_id;

  // Orientation IS available (full NED euler -> ENU quaternion).
  const frames::Quat q = frames::ned_euler_to_enu_quat(
    tel.roll_rad, tel.pitch_rad, tel.yaw_ned_rad);
  imu.orientation.x = q.x;
  imu.orientation.y = q.y;
  imu.orientation.z = q.z;
  imu.orientation.w = q.w;

  // Angular velocity + linear acceleration are NOT provided by AERPAW.
  // Zeroed and flagged UNKNOWN via covariance -1 (never fabricate 9.81 / rates).
  imu.angular_velocity_covariance[0] = kCovUnknown;
  imu.linear_acceleration_covariance[0] = kCovUnknown;
  return imu;
}

sensor_msgs::msg::BatteryState makeBattery(const TelemetryState & tel)
{
  sensor_msgs::msg::BatteryState bat;
  // Voltage / current / percentage ARE available from AERPAW.
  bat.voltage = static_cast<float>(tel.battery_voltage);
  bat.current = static_cast<float>(tel.battery_current);
  bat.percentage = static_cast<float>(tel.battery_level / 100.0);  // AERPAW 0-100 -> 0-1
  // Fields AERPAW does not provide stay at sensor_msgs defaults (UNKNOWN), not faked.
  bat.present = tel.battery_valid;
  bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  bat.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  return bat;
}

}  // namespace state
}  // namespace aerpaw_platform
