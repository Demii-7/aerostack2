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
 * @file command_translation.cpp
 * @brief AERPAW-side command wire format. Keys match aerpaw_as2_runner.py.
 */

#include "command_translation.hpp"

#include <cmath>

namespace aerpaw_platform
{
namespace command
{

nlohmann::json arm(bool armed)
{
  return nlohmann::json{{"cmd", armed ? "arm" : "disarm"}};
}

nlohmann::json takeoff(double altitude_m)
{
  return nlohmann::json{{"cmd", "takeoff"}, {"alt", altitude_m}};
}

nlohmann::json land()
{
  return nlohmann::json{{"cmd", "land"}};
}

nlohmann::json kill()
{
  return nlohmann::json{{"cmd", "kill"}};
}

nlohmann::json stop()
{
  return nlohmann::json{{"cmd", "stop"}};
}

nlohmann::json hover()
{
  return speed_enu(frames::Vec3{0.0, 0.0, 0.0});
}

nlohmann::json hover_at(const frames::Vec3 & enu_pos, double yaw_enu_rad, const GeoHome & home)
{
  // Reposition to the current pose - a supported primitive on the testbed, unlike
  // LOITER/offboard. Same wire form as a position command.
  return goto_enu(enu_pos, yaw_enu_rad, home);
}

nlohmann::json speed_enu(const frames::Vec3 & enu_vel)
{
  const frames::Vec3 ned = frames::enu_to_ned(enu_vel);
  return nlohmann::json{
    {"cmd", "set_velocity"},
    {"vx", ned.x},   // north
    {"vy", ned.y},   // east
    {"vz", ned.z}};  // down
}

nlohmann::json speed_enu(const geometry_msgs::msg::TwistStamped & twist)
{
  return speed_enu(
    frames::Vec3{
      twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z});
}

nlohmann::json goto_enu(
  const frames::Vec3 & enu_pos, double yaw_enu_rad, const GeoHome & home)
{
  const frames::Vec3 ned = frames::enu_to_ned(enu_pos);
  nlohmann::json cmd{
    {"cmd", "goto_ned"},
    {"north", ned.x},
    {"east", ned.y},
    {"down", ned.z},
    {"heading", frames::enu_yaw_rad_to_heading_deg(yaw_enu_rad)}};
  // Absolute target is rebuilt on the AERPAW side from the fixed odom origin.
  cmd["home_lat"] = home.lat;
  cmd["home_lon"] = home.lon;
  return cmd;
}

nlohmann::json goto_enu(const geometry_msgs::msg::PoseStamped & pose, const GeoHome & home)
{
  const frames::Vec3 enu_pos{
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
  const double yaw = frames::quat_to_yaw_enu(
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);
  return goto_enu(enu_pos, yaw, home);
}

bool position_reference_changed(
  const frames::Vec3 & prev, const frames::Vec3 & cur,
  double yaw_prev, double yaw_cur, double pos_eps, double yaw_eps)
{
  const double dx = cur.x - prev.x, dy = cur.y - prev.y, dz = cur.z - prev.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) > pos_eps) {
    return true;
  }
  // Shortest angular difference, wrapped to (-pi, pi].
  double dyaw = std::fmod(yaw_cur - yaw_prev + M_PI, 2.0 * M_PI);
  if (dyaw < 0.0) {
    dyaw += 2.0 * M_PI;
  }
  return std::abs(dyaw - M_PI) > yaw_eps;
}

}  // namespace command
}  // namespace aerpaw_platform
