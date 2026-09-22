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
 * @file command_translation.hpp
 * @brief AeroStack2 command -> AERPAW runner command translation (adapter).
 *
 * One direction of the seam: it turns an AS2 command (already delivered in the
 * ENU odom frame by as2::AerialPlatform) into the JSON the aerpawlib runner
 * consumes, using the NED/heading conversions in frame_conversions.hpp. This is
 * pure integration/translation - no control law or planning lives here, those are
 * AeroStack2 behaviours/controllers upstream.
 */

#ifndef AS2_PLATFORM_AERPAW__COMMAND_TRANSLATION_HPP_
#define AS2_PLATFORM_AERPAW__COMMAND_TRANSLATION_HPP_

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nlohmann/json.hpp>

#include "adapter_types.hpp"
#include "frame_conversions.hpp"

namespace aerpaw_platform
{
namespace command
{

// Lifecycle / discrete commands -------------------------------------------------
nlohmann::json arm(bool armed);
nlohmann::json takeoff(double altitude_m);
nlohmann::json land();
nlohmann::json kill();
nlohmann::json stop();
/// Velocity-zero hover (works only where AERPAW allows offboard velocity, i.e. SITL).
nlohmann::json hover();

// Continuous commands (ENU, earth-fixed odom frame) ----------------------------

/// Speed command: ENU velocity -> {"cmd":"set_velocity", NED vx,vy,vz}.
nlohmann::json speed_enu(const frames::Vec3 & enu_vel);

/// Speed command taken straight from an AS2 TwistStamped (linear part, ENU).
nlohmann::json speed_enu(const geometry_msgs::msg::TwistStamped & twist);

/// Hover on the testbed WITHOUT the (filter-banned) LOITER: reposition to the
/// current ENU pose using the supported DO_REPOSITION primitive.
nlohmann::json hover_at(const frames::Vec3 & enu_pos, double yaw_enu_rad, const GeoHome & home);

/// Pose command: ENU position + ENU yaw -> {"cmd":"goto_ned", NED offsets + heading}.
nlohmann::json goto_enu(const frames::Vec3 & enu_pos, double yaw_enu_rad,
  const GeoHome & home);

/// Pose command taken straight from an AS2 PoseStamped (ENU position + yaw).
nlohmann::json goto_enu(const geometry_msgs::msg::PoseStamped & pose, const GeoHome & home);

/**
 * @brief Online-update gate (Stage 11): did a commanded position reference change
 * enough to deserve a fresh reposition?
 *
 * AERPAW's executable primitive is DO_REPOSITION (a *target*, not a high-rate
 * setpoint), so the adapter forwards a new waypoint only when it actually moves
 * (beyond pos_eps metres / yaw_eps radians) - while still allowing continuous
 * updates during flight. Pure + unit-tested.
 */
bool position_reference_changed(
  const frames::Vec3 & prev, const frames::Vec3 & cur,
  double yaw_prev, double yaw_cur, double pos_eps, double yaw_eps);

}  // namespace command
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__COMMAND_TRANSLATION_HPP_
