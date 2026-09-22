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
 * @file state_translation.hpp
 * @brief AERPAW telemetry -> AeroStack2 sensor-message translation (adapter).
 *
 * The other direction of the seam. Turns the raw AERPAW vehicle state (lat/lon,
 * NED velocity, ArduPilot heading) into AS2 `sensor_measurements/*` messages in
 * the ENU odom frame, via frame_conversions.hpp. The node stays wiring-only; the
 * estimator (not this adapter) fuses these into self_localization/*.
 */

#ifndef AS2_PLATFORM_AERPAW__STATE_TRANSLATION_HPP_
#define AS2_PLATFORM_AERPAW__STATE_TRANSLATION_HPP_

#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "adapter_types.hpp"
#include "ipc_bridge.hpp"

namespace aerpaw_platform
{
namespace state
{

/// Raw platform odometry (ENU pose about `home`; body-frame twist), for
/// sensor_measurements/odom. Angular velocity is unavailable (cov -1 marker).
nav_msgs::msg::Odometry makeOdometry(
  const TelemetryState & tel, const GeoHome & home,
  const std::string & frame_id, const std::string & child_frame_id);

/// Raw GNSS fix incl. real fix quality, for sensor_measurements/gps.
sensor_msgs::msg::NavSatFix makeGps(const TelemetryState & tel, const std::string & frame_id);

/// Attitude from AERPAW euler; angular velocity + linear acceleration are NOT
/// provided by AERPAW and are marked UNAVAILABLE (covariance -1), never faked.
sensor_msgs::msg::Imu makeImu(const TelemetryState & tel, const std::string & base_frame_id);

/// Battery from AERPAW (voltage/current/percentage); other fields left UNKNOWN.
sensor_msgs::msg::BatteryState makeBattery(const TelemetryState & tel);

}  // namespace state
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__STATE_TRANSLATION_HPP_
