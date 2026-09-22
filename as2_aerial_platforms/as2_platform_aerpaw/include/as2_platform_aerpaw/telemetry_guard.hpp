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
 * @file telemetry_guard.hpp
 * @brief Pure input-validation helpers so malformed / stale data never silently
 * propagates from AERPAW into AeroStack2 (Stage 17 - Handle Failures Safely).
 *
 * Stateless and ROS-free so it is unit-testable without a vehicle.
 */

#ifndef AS2_PLATFORM_AERPAW__TELEMETRY_GUARD_HPP_
#define AS2_PLATFORM_AERPAW__TELEMETRY_GUARD_HPP_

#include <cmath>

#include "frame_conversions.hpp"
#include "ipc_bridge.hpp"

namespace aerpaw_platform
{
namespace guard
{

/// A telemetry sample is usable only if its numbers are finite and plausible.
/// Rejects NaN/Inf, out-of-range lat/lon, the (0,0) "no fix" sentinel, and
/// non-finite attitude, so we never publish a garbage odom/gps/imu.
inline bool telemetry_is_valid(const TelemetryState & t)
{
  if (!std::isfinite(t.lat) || !std::isfinite(t.lon) ||
    !std::isfinite(t.alt_msl) || !std::isfinite(t.rel_alt))
  {
    return false;
  }
  if (std::fabs(t.lat) > 90.0 || std::fabs(t.lon) > 180.0) {
    return false;
  }
  if (t.lat == 0.0 && t.lon == 0.0) {  // MAVLink "position not known" sentinel
    return false;
  }
  if (!std::isfinite(t.vx_ned) || !std::isfinite(t.vy_ned) || !std::isfinite(t.vz_ned)) {
    return false;
  }
  if (!std::isfinite(t.roll_rad) || !std::isfinite(t.pitch_rad) ||
    !std::isfinite(t.yaw_ned_rad))
  {
    return false;
  }
  return true;
}

/// A commanded reference is only forwarded if finite (guards malformed setpoints).
inline bool command_is_valid(const frames::Vec3 & pos, double yaw)
{
  return std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z) &&
    std::isfinite(yaw);
}

/// Freshness: a packet is stale if it is older than `timeout_s` (measured by the
/// caller from the receive clock) OR its timestamp has not advanced.
inline bool telemetry_is_fresh(double age_s, double timeout_s, double ts, double last_ts)
{
  return age_s <= timeout_s && ts > last_ts;
}

}  // namespace guard
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__TELEMETRY_GUARD_HPP_
