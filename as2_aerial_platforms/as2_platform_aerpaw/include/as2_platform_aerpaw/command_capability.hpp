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
 * @file command_capability.hpp
 * @brief What the AERPAW side can ACTUALLY execute, per AS2 command/mode.
 *
 * Source of truth = aerpawlib's own declarations + the AERPAW E-VM command filter:
 *   - goto_coordinates (DO_REPOSITION) + takeoff + land + arm/disarm : supported.
 *   - set_velocity / offboard  : aerpawlib marks "[NOT SUPPORTED] ... blocked by
 *     the filter" on the testbed; works only in local SITL.
 *   - LOITER (action.hold)     : the filter SEVERS the link, so "hover" must NOT use
 *     it on the testbed - the adapter hovers via a supported reposition instead.
 *   - trajectory / attitude / body-rate / yaw-rate : no AERPAW offboard path at all.
 *
 * Pure (no ROS), so it is unit-tested and the "explicitly handle unsupported"
 * decision lives in exactly one place instead of being scattered in the node.
 */

#ifndef AS2_PLATFORM_AERPAW__COMMAND_CAPABILITY_HPP_
#define AS2_PLATFORM_AERPAW__COMMAND_CAPABILITY_HPP_

#include <cstdint>

#include "adapter_types.hpp"
#include "as2_msgs/msg/control_mode.hpp"

namespace aerpaw_platform
{
namespace capability
{

enum class Support
{
  SUPPORTED,             ///< AERPAW can execute it here.
  UNSUPPORTED_ON_TESTBED,///< Works in SITL only (aerpawlib filter blocks it).
  UNSUPPORTED_ALWAYS,    ///< AERPAW has no equivalent at all.
};

inline bool is_supported(Support s)
{
  return s == Support::SUPPORTED;
}

/// Human-readable reason for an unsupported command (for explicit handling/logs).
inline const char * reason(PlatformBackend backend, Support s)
{
  (void) backend;
  switch (s) {
    case Support::UNSUPPORTED_ON_TESTBED:
      return "AERPAW filter blocks velocity/offboard on the testbed (SITL only)";
    case Support::UNSUPPORTED_ALWAYS:
      return "AERPAW/aerpawlib provides no such command";
    default:
      return "supported";
  }
}

// Discrete lifecycle commands - all executable on AERPAW.
inline Support arm(PlatformBackend) {return Support::SUPPORTED;}
inline Support takeoff(PlatformBackend) {return Support::SUPPORTED;}
inline Support land(PlatformBackend) {return Support::SUPPORTED;}
inline Support kill(PlatformBackend) {return Support::SUPPORTED;}  // = disarm (documented)

// Continuous control modes.
inline Support position(PlatformBackend) {return Support::SUPPORTED;}  // DO_REPOSITION
inline Support yaw_angle(PlatformBackend) {return Support::SUPPORTED;} // via goto heading
inline Support hover(PlatformBackend) {return Support::SUPPORTED;}     // reposition/velocity
inline Support speed(PlatformBackend b)
{
  // Gated on AERPAW's ONE environment distinction: offboard/velocity is blocked
  // whenever the AERPAW environment (Digital Twin OR physical) is in play; it works
  // only off-AERPAW (local SITL). DT and physical are therefore capability-identical.
  return is_aerpaw_environment(b) ? Support::UNSUPPORTED_ON_TESTBED : Support::SUPPORTED;
}
inline Support trajectory(PlatformBackend) {return Support::UNSUPPORTED_ALWAYS;}
inline Support attitude(PlatformBackend) {return Support::UNSUPPORTED_ALWAYS;}
inline Support body_rates(PlatformBackend) {return Support::UNSUPPORTED_ALWAYS;}
inline Support yaw_speed(PlatformBackend) {return Support::UNSUPPORTED_ALWAYS;}

/**
 * @brief Capability of an AS2 control mode (control_mode + yaw_mode) on a backend.
 */
inline Support for_control_mode(
  PlatformBackend backend, int8_t control_mode, int8_t yaw_mode)
{
  using as2_msgs::msg::ControlMode;
  Support base;
  switch (control_mode) {
    case ControlMode::POSITION:
    case ControlMode::HOVER:
      base = Support::SUPPORTED;
      break;
    case ControlMode::SPEED:
    case ControlMode::SPEED_IN_A_PLANE:
      base = speed(backend);
      break;
    case ControlMode::TRAJECTORY:
      base = trajectory(backend);
      break;
    case ControlMode::ATTITUDE:
      base = attitude(backend);
      break;
    case ControlMode::BODY_RATES:
      base = body_rates(backend);
      break;
    default:
      base = Support::UNSUPPORTED_ALWAYS;
      break;
  }
  // Yaw-rate control needs offboard, unsupported even if the base mode is fine.
  if (yaw_mode == ControlMode::YAW_SPEED && is_supported(base)) {
    return yaw_speed(backend);
  }
  return base;
}

}  // namespace capability
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__COMMAND_CAPABILITY_HPP_
