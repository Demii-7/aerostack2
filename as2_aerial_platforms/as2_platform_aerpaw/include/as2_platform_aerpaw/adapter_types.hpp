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
 * @file adapter_types.hpp
 * @brief AERPAW-specific types for the adapter, kept isolated from AeroStack2 core.
 *
 * Nothing here is added to as2_msgs / as2_core. These are internal to
 * as2_platform_aerpaw so that AERPAW concepts (Digital Twin vs physical testbed,
 * the geographic home origin) do not leak into the wider AeroStack2 codebase.
 */

#ifndef AS2_PLATFORM_AERPAW__ADAPTER_TYPES_HPP_
#define AS2_PLATFORM_AERPAW__ADAPTER_TYPES_HPP_

#include <string>

namespace aerpaw_platform
{

/**
 * @brief What kind of AERPAW side this platform is bridged to.
 *
 * This label MIRRORS AERPAW's own environment abstraction; it does NOT introduce a
 * parallel one. aerpawlib has exactly ONE environment distinction: whether the AERPAW
 * OEO forward server is reachable (`AerpawPlatform` / `ping_forward_server`, surfaced on
 * the CLI as `--no-aerpaw-environment`). Both Digital Twin and physical testbed ARE the
 * AERPAW environment - aerpawlib's `Drone` API is byte-identical between them and the
 * only difference is which E-VM address `--conn` points at (10.14.x.x vs 192.168.32.x),
 * which is invisible to the robotics integration.
 *
 * Therefore capabilities gate on `is_aerpaw_environment()` (DT ≡ physical), and the sole
 * DT-vs-physical distinction here is the safety posture (`is_real_hardware`), which
 * changes no command, only a warning log. Switching an experiment between DT and physical
 * requires NO code change - only the connection endpoints + this label.
 */
enum class PlatformBackend
{
  SITL,          ///< Local ArduPilot SITL (development; NOT the AERPAW environment).
  DIGITAL_TWIN,  ///< AERPAW Digital Twin (AERPAW env, virtual E-VMs, 10.14.x.x).
  PHYSICAL,      ///< AERPAW physical testbed (AERPAW env, real vehicles, 192.168.32.x).
};

/// Parse a backend name ("sitl" | "digital_twin" | "dt" | "physical" | "testbed").
inline PlatformBackend backend_from_string(const std::string & s)
{
  if (s == "digital_twin" || s == "dt") {
    return PlatformBackend::DIGITAL_TWIN;
  }
  if (s == "physical" || s == "testbed") {
    return PlatformBackend::PHYSICAL;
  }
  return PlatformBackend::SITL;
}

/// Human-readable backend name (for logs / platform_info).
inline const char * backend_name(PlatformBackend b)
{
  switch (b) {
    case PlatformBackend::DIGITAL_TWIN:
      return "digital_twin";
    case PlatformBackend::PHYSICAL:
      return "physical";
    default:
      return "sitl";
  }
}

/// True when commands drive real hardware - used to gate safety-relevant notes.
/// This is a SAFETY nuance within the AERPAW environment; it changes no capability.
inline bool is_real_hardware(PlatformBackend b)
{
  return b == PlatformBackend::PHYSICAL;
}

/// AERPAW's single environment abstraction, mirrored: Digital Twin and physical are the
/// SAME AERPAW environment (OEO forward server present); SITL is not. Capabilities gate
/// on this, so DT and physical behave identically for the robotics integration.
inline bool is_aerpaw_environment(PlatformBackend b)
{
  return b == PlatformBackend::DIGITAL_TWIN || b == PlatformBackend::PHYSICAL;
}

/**
 * @brief The vehicle's GPS home = the origin of the local ENU odom frame.
 * Captured from the first disarmed telemetry, exactly as before.
 */
struct GeoHome
{
  double lat{0.0};
  double lon{0.0};
  double alt_msl{0.0};
  bool set{false};
};

}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__ADAPTER_TYPES_HPP_
