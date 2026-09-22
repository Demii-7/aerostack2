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
 * @file vehicle_identity.hpp
 * @brief Multi-UAV identification + IPC-address mapping for the AERPAW adapter.
 *
 * Centralises how one AERPAW vehicle maps onto one AeroStack2 platform: the ROS
 * namespace, the per-drone UDP IPC ports, and the AERPAW vehicle id. Previously
 * the `+i*2` port and `drone{i}` rules were implicit/duplicated across launch and
 * node; here they are one source of truth, so the multi-UAV mapping lives only in
 * the adapter and stays out of the AeroStack2 core.
 */

#ifndef AS2_PLATFORM_AERPAW__VEHICLE_IDENTITY_HPP_
#define AS2_PLATFORM_AERPAW__VEHICLE_IDENTITY_HPP_

#include <set>
#include <string>
#include <vector>

#include "adapter_types.hpp"

namespace aerpaw_platform
{
namespace identity
{

/// First command port (drone0); matches the platform_params.yaml default.
constexpr int kDefaultCmdPortBase = 15760;
/// First telemetry port (drone0).
constexpr int kDefaultTelPortBase = 15761;
/// Port spacing between drones (each drone uses one cmd + one tel port).
constexpr int kPortStride = 2;
/// MAVLink UDP base port / per-drone offset (SITL convention).
constexpr int kMavlinkPortBase = 14550;
constexpr int kMavlinkPortStride = 10;

/**
 * @brief The identity of a single vehicle within a (possibly multi-UAV) experiment.
 *
 * One of these == one AERPAW UAV == one AeroStack2 platform. It is the single
 * source of truth for the AERPAW<->AS2 mapping: the AERPAW side (conn / MAVLink
 * system id) and the AS2 side (namespace = the platform's ROS identity, and the
 * UDP IPC pair that reaches exactly this one platform).
 */
struct VehicleIdentity
{
  /// AERPAW-side id (e.g. the OEO portable-node name); empty = use index fallback.
  std::string vehicle_id;
  /// Zero-based drone index in the experiment.
  int drone_index{0};
  /// ROS 2 namespace (e.g. "drone0"); the AS2-side unique platform identity.
  std::string ns;
  /// UDP command port this platform sends AS2->AERPAW commands to.
  int cmd_port{kDefaultCmdPortBase};
  /// UDP telemetry port this platform receives AERPAW->AS2 telemetry on.
  int tel_port{kDefaultTelPortBase};
  /// AERPAW MAVLink endpoint (e.g. udpin://127.0.0.1:14550 or a real E-VM).
  std::string aerpaw_conn;
  /// AERPAW MAVLink system id, if known; the hardware-unique vehicle id.
  int mavlink_sysid{0};
  /// Which AERPAW side this maps to.
  PlatformBackend backend{PlatformBackend::SITL};
};

/// Default ROS namespace for a drone index ("drone0", "drone1", ...).
inline std::string default_namespace(int drone_index)
{
  return "drone" + std::to_string(drone_index);
}

/// Command UDP port for a drone index given a base (drone0 -> base).
inline int cmd_port_for(int drone_index, int base = kDefaultCmdPortBase)
{
  return base + drone_index * kPortStride;
}

/// Telemetry UDP port for a drone index given a base (drone0 -> base + 1).
inline int tel_port_for(int drone_index, int base = kDefaultTelPortBase)
{
  return base + drone_index * kPortStride;
}

/// Default SITL MAVLink endpoint for a drone index (local convention).
inline std::string default_conn(int drone_index)
{
  return "udpin://127.0.0.1:" + std::to_string(kMavlinkPortBase + drone_index * kMavlinkPortStride);
}

/**
 * @brief Build the vehicle map for a fleet of N AERPAW UAVs -> N AS2 platforms.
 *
 * Assigns each index a unique namespace, unique id (explicit `vehicle_ids` when
 * supplied, else `drone{i}`), a unique IPC port pair and a MAVLink conn
 * (explicit `conns` when supplied, else the SITL default). This is what lets many
 * AERPAW UAVs coexist simultaneously while each is reached through the standard
 * as2::AerialPlatform abstraction.
 */
inline std::vector<VehicleIdentity> build_vehicle_map(
  size_t num_drones,
  const std::vector<std::string> & vehicle_ids = {},
  const std::vector<std::string> & conns = {},
  PlatformBackend backend = PlatformBackend::SITL,
  int cmd_base = kDefaultCmdPortBase,
  int tel_base = kDefaultTelPortBase)
{
  std::vector<VehicleIdentity> map;
  map.reserve(num_drones);
  for (size_t i = 0; i < num_drones; ++i) {
    VehicleIdentity v;
    v.drone_index = static_cast<int>(i);
    v.ns = default_namespace(v.drone_index);
    v.vehicle_id = (i < vehicle_ids.size() && !vehicle_ids[i].empty()) ?
      vehicle_ids[i] : v.ns;
    v.cmd_port = cmd_port_for(v.drone_index, cmd_base);
    v.tel_port = tel_port_for(v.drone_index, tel_base);
    v.aerpaw_conn = (i < conns.size() && !conns[i].empty()) ?
      conns[i] : default_conn(v.drone_index);
    v.backend = backend;
    map.push_back(v);
  }
  return map;
}

/**
 * @brief Verify a fleet map has no identity or port collisions.
 * @return empty string if valid, else a human-readable reason.
 */
inline std::string validate_unique(const std::vector<VehicleIdentity> & map)
{
  std::set<std::string> namespaces, ids;
  std::set<int> ports;
  for (const auto & v : map) {
    if (!namespaces.insert(v.ns).second) {
      return "duplicate namespace '" + v.ns + "'";
    }
    if (!ids.insert(v.vehicle_id).second) {
      return "duplicate vehicle_id '" + v.vehicle_id + "'";
    }
    if (v.cmd_port == v.tel_port) {
      return "cmd/tel port collide for " + v.ns;
    }
    if (!ports.insert(v.cmd_port).second) {
      return "duplicate cmd port " + std::to_string(v.cmd_port);
    }
    if (!ports.insert(v.tel_port).second) {
      return "duplicate tel port " + std::to_string(v.tel_port);
    }
  }
  return {};
}

}  // namespace identity
}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__VEHICLE_IDENTITY_HPP_
