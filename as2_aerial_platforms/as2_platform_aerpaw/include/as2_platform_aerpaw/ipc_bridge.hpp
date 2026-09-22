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
 * @file ipc_bridge.hpp
 * @brief UDP socket bridge between AS2 C++ platform and aerpawlib Python runner.
 */

#ifndef AS2_PLATFORM_AERPAW__IPC_BRIDGE_HPP_
#define AS2_PLATFORM_AERPAW__IPC_BRIDGE_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

namespace aerpaw_platform
{

/**
 * @brief AERPAW <-> Adapter wire-protocol version (IF-1, docs/INTERFACES.md).
 *
 * Carried as `"proto"` on every inbound message and asserted-compatible by the
 * runner's matching constant (see aerpaw_as2_runner.PROTOCOL_VERSION). Versioning
 * policy: MAJOR bump for incompatible schema change, MINOR for additive/backward-
 * compatible fields. Keep the C++ and Python constants in lockstep (guarded by
 * test_layering.test_protocol_version_consistent).
 */
constexpr int kProtocolVersion = 1;

/// Which AERPAW->AS2 message a telemetry-channel datagram carries.
enum class InboundKind
{
  NONE,        ///< Nothing received yet.
  TELEMETRY,   ///< Vehicle state (position/attitude/velocity).
  STATUS,      ///< Non-fatal status/report from the AERPAW side.
  ERROR,       ///< A failed command / AERPAW-side error report.
  MEASUREMENT, ///< Opaque AERPAW wireless/RF/network metrics for experiment use.
};

struct TelemetryState
{
  double lat = 0.0;
  double lon = 0.0;
  double alt_msl = 0.0;
  double rel_alt = 0.0;
  double vx_ned = 0.0;   // aerpawlib velocity.north (m/s)
  double vy_ned = 0.0;   // aerpawlib velocity.east  (m/s)
  double vz_ned = 0.0;   // aerpawlib velocity.down  (m/s)
  // Attitude (MAVSDK NED/FRD euler, radians). roll/pitch/yaw kept for heading;
  // the *_rad triple is the authoritative full attitude for orientation.
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;      // heading, degrees from North CW
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_ned_rad = 0.0;
  // GPS fix quality (aerpawlib gps.*).
  int gps_fix_type = 0;          // 0/1 no fix, 2 2D, 3 3D
  int gps_satellites = 0;
  // Battery (aerpawlib battery.*).
  double battery_voltage = 0.0;
  double battery_current = 0.0;
  double battery_level = 0.0;    // 0-100 %
  bool battery_valid = false;    // runner reported battery at all
  // Flight / health state.
  bool armed = false;
  bool armable = false;          // pre-arm checks pass
  bool ekf_ready = false;        // EKF ready for takeoff
  bool aerpaw_connected = false; // aerpawlib connected/link_alive
  std::string mode = "STANDBY";
  std::string vehicle_id;
  int mavlink_sysid = 0;  ///< AERPAW hardware-unique id (MAVLink system id).
  double ts = 0.0;
  bool connected = false;
  int proto = kProtocolVersion;
  InboundKind kind = InboundKind::NONE;
};

/// A status/error report pushed by the AERPAW runner (lifecycle / error propagation).
struct AdapterEvent
{
  InboundKind kind = InboundKind::NONE;   ///< STATUS or ERROR.
  std::string vehicle_id;
  std::string command;   ///< For ERROR: the command that failed.
  std::string message;
  double ts = 0.0;
  int proto = kProtocolVersion;
};

/// Opaque AERPAW wireless/RF/network measurement payload.
/// The adapter is a TRANSPORT only: it does NOT interpret the metrics (that is
/// experiment-layer work via experiment/rf_metrics.MeasurementSource). Keeping the
/// payload as a JSON string means no AERPAW-specific message type leaks into AS2.
struct AdapterMeasurement
{
  std::string vehicle_id;
  std::string payload_json;   ///< e.g. {"rssi_dbm":-65,"sinr_db":12,"throughput_mbps":48}
  double ts = 0.0;
  int proto = kProtocolVersion;
  bool valid = false;
};

class IpcBridge
{
public:
  IpcBridge(int cmd_port, int tel_port);
  ~IpcBridge();

  void start();
  void stop();
  /// True once start() has bound the sockets and launched the receiver thread.
  /// False if startup failed (e.g. telemetry port in use) -> the node must fail fast.
  bool isRunning() const {return running_;}

  bool sendCommand(const nlohmann::json & cmd);

  /// Latest telemetry snapshot (kind==NONE until the first packet arrives).
  TelemetryState getTelemetry() const;
  /// Latest status/error report, if any (kind==NONE if none seen).
  AdapterEvent getEvent() const;
  /// Latest AERPAW measurement payload (transport passthrough; valid==false if none).
  AdapterMeasurement getMeasurement() const;
  /// Seconds since the last telemetry packet (large if none). Link watchdog input.
  double telemetryAgeSeconds() const;

private:
  int cmd_port_;
  int tel_port_;
  int cmd_sock_ = -1;
  int tel_sock_ = -1;

  std::atomic<bool> running_{false};
  std::thread tel_thread_;

  mutable std::mutex tel_mutex_;
  TelemetryState latest_telemetry_;
  AdapterEvent latest_event_;
  AdapterMeasurement latest_measurement_;
  std::chrono::steady_clock::time_point last_tel_time_;
  bool ever_received_tel_ = false;
};

}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__IPC_BRIDGE_HPP_
