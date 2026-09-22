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
 * @file ipc_bridge.cpp
 * @brief UDP socket bridge between AS2 C++ platform and aerpawlib Python runner.
 */

#include "ipc_bridge.hpp"

#include <fcntl.h>

#include <cstring>
#include <iostream>
#include <limits>
#include <thread>

namespace aerpaw_platform
{

IpcBridge::IpcBridge(int cmd_port, int tel_port)
: cmd_port_(cmd_port), tel_port_(tel_port)
{}

IpcBridge::~IpcBridge()
{
  stop();
}

void IpcBridge::start()
{
  // Telemetry socket: Python runner sends telemetry to C++
  tel_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (tel_sock_ < 0) {
    std::cerr << "[IpcBridge] Failed to create telemetry socket" << std::endl;
    return;
  }

  int tel_opt = 1;
  setsockopt(tel_sock_, SOL_SOCKET, SO_REUSEADDR, &tel_opt, sizeof(tel_opt));

  struct sockaddr_in tel_addr;
  memset(&tel_addr, 0, sizeof(tel_addr));
  tel_addr.sin_family = AF_INET;
  tel_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  tel_addr.sin_port = htons(tel_port_);

  if (bind(tel_sock_, reinterpret_cast<struct sockaddr *>(&tel_addr), sizeof(tel_addr)) < 0) {
    std::cerr << "[IpcBridge] Failed to bind telemetry socket on port " << tel_port_ << std::endl;
    close(tel_sock_);
    tel_sock_ = -1;
    return;
  }

  // Command socket: C++ sends commands to Python runner.
  // No need to bind to a specific port; the OS assigns an ephemeral source port.
  cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (cmd_sock_ < 0) {
    std::cerr << "[IpcBridge] Failed to create command socket" << std::endl;
    close(tel_sock_);
    tel_sock_ = -1;
    return;
  }

  // Set non-blocking for telemetry socket
  int flags = fcntl(tel_sock_, F_GETFL, 0);
  fcntl(tel_sock_, F_SETFL, flags | O_NONBLOCK);

  running_ = true;

  // Telemetry receiver thread
  tel_thread_ = std::thread([this]() {
    char buf[4096];
    while (running_) {
      struct sockaddr_in from;
      socklen_t fromlen = sizeof(from);
      ssize_t n = recvfrom(tel_sock_, buf, sizeof(buf) - 1, 0,
        reinterpret_cast<struct sockaddr *>(&from), &fromlen);
      if (n > 0) {
        buf[n] = '\0';
        try {
          auto j = nlohmann::json::parse(buf);
          const std::string type = j.value("type", std::string("telemetry"));

          if (type == "status" || type == "error") {
            AdapterEvent ev;
            ev.kind = (type == "error") ? InboundKind::ERROR : InboundKind::STATUS;
            ev.vehicle_id = j.value("vehicle_id", std::string(""));
            ev.command = j.value("command", std::string(""));
            ev.message = j.value("message", std::string(""));
            ev.ts = j.value("ts", 0.0);
            ev.proto = j.value("proto", kProtocolVersion);
            std::lock_guard<std::mutex> lock(tel_mutex_);
            latest_event_ = ev;
          } else if (type == "measurement") {
            // Transport passthrough: keep the metrics object verbatim (JSON string).
            AdapterMeasurement m;
            m.vehicle_id = j.value("vehicle_id", std::string(""));
            m.ts = j.value("ts", 0.0);
            m.proto = j.value("proto", kProtocolVersion);
            m.payload_json = j.contains("metrics") ? j["metrics"].dump() : std::string("{}");
            m.valid = true;
            std::lock_guard<std::mutex> lock(tel_mutex_);
            latest_measurement_ = m;
          } else {
            TelemetryState tel;
            tel.kind = InboundKind::TELEMETRY;
            tel.lat = j.value("lat", 0.0);
            tel.lon = j.value("lon", 0.0);
            tel.alt_msl = j.value("alt_msl", 0.0);
            tel.rel_alt = j.value("rel_alt", 0.0);
            tel.vx_ned = j.value("vx_ned", 0.0);
            tel.vy_ned = j.value("vy_ned", 0.0);
            tel.vz_ned = j.value("vz_ned", 0.0);
            tel.roll = j.value("roll", 0.0);
            tel.pitch = j.value("pitch", 0.0);
            tel.yaw = j.value("yaw", 0.0);
            tel.roll_rad = j.value("roll_rad", 0.0);
            tel.pitch_rad = j.value("pitch_rad", 0.0);
            tel.yaw_ned_rad = j.value("yaw_ned_rad", 0.0);
            tel.gps_fix_type = j.value("gps_fix_type", 0);
            tel.gps_satellites = j.value("gps_satellites", 0);
            tel.battery_voltage = j.value("battery_voltage", 0.0);
            tel.battery_current = j.value("battery_current", 0.0);
            tel.battery_level = j.value("battery_level", 0.0);
            tel.battery_valid = j.find("battery_voltage") != j.end();
            tel.armed = j.value("armed", false);
            tel.armable = j.value("armable", false);
            tel.ekf_ready = j.value("ekf_ready", false);
            tel.aerpaw_connected = j.value("connected", false);
            tel.mode = j.value("mode", "UNKNOWN");
            tel.vehicle_id = j.value("vehicle_id", std::string(""));
            tel.mavlink_sysid = j.value("mavlink_sysid", 0);
            tel.ts = j.value("ts", 0.0);
            tel.proto = j.value("proto", kProtocolVersion);
            tel.connected = true;

            std::lock_guard<std::mutex> lock(tel_mutex_);
            latest_telemetry_ = tel;
            last_tel_time_ = std::chrono::steady_clock::now();
            ever_received_tel_ = true;
          }
        } catch (const std::exception & e) {
          std::cerr << "[IpcBridge] Telemetry parse error: " << e.what() << std::endl;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  });
}

void IpcBridge::stop()
{
  running_ = false;
  if (tel_thread_.joinable()) {
    tel_thread_.join();
  }
  if (cmd_sock_ >= 0) {close(cmd_sock_); cmd_sock_ = -1;}
  if (tel_sock_ >= 0) {close(tel_sock_); tel_sock_ = -1;}
}

bool IpcBridge::sendCommand(const nlohmann::json & cmd)
{
  if (cmd_sock_ < 0) {return false;}

  std::string msg = cmd.dump() + "\n";

  struct sockaddr_in runner_addr;
  memset(&runner_addr, 0, sizeof(runner_addr));
  runner_addr.sin_family = AF_INET;
  runner_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  runner_addr.sin_port = htons(cmd_port_);

  ssize_t sent = sendto(cmd_sock_, msg.c_str(), msg.size(), 0,
    reinterpret_cast<struct sockaddr *>(&runner_addr), sizeof(runner_addr));

  return sent > 0;
}

TelemetryState IpcBridge::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(tel_mutex_);
  return latest_telemetry_;
}

AdapterEvent IpcBridge::getEvent() const
{
  std::lock_guard<std::mutex> lock(tel_mutex_);
  return latest_event_;
}

AdapterMeasurement IpcBridge::getMeasurement() const
{
  std::lock_guard<std::mutex> lock(tel_mutex_);
  return latest_measurement_;
}

double IpcBridge::telemetryAgeSeconds() const
{
  std::lock_guard<std::mutex> lock(tel_mutex_);
  if (!ever_received_tel_) {
    return std::numeric_limits<double>::infinity();
  }
  const auto dt = std::chrono::steady_clock::now() - last_tel_time_;
  return std::chrono::duration<double>(dt).count();
}

}  // namespace aerpaw_platform
