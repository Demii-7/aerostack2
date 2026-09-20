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
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

namespace aerpaw_platform
{

struct TelemetryState
{
  double lat = 0.0;
  double lon = 0.0;
  double alt_msl = 0.0;
  double rel_alt = 0.0;
  double vx_ned = 0.0;
  double vy_ned = 0.0;
  double vz_ned = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  bool armed = false;
  bool connected = false;
  std::string mode = "STANDBY";
  double ts = 0.0;
};

class IpcBridge
{
public:
  IpcBridge(int cmd_port, int tel_port);
  ~IpcBridge();

  void start();
  void stop();

  bool sendCommand(const nlohmann::json & cmd);
  TelemetryState getTelemetry() const;

private:
  int cmd_port_;
  int tel_port_;
  int cmd_sock_ = -1;
  int tel_sock_ = -1;

  std::atomic<bool> running_{false};
  std::thread tel_thread_;

  mutable std::mutex tel_mutex_;
  TelemetryState latest_telemetry_;
};

}  // namespace aerpaw_platform

#endif  // AS2_PLATFORM_AERPAW__IPC_BRIDGE_HPP_
