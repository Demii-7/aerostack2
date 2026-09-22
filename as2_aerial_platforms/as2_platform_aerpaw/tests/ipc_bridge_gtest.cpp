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
 * @file ipc_bridge_gtest.cpp
 * @brief Tests for the IPC bridge between the C++ platform and Python runner.
 */

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>

#include "ipc_bridge.hpp"

namespace aerpaw_platform
{

class IpcBridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Pick distinct ports to avoid clashes
    cmd_port_ = 26000;
    tel_port_ = 27000;
  }

  int cmd_port_;
  int tel_port_;
};

TEST_F(IpcBridgeTest, SendCommandReachesListener)
{
  // Listener socket on cmd_port (where the Python runner binds)
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(sock, 0);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(cmd_port_);

  ASSERT_EQ(bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)), 0);

  {
    IpcBridge bridge(cmd_port_, tel_port_);
    bridge.start();

    nlohmann::json cmd = {{"cmd", "arm"}};
    EXPECT_TRUE(bridge.sendCommand(cmd));

    char buf[1024];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
      reinterpret_cast<struct sockaddr *>(&from), &fromlen);
    ASSERT_GT(n, 0);
    buf[n] = '\0';

    nlohmann::json parsed = nlohmann::json::parse(buf);
    EXPECT_EQ(parsed["cmd"], "arm");

    bridge.stop();
  }
  close(sock);
}

TEST_F(IpcBridgeTest, TelemetryReceivedAndStored)
{
  IpcBridge bridge(cmd_port_, tel_port_);
  bridge.start();

  // Unsolicited telemetry packet to the bridge's tel port
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(sock, 0);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(tel_port_);

  nlohmann::json tel = {
    {"lat", 35.727}, {"lon", -78.698}, {"alt_msl", 85.0}, {"rel_alt", 25.1},
    {"vx_ned", 0.1}, {"vy_ned", 0.0}, {"vz_ned", 0.0},
    {"roll", 0.0}, {"pitch", 0.01}, {"yaw", 1.57},
    {"armed", true}, {"mode", "GUIDED"}, {"ts", 1234567890.123}
  };
  std::string msg = tel.dump();
  sendto(sock, msg.c_str(), msg.size(), 0,
    reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));

  // Give the receiver thread a chance to process
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  TelemetryState state = bridge.getTelemetry();
  EXPECT_TRUE(state.connected);
  EXPECT_DOUBLE_EQ(state.lat, 35.727);
  EXPECT_DOUBLE_EQ(state.rel_alt, 25.1);
  EXPECT_TRUE(state.armed);
  EXPECT_EQ(state.mode, "GUIDED");

  bridge.stop();
  close(sock);
}

TEST_F(IpcBridgeTest, MeasurementIsTransportedVerbatim)
{
  IpcBridge bridge(cmd_port_, tel_port_);
  bridge.start();

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(sock, 0);
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(tel_port_);

  nlohmann::json m = {
    {"type", "measurement"}, {"vehicle_id", "drone0"},
    {"metrics", {{"sinr_db", 18.0}, {"rssi_dbm", -62.0}, {"throughput_mbps", 47.5}}},
    {"ts", 555.0}
  };
  std::string msg = m.dump();
  sendto(sock, msg.c_str(), msg.size(), 0,
    reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  AdapterMeasurement meas = bridge.getMeasurement();
  EXPECT_TRUE(meas.valid);
  EXPECT_EQ(meas.vehicle_id, "drone0");
  // payload is passed through as raw JSON, not interpreted by the adapter
  EXPECT_NE(meas.payload_json.find("sinr_db"), std::string::npos);
  EXPECT_NE(meas.payload_json.find("throughput_mbps"), std::string::npos);

  bridge.stop();
  close(sock);
}

}  // namespace aerpaw_platform

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}