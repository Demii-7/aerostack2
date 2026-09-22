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
 * @file aerpaw_platform_gtest.cpp
 * @brief AERPAW platform node gtest.
 */

#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "aerpaw_platform.hpp"

namespace aerpaw_platform
{

std::shared_ptr<AerpawPlatform> get_node(
  const std::string & name_space = "aerpaw_platform",
  int cmd_port = 26010, int tel_port = 27010)
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_aerpaw");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_params.yaml";

  // NOTE: rclcpp applies parameter sources in order, later wins. Put the shared
  // params-file FIRST and the per-node overrides AFTER so each platform keeps its
  // own unique namespace + IPC ports (otherwise the file's 15760/15761 would make
  // several nodes collide on the same socket).
  std::vector<std::string> node_args = {
    "--ros-args",
    "--params-file",
    platform_config_file,
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "-p",
    "ipc_cmd_port:=" + std::to_string(cmd_port),
    "-p",
    "ipc_tel_port:=" + std::to_string(tel_port),
    "-p",
    "vehicle_id:=" + name_space,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<AerpawPlatform>(node_options);
}

TEST(AerpawPlatformGTest, Constructor) {
  // The bridge binds fixed ports; skip if they are taken by a running instance.
  auto node = get_node();
  EXPECT_NO_THROW(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

// One AERPAW UAV == one AeroStack2 platform. Verify several coexist at once, each
// a distinct namespaced as2::AerialPlatform reached through the normal abstraction
// (Stage 4). Distinct IPC ports keep their UDP sockets from colliding.
TEST(AerpawPlatformGTest, MultiplePlatformsCoexist) {
  std::vector<std::shared_ptr<AerpawPlatform>> nodes;
  rclcpp::executors::MultiThreadedExecutor executor;
  for (int i = 0; i < 3; ++i) {
    const std::string ns = "drone" + std::to_string(i);
    auto node = get_node(ns, 26010 + i * 2, 27010 + i * 2);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->get_namespace(), "/" + ns);  // unique AS2 platform identity
    executor.add_node(node);
    nodes.push_back(node);
  }
  executor.spin_some();
  for (auto & n : nodes) {
    executor.remove_node(n);
  }
}

// Stage 12: multi-UAV isolation. Each vehicle's state feed and command input must
// be exclusively its own (namespaced), so a command to droneA can never be heard by
// droneB and no two vehicles share a state/command endpoint (no cross-control).
TEST(AerpawPlatformGTest, MultiUavNamespaceIsolation) {
  const int N = 3;
  std::vector<std::shared_ptr<AerpawPlatform>> nodes;
  rclcpp::executors::MultiThreadedExecutor executor;
  for (int i = 0; i < N; ++i) {
    auto node = get_node("drone" + std::to_string(i), 26110 + i * 2, 27110 + i * 2);
    executor.add_node(node);
    nodes.push_back(node);
  }
  executor.spin_some();
  auto * g = nodes[0].get();  // any node sees the full discovered graph

  for (int i = 0; i < N; ++i) {
    const std::string ns = "/drone" + std::to_string(i);

    // (1) Independent state interface: only THIS vehicle's node publishes its odom.
    auto pub = g->get_publishers_info_by_topic(ns + "/sensor_measurements/odom");
    ASSERT_FALSE(pub.empty()) << ns << " odom has no publisher";
    for (const auto & p : pub) {
      EXPECT_EQ(p.node_namespace(), ns) << "cross-control: " << ns << " odom published by "
                                        << p.node_namespace();
    }

    // (2) Independent command interface: only THIS vehicle's node listens to its pose cmd.
    auto sub = g->get_subscriptions_info_by_topic(ns + "/actuator_command/pose");
    ASSERT_FALSE(sub.empty()) << ns << " command topic has no subscriber";
    for (const auto & s : sub) {
      EXPECT_EQ(s.node_namespace(), ns) << "cross-control: " << ns << " command read by "
                                        << s.node_namespace();
    }
  }

  // (3) The per-vehicle topic namespaces are pairwise disjoint (prefix sets differ).
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      const std::string pi = "/drone" + std::to_string(i) + "/";
      const std::string pj = "/drone" + std::to_string(j) + "/";
      for (const auto & kv : g->get_topic_names_and_types()) {
        bool in_i = kv.first.rfind(pi, 0u) == 0u;
        bool in_j = kv.first.rfind(pj, 0u) == 0u;
        EXPECT_FALSE(in_i && in_j) << "topic shared across vehicles: " << kv.first;
      }
    }
  }

  for (auto & n : nodes) {
    executor.remove_node(n);
  }
}

}  // namespace aerpaw_platform

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}