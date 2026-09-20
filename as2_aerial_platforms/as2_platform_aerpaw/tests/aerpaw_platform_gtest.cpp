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
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "aerpaw_platform.hpp"

namespace aerpaw_platform
{

std::shared_ptr<AerpawPlatform> get_node(
  const std::string & name_space = "aerpaw_platform")
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_aerpaw");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_params.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "-p",
    "ipc_cmd_port:=26010",
    "-p",
    "ipc_tel_port:=27010",
    "--params-file",
    platform_config_file,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<AerpawPlatform>(node_options);
}

TEST(AerpawPlatformGTest, Constructor) {
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_aerpaw");

  // The bridge binds fixed ports; skip if they are taken by a running instance.
  auto node = get_node();
  EXPECT_NO_THROW(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
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