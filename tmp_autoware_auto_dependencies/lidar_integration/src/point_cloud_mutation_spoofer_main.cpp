// Copyright 2018 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <common/types.hpp>
#include <lidar_integration/lidar_integration_common.hpp>
#include <lidar_integration/point_cloud_mutation_spoofer.hpp>

#include <iostream>
#include <string>
#include <memory>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

int32_t main(const int32_t argc, char8_t ** const argv)
{
  rclcpp::init(argc, argv);

  std::stringstream help_msg;
  help_msg << "point_cloud_mutation_spoofer [OPTION VALUE [...]]" << std::endl;
  help_msg << "Usage:" << std::endl;
  help_msg << "OPTION" << std::endl;
  help_msg << "--mean\tMean of data size\t" <<
    "Default=3000000" << std::endl;
  const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--mean");
  uint32_t mean = 300000U;
  if (nullptr != arg) {
    mean = static_cast<uint32_t>(std::stoul(arg));
  }
  help_msg << "--std\tStandar deviation of data size\t" <<
    "Default=50000" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--std");
  uint32_t std = 50000U;
  if (nullptr != arg) {
    std = static_cast<uint32_t>(std::stoul(arg));
  }
  help_msg << "--w_mean\tMean of width\t" <<
    "Default=18000" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--w_mean");
  uint32_t w_mean = 18000U;
  if (nullptr != arg) {
    w_mean = static_cast<uint32_t>(std::stoul(arg));
  }
  help_msg << "--w_std\tStandard deviation of width.\t" <<
    "Default=1000" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--w_std");
  uint32_t w_std = 1000U;
  if (nullptr != arg) {
    w_std = static_cast<uint32_t>(std::stoul(arg));
  }
  help_msg << "--runtime\tTime of seconds to run the spoofer.\t" <<
    "Default=10U" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--runtime");
  uint32_t runtime = 10U;
  if (nullptr != arg) {
    runtime = static_cast<uint32_t>(std::stoul(arg));
  }
  help_msg << "--freq\tPublishing frequency.\t" <<
    "Default=1.0" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--freq");
  float32_t freq = 1.0F;
  if (nullptr != arg) {
    freq = std::stof(arg);
  }
  help_msg << "--topic\tPublishing topic of input pointcloud 2 message.\t" <<
    "points_nonground" << std::endl;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--topic");
  const char8_t * topic = "points_nonground";
  if (nullptr != arg) {
    topic = arg;
  }

  bool8_t needs_help = rcutils_cli_option_exist(argv, &argv[argc], "-h");
  needs_help = rcutils_cli_option_exist(argv, &argv[argc], "--help") || needs_help;
  if (needs_help) {
    std::cout << help_msg.str() << std::endl;
  }

  LIDAR_INTEGRATION_INFO("Spoofer arguments:");
  LIDAR_INTEGRATION_INFO("\tMean of data size: %d", mean);
  LIDAR_INTEGRATION_INFO("\tStd of data size: %d", std);
  LIDAR_INTEGRATION_INFO("\tMean of width: %d", w_mean);
  LIDAR_INTEGRATION_INFO("\tStd of width: %d", w_std);
  LIDAR_INTEGRATION_INFO("\tRuntime: %d", runtime);
  LIDAR_INTEGRATION_INFO("\tPublishing frequency: %f", static_cast<float64_t>(freq));

  const auto spoof = std::make_shared<lidar_integration::PointCloudMutationSpooferNode>(
    topic,
    mean,
    std,
    w_mean,
    w_std,
    freq
  );
  spoof->init();
  spoof->start();

  LIDAR_INTEGRATION_INFO("Spoofer started.");
  std::cout << "Spoofer(s) number is: 1" << std::endl;

  const auto end = std::chrono::seconds(static_cast<int64_t>(runtime)) +
    std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (std::chrono::steady_clock::now() > end) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10U));
  }

  spoof->stop();
  rclcpp::shutdown();
  LIDAR_INTEGRATION_INFO("Spoofer is done. ");
  return 0;
}
