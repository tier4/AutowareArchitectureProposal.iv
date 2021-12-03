// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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

#include <common/types.hpp>
#include <signal.h>
#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <memory>
#include "lidar_integration/lidar_integration_listener.hpp"
#include "lidar_lc_integration_listener.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

enum class ListenerType : uint8_t
{
  CLOUD = 0,
  BOX,
};

int32_t main(const int32_t argc, char8_t ** const argv)
{
  int32_t ret;
  try {
    rclcpp::init(argc, argv);

    LIDAR_INTEGRATION_INFO("listener pre-parse");
    std::stringstream help_msg;
    help_msg << "vlp16_integration_spoofer [OPTION VALUE [...]]" << std::endl;
    help_msg << "Usage:" << std::endl;
    help_msg << "OPTION" << std::endl;
    // Type
    help_msg << "--type\tType of listener: 'cloud' or 'box'\t" <<
      "Default=\"\"" << std::endl;
    const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--type");
    std::string type_str;
    if (nullptr != arg) {
      type_str = arg;
    }
    ListenerType type = ListenerType::CLOUD;
    if (0 == type_str.compare("box")) {
      type = ListenerType::BOX;
    } else if (0 == type_str.compare("cloud")) {
      type = ListenerType::CLOUD;
    } else {
      throw std::domain_error{"Incorrect listener type"};
    }
    // Topics
    help_msg << "--topic\tname of input topic\t" <<
      "Default=\"\"" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--topic");
    const char8_t * topic = "";
    if (nullptr != arg) {
      topic = arg;
    }
    // Sizes
    help_msg << "--size\texpected size of input topic\t" <<
      "Default=0" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--size");
    uint32_t size = 0U;
    if (nullptr != arg) {
      size = static_cast<uint32_t>(std::stoul(arg));
    }
    // etc
    help_msg << "--period\texpected period between the messages (ms)\t" <<
      "Default=100" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--period");
    float32_t period = 100.0F;
    if (nullptr != arg) {
      period = std::stof(arg);
    }
    help_msg << "--period_tolerance\trelative tolerance for periodicity\t" <<
      "Default=0.1" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--period_tolerance");
    float32_t period_tolerance = 0.1F;
    if (nullptr != arg) {
      period_tolerance = std::stof(arg);
    }
    help_msg << "--size_tolerance\trelative tolerance for size\t" <<
      "Default=0.1" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--size_tolerance");
    float32_t size_tolerance = 0.1F;
    if (nullptr != arg) {
      size_tolerance = std::stof(arg);
    }
    help_msg << "--runtime\truntime of this listener (s)\t" <<
      "Default=10" << std::endl;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--runtime");
    float32_t runtime = 10.0F;
    if (nullptr != arg) {
      runtime = std::stof(arg);
    }
    help_msg << "--lifecycle_node\tif present, will assume this is a lifecycle node\t" << std::endl;
    const bool8_t lifecycle_node = rcutils_cli_option_exist(argv, &argv[argc], "--lifecycle_node");
    bool8_t needs_help = rcutils_cli_option_exist(argv, &argv[argc], "-h");
    needs_help = rcutils_cli_option_exist(argv, &argv[argc], "--help") || needs_help;
    if (needs_help) {
      std::cout << help_msg.str() << std::endl;
      throw std::runtime_error{"Exiting due to help"};
    }
    LIDAR_INTEGRATION_INFO("Lidar integration listener parsed arguments. ");
    LIDAR_INTEGRATION_INFO("listener post-parse");
    using lidar_integration::LidarIntegrationListener;
    std::shared_ptr<LidarIntegrationListener> nd_ptr;
    switch (type) {
      case ListenerType::BOX:
        using lidar_integration::LidarIntegrationBoxListener;
        nd_ptr = std::make_shared<LidarIntegrationBoxListener>(
          topic,
          period,
          size,
          period_tolerance,
          size_tolerance
        );
        break;
      case ListenerType::CLOUD:
        using lidar_integration::LidarIntegrationPclListener;
        nd_ptr = std::make_shared<LidarIntegrationPclListener>(
          topic,
          period,
          size,
          period_tolerance,
          size_tolerance
        );
        break;
      default:
        throw std::logic_error{"Impossible case"};
    }
    LIDAR_INTEGRATION_INFO("listener construct");
    rclcpp::executors::SingleThreadedExecutor exec;

    LIDAR_INTEGRATION_INFO("lifecycle node: %s", lifecycle_node ? "true" : "false");
    exec.add_node(nd_ptr);

    // if it is a lifecycle node
    std::shared_ptr<lidar_integration::Lcm> lcm_client;
    if (lifecycle_node) {
      LIDAR_INTEGRATION_INFO("Lidar integration test starts a lifecycle node.");

      // add lcm client to executor
      lcm_client = std::make_shared<lidar_integration::Lcm>("lidar_integration_test_lcm");
      lcm_client->init();
      exec.add_node(lcm_client->get_node_base_interface());

      // start the threads
      lcm_client->start();
    }

    const auto end =
      std::chrono::steady_clock::now() + std::chrono::seconds(static_cast<int32_t>(runtime));
    LIDAR_INTEGRATION_INFO("Lidar integration test listener started.");

    while (rclcpp::ok()) {
      exec.spin_once(std::chrono::milliseconds(100LL));
      if (std::chrono::steady_clock::now() > end) {
        break;
      }
    }
    LIDAR_INTEGRATION_INFO("Listener done");
    LIDAR_INTEGRATION_INFO("Lidar integration test listener is done.");
    ret = 0;
    if (!nd_ptr->is_success()) {
      ret = ret + (1 << 0U);
      LIDAR_INTEGRATION_FATAL("failed");
    }
    if (0 == ret) {
      LIDAR_INTEGRATION_INFO("CONSOLE_SUCCESS");
      // This is to ensure logging has enough time to output everything.
      std::this_thread::sleep_for(std::chrono::seconds(1));
      printf("success\n");
    } else {
      LIDAR_INTEGRATION_FATAL("CONSOFAILURE");
      // This is to ensure logging has enough time to output everything.
      std::this_thread::sleep_for(std::chrono::seconds(1));
      printf("failure\n");
    }
  } catch (const std::exception & e) {
    LIDAR_INTEGRATION_FATAL("Listener: Got error: %s", e.what());
    ret = __LINE__;
  } catch (...) {
    LIDAR_INTEGRATION_FATAL("Listener: Got unknown error");
    ret = __LINE__;
  }

  // Logging relies on DDS: Shutdown should happen last
  (void)rclcpp::shutdown();
  return ret;
}
