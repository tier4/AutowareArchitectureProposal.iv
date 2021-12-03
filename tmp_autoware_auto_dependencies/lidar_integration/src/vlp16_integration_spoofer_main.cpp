// Copyright 2017-2018 the Autoware Foundation
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
#include <lidar_integration/vlp16_integration_spoofer.hpp>

#include <string>
#include <thread>
#include <chrono>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

int32_t main(const int32_t argc, char8_t ** const argv)
{
  using namespace std::chrono_literals;  // NOLINT

  int32_t ret;
  try {
    rclcpp::init(argc, argv);

    // common stuff
    const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--rpm");
    float32_t rpm = 600.0F;
    if (nullptr != arg) {
      rpm = std::stof(arg);
    }
    arg = rcutils_cli_get_option(argv, &argv[argc], "--runtime");
    uint32_t runtime = 30U;
    if (nullptr != arg) {
      runtime = static_cast<uint32_t>(std::stoul(arg));
    }
    // individual spoofers
    arg = rcutils_cli_get_option(argv, &argv[argc], "--ip1");
    const char8_t * ip1 = "127.0.0.1";
    if (nullptr != arg) {
      ip1 = arg;
    }
    arg = rcutils_cli_get_option(argv, &argv[argc], "--port1");
    uint16_t port1 = 5001U;
    if (nullptr != arg) {
      port1 = static_cast<uint16_t>(std::stoul(arg));
    }
    const bool8_t do_second_spoof =
      rcutils_cli_option_exist(argv, &argv[argc], "--do_second_spoof");
    arg = rcutils_cli_get_option(argv, &argv[argc], "--ip2");
    const char8_t * ip2 = "127.0.0.1";
    if (nullptr != arg) {
      ip2 = arg;
    }
    arg = rcutils_cli_get_option(argv, &argv[argc], "--port2");
    uint16_t port2 = 5002U;
    if (nullptr != arg) {
      port2 = static_cast<uint16_t>(std::stoul(arg));
    }
    const bool8_t do_third_spoof = rcutils_cli_option_exist(argv, &argv[argc], "--do_third_spoof");
    arg = rcutils_cli_get_option(argv, &argv[argc], "--ip3");
    const char8_t * ip3 = "127.0.0.1";
    if (nullptr != arg) {
      ip3 = arg;
    }
    arg = rcutils_cli_get_option(argv, &argv[argc], "--port3");
    uint16_t port3 = 5003U;
    if (nullptr != arg) {
      port3 = static_cast<uint16_t>(std::stoul(arg));
    }
    const bool8_t do_fourth_spoof =
      rcutils_cli_option_exist(argv, &argv[argc], "--do_fourth_spoof");
    arg = rcutils_cli_get_option(argv, &argv[argc], "--ip4");
    const char8_t * ip4 = "127.0.0.1";
    if (nullptr != arg) {
      ip4 = arg;
    }
    arg = rcutils_cli_get_option(argv, &argv[argc], "--port4");
    uint16_t port4 = 5004U;
    if (nullptr != arg) {
      port4 = static_cast<uint16_t>(std::stoul(arg));
    }
    // help
    bool8_t needs_help = rcutils_cli_option_exist(argv, &argv[argc], "-h");
    needs_help = rcutils_cli_option_exist(argv, &argv[argc], "--help") || needs_help;
    if (needs_help) {
      std::cout << "vlp16_integration_spoofer [OPTION VALUE [...]]" << std::endl;
      std::cout << "Usage:" << std::endl;
      std::cout << "OPTION" << std::endl;
      std::cout << "--rpm\tEquivalent rotation speed of spoofed packets\t" <<
        "Default=600" << std::endl;
      std::cout << "--runtime\tApproximate time this executable runs for(s)\t" <<
        "Default=30" << std::endl;
      std::cout << "--ip1\tTarget ip for the first spoofer\t" <<
        "Default=127.0.0.1" << std::endl;
      std::cout << "--port1\tTarget port for the first spoofer\t" <<
        "Default=5001" << std::endl;
      std::cout << "--do_second_spoof\tIf present, does first and second spoof" << std::endl;
      std::cout << "--ip2\tTarget ip for the second spoofer\t" <<
        "Default=127.0.0.1" << std::endl;
      std::cout << "--port2\tTarget port for the second spoofer\t" <<
        "Default=5001" << std::endl;
      std::cout << "--do_third_spoof\tIf present, does third spoof if second spoof" << std::endl;
      std::cout << "--ip3\tTarget ip for the third spoofer\t" <<
        "Default=127.0.0.1" << std::endl;
      std::cout << "--port3\tTarget port for the third spoofer\t" <<
        "Default=5003" << std::endl;
      std::cout << "--do_fourth_spoof\tIf present, does fourth spoof if second and third spoof" <<
        std::endl;
      std::cout << "--ip4\tTarget ip for the fourth spoofer\t" <<
        "Default=127.0.0.1" << std::endl;
      std::cout << "--port4\tTarget port for the fourth spoofer\t" <<
        "Default=5004" << std::endl;
      throw std::runtime_error{"Exiting due to help"};
    }

    ret = 0;

    if ((rpm < lidar_integration::Vlp16IntegrationSpoofer::MIN_RPM) ||
      (rpm > lidar_integration::Vlp16IntegrationSpoofer::MAX_RPM))
    {
      throw std::runtime_error("Invalid RPM");
    }
    lidar_integration::Vlp16IntegrationSpoofer spoof1(ip1, port1, rpm);
    lidar_integration::Vlp16IntegrationSpoofer spoof2(ip2, port2, rpm);
    lidar_integration::Vlp16IntegrationSpoofer spoof3(ip3, port3, rpm);
    lidar_integration::Vlp16IntegrationSpoofer spoof4(ip4, port4, rpm);

    spoof1.start();

    uint32_t spoofer_count = 1;
    if (do_second_spoof) {
      spoof2.start();
      spoofer_count++;
      if (do_third_spoof) {
        spoof3.start();
        spoofer_count++;
        if (do_fourth_spoof) {
          spoof4.start();
          spoofer_count++;
        }
      }
    }
    LIDAR_INTEGRATION_INFO("Spoofer(s) number is: %d", spoofer_count);
    // FIXME required for integration test due to buffered output
    std::cout << "Spoofer(s) number is: " << spoofer_count << ::std::endl;

    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(runtime);
    while (rclcpp::ok()) {
      if (end < std::chrono::steady_clock::now()) {
        break;
      }
      std::this_thread::sleep_for(1ms);
    }

    LIDAR_INTEGRATION_INFO("spoofer done");
    spoof1.stop();
    std::cout << spoof1.send_count() << " spoofed UDP packets were sent" << std::endl;
    if (do_second_spoof) {
      spoof2.stop();
      if (do_third_spoof) {
        spoof3.stop();
        if (do_fourth_spoof) {
          spoof4.stop();
        }
      }
    }
    LIDAR_INTEGRATION_INFO("Spoofer(s) finished.");
  } catch (const std::runtime_error & e) {
    LIDAR_INTEGRATION_ERROR("Got error: %s", e.what());
    ret = 2;
  } catch (...) {
    LIDAR_INTEGRATION_FATAL("Unknown error occured");
  }

  return ret;
}
