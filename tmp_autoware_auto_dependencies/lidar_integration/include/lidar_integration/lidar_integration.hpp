// Copyright 2017-2019 the Autoware Foundation
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

#ifndef LIDAR_INTEGRATION__LIDAR_INTEGRATION_HPP_
#define LIDAR_INTEGRATION__LIDAR_INTEGRATION_HPP_

#include <lidar_integration/lidar_integration_listener.hpp>
#include <lidar_integration/vlp16_integration_spoofer.hpp>

#include <memory>
#include <vector>

namespace lidar_integration
{

using autoware::common::types::bool8_t;

/// \brief Runs a simple integration test: spoofs vlp16 data, runs test nodes, and listener.
///        Normal nodes are run using a single-threaded exuecutor for simplicity
/// \return True if frequency and size conditions of listener are satisfied
/// \param[in] start_test_nodes Function to start the nodes to be tested
/// \param[in] stop_test_nodes Function to stop the nodes to be tested
/// \param[in] udp_spoofers The vlp16 spoofers to run, appropriately configured for the problem
/// \param[in] listeners The appropriately configured listeners for the given test case
/// \param[in] runtime The runtime of the test. Actualy runtime is a bit longer to account for
///                    threading etc.
/// \param[in] nodes Normal nodes to also run during test, included for compatibility, generally
///                  not recommended to use. These get registered with a single-threaded executor
bool8_t LIDAR_INTEGRATION_PUBLIC lidar_integration_test(
  const std::function<void()> & start_test_nodes,
  const std::function<void()> & stop_test_nodes,
  const std::vector<std::shared_ptr<Vlp16IntegrationSpoofer>> & udp_spoofers,
  const std::vector<std::shared_ptr<LidarIntegrationListener>> & listeners,
  const std::chrono::milliseconds runtime = std::chrono::seconds{10},
  const std::vector<rclcpp::Node::SharedPtr> & nodes = {});

}  // namespace lidar_integration
#endif  // LIDAR_INTEGRATION__LIDAR_INTEGRATION_HPP_
