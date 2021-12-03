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

#include <lidar_integration/lidar_integration.hpp>

#include <thread>
#include <memory>
#include <vector>

namespace lidar_integration
{

bool8_t lidar_integration_test(
  const std::function<void()> & start_test_nodes,
  const std::function<void()> & stop_test_nodes,
  const std::vector<std::shared_ptr<Vlp16IntegrationSpoofer>> & udp_spoofers,
  const std::vector<std::shared_ptr<LidarIntegrationListener>> & listeners,
  const std::chrono::milliseconds runtime,
  const std::vector<rclcpp::Node::SharedPtr> & nodes)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  for (auto listen_ptr : listeners) {
    exec.add_node(listen_ptr);
  }
  for (auto nd_ptr : nodes) {
    exec.add_node(nd_ptr);
  }
  auto listen_promise = std::promise<void>{};
  std::shared_future<void> listen_future = listen_promise.get_future();
  const auto listen_fn = [&exec, &listen_future, runtime]() {
      const auto padded_runtime = runtime + std::chrono::seconds(1);
      exec.spin_until_future_complete(
        listen_future,
        std::chrono::duration_cast<std::chrono::duration<int64_t>>(padded_runtime));
    };

  // Create listener thread
  std::thread listen_thread{listen_fn};

  // Run node and spoofer
  start_test_nodes();
  for (auto spoofer : udp_spoofers) {
    spoofer->start();
  }

  // Block main thread until running is done
  std::this_thread::sleep_for(runtime);

  // Shut everything down
  stop_test_nodes();
  for (auto spoofer : udp_spoofers) {
    spoofer->stop();
  }
  listen_promise.set_value();
  listen_thread.join();

  bool8_t ret = true;
  for (auto listen_ptr : listeners) {
    ret = listen_ptr->is_success() && ret;
  }
  return ret;
}

}  // namespace lidar_integration
