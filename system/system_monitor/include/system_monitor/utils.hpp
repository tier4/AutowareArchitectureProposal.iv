// Copyright 2020 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file utils.hpp
 * @brief Utility functions used by different system monitor nodes.
 */

/// This function will spin the given node and update the diagnostic state at each iteration
/// \tparam MonitorT Monitor type
/// \param monitor_ptr Shared pointer of a monitor node to be spinned.
/// \param period Spin period.

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

template<typename MonitorT>
void spin_and_update(const std::shared_ptr<MonitorT> & monitor_ptr, std::chrono::seconds period)
{
  while (rclcpp::ok()) {
    monitor_ptr->update();
    rclcpp::spin_some(monitor_ptr);
    std::this_thread::sleep_for(period);
  }
}
