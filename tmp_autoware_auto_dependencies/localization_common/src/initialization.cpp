// Copyright 2020 the Autoware Foundation
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

#include <localization_common/initialization.hpp>
#include <time_utils/time_utils.hpp>
#include <string>

namespace autoware
{
namespace localization
{
namespace localization_common
{
geometry_msgs::msg::TransformStamped BestEffortInitializer::extrapolate(
  const tf2::BufferCore & tf_graph, tf2::TimePoint time_point,
  const std::string & target_frame, const std::string & source_frame)
{
  (void) time_point;
  const auto ret = tf_graph.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  if (time_utils::from_message(ret.header.stamp) > time_point) {
    // It's older than the oldest because if it was within [oldest_available, newest_available]
    // we wouldn't be in this function.
    throw std::domain_error(
            "BestEffortInitializer: Backwards extrapolation is not supported."
            "Initialization timepoint is older than the oldest available "
            "transform in the transform graph.");
  }
  return ret;
}

}  // namespace localization_common
}  // namespace localization
}  // namespace autoware
