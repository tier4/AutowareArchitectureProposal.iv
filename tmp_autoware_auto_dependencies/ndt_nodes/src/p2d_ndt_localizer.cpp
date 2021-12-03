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

#include <ndt_nodes/ndt_localizer_nodes.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{
struct P2DNDTLocalizerNodeComponent
  : public autoware::localization::ndt_nodes::P2DNDTLocalizerNode<>
{
  explicit P2DNDTLocalizerNodeComponent(const rclcpp::NodeOptions & node_options)
  : autoware::localization::ndt_nodes::P2DNDTLocalizerNode<>(
      "p2d_ndt_localizer_node", node_options,
      autoware::localization::ndt_nodes::PoseInitializer_{})
  {
  }
};
}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization::ndt_nodes::P2DNDTLocalizerNodeComponent)
