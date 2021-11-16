// Copyright 2021 Autoware Foundation
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

#include <dynamic_object_converter/dynamic_object_converter.hpp>

namespace dynamic_object_converter
{
DynamicObjectConverter::DynamicObjectConverter(const rclcpp::NodeOptions & node_options)
: Node("dynamic_object_converter", node_options)
{
  using std::placeholders::_1;
  pub_ = this->create_publisher<DetectedObjects>("~/output", rclcpp::QoS(1));
  sub_ = this->create_subscription<DetectedObjectsWithFeature>(
    "~/input", 1, std::bind(&DynamicObjectConverter::objectCallback, this, _1));
}

void DynamicObjectConverter::objectCallback(const DetectedObjectsWithFeature::ConstSharedPtr input)
{
  pub_->publish(convert(*input));
}

DetectedObjects DynamicObjectConverter::convert(
  const DetectedObjectsWithFeature & objs_with_feature)
{
  DetectedObjects obj;
  obj.header = objs_with_feature.header;
  for (const auto & obj_with_feature : objs_with_feature.feature_objects) {
    obj.objects.emplace_back(obj_with_feature.object);
  }
  return obj;
}

}  // namespace dynamic_object_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_object_converter::DynamicObjectConverter)
