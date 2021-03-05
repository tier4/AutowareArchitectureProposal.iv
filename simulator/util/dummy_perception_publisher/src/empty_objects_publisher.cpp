// Copyright 2020 Tier IV, Inc.
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

#include <memory>
#include <utility>

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "rclcpp/rclcpp.hpp"

class EmptyObjectsPublisher : public rclcpp::Node
{
public:
  EmptyObjectsPublisher()
  : Node("empty_objects_publisher")
  {
    empty_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
      "output/objects", 1);

    auto timer_callback = std::bind(&EmptyObjectsPublisher::timerCallback, this);
    const auto period = std::chrono::milliseconds(100);
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }

private:
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    empty_objects_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    autoware_perception_msgs::msg::DynamicObjectArray empty_objects;
    empty_objects.header.frame_id = "map";
    empty_objects.header.stamp = this->now();
    empty_objects_pub_->publish(empty_objects);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmptyObjectsPublisher>());
  rclcpp::shutdown();
  return 0;
}
