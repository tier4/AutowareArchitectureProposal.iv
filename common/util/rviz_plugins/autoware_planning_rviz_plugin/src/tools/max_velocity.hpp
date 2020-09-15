/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <deque>
#include <memory>

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/validate_floats.hpp"

#include "OgreBillboardSet.h"
#include "OgreManualObject.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"

#include "std_msgs/msg/float32.hpp"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class MaxVelocityDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  MaxVelocityDisplay();
  virtual ~MaxVelocityDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();
private Q_SLOTS:
  void updateTopic();
  void updateVisualization();

protected:
  void processMessage(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::StringProperty * property_topic_name_;
  rviz_common::properties::FloatProperty * property_value_scale_;

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_vel_sub_;
  std_msgs::msg::Float32::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
