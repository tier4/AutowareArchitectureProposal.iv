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

#pragma once

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "OgreBillboardSet.h"
#include "OgreManualObject.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"

#include <deque>
#include <memory>
#include <iomanip>

#include "autoware_vehicle_msgs/msg/steering.hpp"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class SteeringAngleDisplay : public rviz_common::MessageFilterDisplay<autoware_vehicle_msgs::msg::Steering>
{
  Q_OBJECT

public:
  SteeringAngleDisplay();
  virtual ~SteeringAngleDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
    const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::FloatProperty * property_handle_angle_scale_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  QPixmap handle_image_;
  // QImage hud_;

private:
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
