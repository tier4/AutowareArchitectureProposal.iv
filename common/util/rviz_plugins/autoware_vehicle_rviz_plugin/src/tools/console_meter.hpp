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
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
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
#include <iomanip>
#include <memory>

#include "autoware_utils/autoware_utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class ConsoleMeterDisplay
: public rviz_common::MessageFilterDisplay<geometry_msgs::msg::TwistStamped>
{
  Q_OBJECT

public:
  ConsoleMeterDisplay();
  virtual ~ConsoleMeterDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  rviz_common::properties::FloatProperty * property_value_scale_;
  // QImage hud_;

private:
  static constexpr float meter_min_velocity_ = autoware_utils::kmph2mps(0.f);
  static constexpr float meter_max_velocity_ = autoware_utils::kmph2mps(60.f);
  static constexpr float meter_min_angle_ = autoware_utils::deg2rad(40.f);
  static constexpr float meter_max_angle_ = autoware_utils::deg2rad(320.f);
  static constexpr int line_width_ = 2;
  static constexpr int hand_width_ = 4;
  struct Line  // for drawLine
  {
    int x0, y0;
    int x1, y1;
  };
  Line min_range_line_;
  Line max_range_line_;
  struct Arc  // for drawArc
  {
    int x0, y0;
    int x1, y1;
    float start_angle, end_angle;
  };
  Arc inner_arc_;
  Arc outer_arc_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
