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

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>

#include "autoware_utils/autoware_utils.h"
#include "geometry_msgs/TwistStamped.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class ConsoleMeterDisplay : public rviz::MessageFilterDisplay<geometry_msgs::TwistStamped>
{
  Q_OBJECT

public:
  ConsoleMeterDisplay();
  ~ConsoleMeterDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const geometry_msgs::TwistStampedConstPtr & msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty * property_text_color_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::IntProperty * property_length_;
  rviz::IntProperty * property_value_height_offset_;
  rviz::FloatProperty * property_value_scale_;
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
  geometry_msgs::TwistStampedConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
