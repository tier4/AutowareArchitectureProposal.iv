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

#ifndef TOOLS__TURN_SIGNAL_HPP_
#define TOOLS__TURN_SIGNAL_HPP_

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
#include "rviz_common/validate_floats.hpp"

#include "OgreBillboardSet.h"
#include "OgreManualObject.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"

#include "autoware_vehicle_msgs/msg/turn_signal.hpp"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class TurnSignalDisplay
  : public rviz_common::MessageFilterDisplay<autoware_vehicle_msgs::msg::TurnSignal>
{
  Q_OBJECT

public:
  TurnSignalDisplay();
  ~TurnSignalDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(
    const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  // QImage hud_;

private:
  QPointF right_arrow_polygon_[7];
  QPointF left_arrow_polygon_[7];
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__TURN_SIGNAL_HPP_
