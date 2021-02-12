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

#include "console_meter.hpp"
#include <rviz/uniform_string_stream.h>
#include <QPainter>

namespace rviz_plugins
{
ConsoleMeterDisplay::ConsoleMeterDisplay()
{
  property_text_color_ = new rviz::ColorProperty(
    "Text Color", QColor(25, 255, 240), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz::IntProperty(
    "Length", 256, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_height_offset_ = new rviz::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_value_scale_ = new rviz::FloatProperty(
    "Value Scale", 1.0 / 6.667, "Value scale", this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
}

ConsoleMeterDisplay::~ConsoleMeterDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void ConsoleMeterDisplay::onInitialize()
{
  MFDClass::onInitialize();
  static int count = 0;
  rviz::UniformStringStream ss;
  ss << "ConsoleMeterDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  // QColor background_color;
  // background_color.setAlpha(0);
  // jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  // hud_ = buffer.getQImage(*overlay_);
  // for (int i = 0; i < overlay_->getTextureWidth(); i++)
  // {
  //   for (int j = 0; j < overlay_->getTextureHeight(); j++)
  //   {
  //     hud_.setPixel(i, j, background_color.rgba());
  //   }
  // }
}

void ConsoleMeterDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void ConsoleMeterDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void ConsoleMeterDisplay::processMessage(const geometry_msgs::TwistStampedConstPtr & msg_ptr)
{
  if (!isEnabled()) {
    return;
  }
  if (!overlay_->isVisible()) {
    return;
  }

  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const int w = overlay_->getTextureWidth() - line_width_;
  const int h = overlay_->getTextureHeight() - line_width_;

  // meter
  QColor white_color(Qt::white);
  white_color.setAlpha(255);
  const float velocity_ratio = std::min(
    std::max(std::fabs(msg_ptr->twist.linear.x) - meter_min_velocity_, 0.0) /
      (meter_max_velocity_ - meter_min_velocity_),
    1.0);
  const float theta =
    (velocity_ratio * (meter_max_angle_ - meter_min_angle_)) + meter_min_angle_ + M_PI_2;

  painter.setPen(QPen(white_color, hand_width_, Qt::SolidLine));
  painter.drawLine(
    w * 0.5, h * 0.5, (w * 0.5) + ((float)w * 0.5 - ((float)hand_width_ * 0.5)) * std::cos(theta),
    (h * 0.5) + ((float)h * 0.5 - ((float)hand_width_ * 0.5)) * std::sin(theta));

  painter.setPen(QPen(white_color, line_width_, Qt::SolidLine));
  painter.drawLine(min_range_line_.x0, min_range_line_.y0, min_range_line_.x1, min_range_line_.y1);
  painter.drawLine(max_range_line_.x0, max_range_line_.y0, max_range_line_.x1, max_range_line_.y1);
  painter.drawArc(
    outer_arc_.x0, outer_arc_.y0, outer_arc_.x1, outer_arc_.y1, outer_arc_.start_angle * 16,
    outer_arc_.end_angle * 16);
  painter.drawArc(
    inner_arc_.x0, inner_arc_.y0, inner_arc_.x1, inner_arc_.y1, inner_arc_.start_angle * 16,
    inner_arc_.end_angle * 16);

  // text
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, int(2), Qt::SolidLine));
  QFont font = painter.font();
  font.setPixelSize(std::max(int(double(w) * property_value_scale_->getFloat()), 1));
  font.setBold(true);
  painter.setFont(font);
  std::ostringstream velocity_ss;
  velocity_ss << std::fixed << std::setprecision(2) << msg_ptr->twist.linear.x * 3.6 << "km/h";
  painter.drawText(
    0, std::min(property_value_height_offset_->getInt(), h - 1), w,
    std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignCenter | Qt::AlignVCenter,
    velocity_ss.str().c_str());
  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void ConsoleMeterDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  const float min_range_theta = meter_max_angle_ + M_PI_2;
  const float max_range_theta = meter_min_angle_ + M_PI_2;
  const int w = overlay_->getTextureWidth() - line_width_;
  const int h = overlay_->getTextureHeight() - line_width_;

  min_range_line_.x0 = (w / 2.0) + line_width_ / 2.0 + ((float)w / 8.0) * std::cos(min_range_theta);
  min_range_line_.y0 = (h / 2.0) + line_width_ / 2.0 + ((float)h / 8.0) * std::sin(min_range_theta);
  min_range_line_.x1 = (w / 2.0) + line_width_ / 2.0 +
                       ((float)w / 2.0 - (line_width_ / 2.0)) * std::cos(min_range_theta);
  min_range_line_.y1 = (h / 2.0) + line_width_ / 2.0 +
                       ((float)h / 2.0 - (line_width_ / 2.0)) * std::sin(min_range_theta);
  max_range_line_.x0 = (w / 2.0) + line_width_ / 2.0 + ((float)w / 8.0) * std::cos(max_range_theta);
  max_range_line_.y0 = (h / 2.0) + line_width_ / 2.0 + ((float)h / 8.0) * std::sin(max_range_theta);
  max_range_line_.x1 = (w * 0.5) + line_width_ * 0.5 +
                       ((float)w / 2.0 - (line_width_ * 0.5)) * std::cos(max_range_theta);
  max_range_line_.y1 = (h * 0.5) + line_width_ * 0.5 +
                       ((float)h / 2.0 - (line_width_ * 0.5)) * std::sin(max_range_theta);
  inner_arc_.x0 = line_width_ / 2.0;
  inner_arc_.y0 = line_width_ / 2.0;
  inner_arc_.x1 = w;
  inner_arc_.y1 = h;
  inner_arc_.start_angle = autoware_utils::rad2deg(min_range_theta - M_PI);
  inner_arc_.end_angle = autoware_utils::rad2deg(max_range_theta - min_range_theta);
  outer_arc_.x0 = w * 3 / 8;
  outer_arc_.y0 = h * 3 / 8;
  outer_arc_.x1 = w / 4;
  outer_arc_.y1 = h / 4;
  outer_arc_.start_angle = autoware_utils::rad2deg(min_range_theta - M_PI);
  outer_arc_.end_angle = autoware_utils::rad2deg(max_range_theta - min_range_theta);

  if (last_msg_ptr_ != nullptr) processMessage(last_msg_ptr_);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ConsoleMeterDisplay, rviz::Display)
