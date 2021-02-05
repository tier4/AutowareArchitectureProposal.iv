/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER_PARAMETERS_H
#define LANE_CHANGE_PLANNER_PARAMETERS_H

struct LaneChangerParameters
{
  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double backward_path_length;
  double forward_path_length;
  double lane_change_prepare_duration;
  double lane_changing_duration;
  double backward_length_buffer_for_end_of_lane;
  double lane_change_finish_judge_buffer;
  double minimum_lane_change_length;
  double minimum_lane_change_velocity;
  double prediction_duration;
  double prediction_time_resolution;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double vehicle_width;
  double vehicle_length;
  double base_link2front;
  double static_obstacle_velocity_thresh;
  double maximum_deceleration;
  int lane_change_sampling_num;
  double abort_lane_change_velocity_thresh;
  double abort_lane_change_angle_thresh;
  double abort_lane_change_distance_thresh;
  bool enable_abort_lane_change;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  double refine_goal_search_radius_range;
  bool enable_blocked_by_obstacle;
};

#endif
