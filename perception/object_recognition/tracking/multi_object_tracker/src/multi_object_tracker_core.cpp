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
//
//

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/optional.hpp"
#include "tf2_ros/create_timer_interface.h"
#include "tf2_ros/create_timer_ros.h"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using SemanticType = autoware_perception_msgs::msg::Semantic;

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

bool transformDynamicObjects(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (size_t i = 0; i < output_msg.feature_objects.size(); ++i) {
      tf2::fromMsg(
        output_msg.feature_objects.at(i).object.state.pose_covariance.pose,
        tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(
        tf_target2objects, output_msg.feature_objects.at(i).object.state.pose_covariance.pose);
    }
  }
  return true;
}

inline float getVelocity(const autoware_perception_msgs::msg::DynamicObject & object)
{
  return std::hypot(
    object.state.twist_covariance.twist.linear.x, object.state.twist_covariance.twist.linear.y);
}

inline geometry_msgs::msg::Pose getPose(const autoware_perception_msgs::msg::DynamicObject & object)
{
  return object.state.pose_covariance.pose;
}

float getXYSquareDistance(
  const geometry_msgs::msg::Transform & self_transform,
  const autoware_perception_msgs::msg::DynamicObject & object)
{
  const auto object_pos = getPose(object).position;
  const float x = self_transform.translation.x - object_pos.x;
  const float y = self_transform.translation.y - object_pos.y;
  return x * x + y * y;
}

/**
 * @brief If the tracker is stable at a low speed and has a vehicle type, it will keep
 * tracking for a longer time to deal with detection lost due to occlusion, etc.
 * @param tracker The tracker to be determined.
 * @param time Target time to determine.
 * @param self_transform Position of the vehicle at the target time.
 * @return Result of deciding whether to leave tracker or not.
 */
bool isSpecificAlivePattern(
  const std::shared_ptr<const Tracker> & tracker, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  autoware_perception_msgs::msg::DynamicObject object;
  tracker->getEstimatedDynamicObject(time, object);

  constexpr float min_detection_rate = 0.2;
  constexpr int min_measurement_count = 5;
  constexpr float max_elapsed_time = 10.0;
  constexpr float max_velocity = 1.0;
  constexpr float max_distance = 100.0;

  const float detection_rate =
    tracker->getTotalMeasurementCount() /
    (tracker->getTotalNoMeasurementCount() + tracker->getTotalMeasurementCount());

  const bool big_vehicle =
    tracker->getType() == SemanticType::TRUCK || tracker->getType() == SemanticType::BUS;

  const bool slow_velocity = getVelocity(object) < max_velocity;

  const bool high_confidence =
    (min_detection_rate < detection_rate ||
     min_measurement_count < tracker->getTotalMeasurementCount());

  const bool not_too_far =
    getXYSquareDistance(self_transform, object) < max_distance * max_distance;

  const bool within_max_survival_period =
    tracker->getElapsedTimeFromLastUpdate(time) < max_elapsed_time;

  const bool is_specific_alive_pattern =
    high_confidence && big_vehicle && within_max_survival_period && not_too_far && slow_velocity;

  return is_specific_alive_pattern;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create publishers and subscribers
  dynamic_object_sub_ =
    create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "input", rclcpp::QoS{1},
      std::bind(&MultiObjectTracker::onMeasurement, this, std::placeholders::_1));
  dynamic_object_pub_ =
    create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>("output", rclcpp::QoS{1});

  // Parameters
  double publish_rate = declare_parameter<double>("publish_rate", 30.0);
  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world");
  bool enable_delay_compensation{declare_parameter("enable_delay_compensation", false)};

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer
  if (enable_delay_compensation) {
    auto timer_callback = std::bind(&MultiObjectTracker::onTimer, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));

    publish_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(publish_timer_, nullptr);
  }

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());

  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");

  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix);
}

void MultiObjectTracker::onMeasurement(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr
    input_objects_msg)
{
  const auto self_transform =
    getTransform(tf_buffer_, "base_link", world_frame_id_, input_objects_msg->header.stamp);
  if (!self_transform) {
    return;
  }

  /* transform to world coordinate */
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray transformed_objects;
  if (!transformDynamicObjects(
        *input_objects_msg, world_frame_id_, tf_buffer_, transformed_objects)) {
    return;
  }
  /* tracker prediction */
  rclcpp::Time measurement_time = input_objects_msg->header.stamp;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    transformed_objects, list_tracker_);  // row : tracker, col : measurement
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      (*(tracker_itr))
        ->updateWithMeasurement(
          transformed_objects.feature_objects.at(direct_assignment.find(tracker_idx)->second)
            .object,
          measurement_time);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, measurement_time, *self_transform);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, measurement_time);

  /* new tracker */
  for (size_t i = 0; i < transformed_objects.feature_objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    list_tracker_.push_back(
      createNewTracker(transformed_objects.feature_objects.at(i).object, measurement_time));
  }

  if (publish_timer_ == nullptr) {
    publish(measurement_time);
  }
}

std::shared_ptr<Tracker> MultiObjectTracker::createNewTracker(
  const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time) const
{
  const int & type = object.semantic.type;
  if (type == SemanticType::CAR || type == SemanticType::TRUCK || type == SemanticType::BUS) {
    return std::make_shared<MultipleVehicleTracker>(time, object);
  } else if (type == SemanticType::PEDESTRIAN) {
    return std::make_shared<PedestrianAndBicycleTracker>(time, object);
  } else if (type == SemanticType::BICYCLE || type == SemanticType::MOTORBIKE) {
    return std::make_shared<PedestrianAndBicycleTracker>(time, object);
  } else {
    return std::make_shared<UnknownTracker>(time, object);
  }
}

void MultiObjectTracker::onTimer()
{
  rclcpp::Time current_time = this->now();
  const auto self_transform = getTransform(tf_buffer_, world_frame_id_, "base_link", current_time);
  if (!self_transform) {
    return;
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, current_time, *self_transform);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, current_time);

  // Publish
  publish(current_time);
}

void MultiObjectTracker::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  /* params */
  constexpr float max_elapsed_time = 1.0;

  /* delete tracker */
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    const bool is_old = max_elapsed_time < (*itr)->getElapsedTimeFromLastUpdate(time);
    const bool is_specific_alive_pattern = isSpecificAlivePattern(*itr, time, self_transform);
    if (is_old && !is_specific_alive_pattern) {
      auto erase_itr = itr;
      --itr;
      list_tracker.erase(erase_itr);
    }
  }
}

void MultiObjectTracker::sanitizeTracker(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
{
  constexpr float min_iou = 0.1;
  constexpr double distance_threshold = 5.0;
  /* delete collision tracker */
  for (auto itr1 = list_tracker.begin(); itr1 != list_tracker.end(); ++itr1) {
    autoware_perception_msgs::msg::DynamicObject object1;
    (*itr1)->getEstimatedDynamicObject(time, object1);
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      autoware_perception_msgs::msg::DynamicObject object2;
      (*itr2)->getEstimatedDynamicObject(time, object2);
      const double distance = std::hypot(
        object1.state.pose_covariance.pose.position.x -
          object2.state.pose_covariance.pose.position.x,
        object1.state.pose_covariance.pose.position.y -
          object2.state.pose_covariance.pose.position.y);
      if (distance_threshold < distance) {
        continue;
      }
      if (min_iou < utils::get2dIoU(object1, object2)) {
        if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
          itr1 = list_tracker.erase(itr1);
          --itr1;
          break;
        } else {
          itr2 = list_tracker.erase(itr2);
          --itr2;
        }
      }
    }
  }
}

inline bool MultiObjectTracker::shouldTrackerPublish(
  const std::shared_ptr<const Tracker> tracker) const
{
  constexpr int measurement_count_threshold = 3;
  if (tracker->getTotalMeasurementCount() < measurement_count_threshold) {
    return false;
  }
  return true;
}

void MultiObjectTracker::publish(const rclcpp::Time & time) const
{
  const auto subscriber_count = dynamic_object_pub_->get_subscription_count() +
                                dynamic_object_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }
  // Create output msg
  autoware_perception_msgs::msg::DynamicObjectArray output_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = time;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if (!shouldTrackerPublish(*itr)) {
      continue;
    }
    autoware_perception_msgs::msg::DynamicObject object;
    (*itr)->getEstimatedDynamicObject(time, object);
    output_msg.objects.push_back(object);
  }

  // Publish
  dynamic_object_pub_->publish(output_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
