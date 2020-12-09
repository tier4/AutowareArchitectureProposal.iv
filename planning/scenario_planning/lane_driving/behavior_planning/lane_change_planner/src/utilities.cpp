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

#include <lane_change_planner/utilities.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

namespace
{
ros::Duration safeSubtraction(const ros::Time & t1, const ros::Time & t2)
{
  ros::Duration duration;
  try {
    duration = t1 - t2;
  } catch (std::runtime_error) {
    if (t1 > t2)
      duration = ros::DURATION_MIN;
    else
      duration = ros::DURATION_MAX;
  }
  return duration;
}
ros::Time safeAddition(const ros::Time & t1, const double seconds)
{
  ros::Time sum;
  try {
    sum = t1 + ros::Duration(seconds);
  } catch (std::runtime_error & err) {
    if (seconds > 0) sum = ros::TIME_MAX;
    if (seconds < 0) sum = ros::TIME_MIN;
  }
  return sum;
}

cv::Point toCVPoint(
  const geometry_msgs::Point & geom_point, const double width_m, const double height_m,
  const double resolution)
{
  return cv::Point(
    static_cast<int>((height_m - geom_point.y) / resolution),
    static_cast<int>((width_m - geom_point.x) / resolution));
}

void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::OccupancyGrid * occupancy_grid)
{
  occupancy_grid->data.reserve(cv_image.rows * cv_image.cols);
  for (int x = cv_image.cols - 1; x >= 0; x--) {
    for (int y = cv_image.rows - 1; y >= 0; y--) {
      const unsigned char intensity = cv_image.at<unsigned char>(y, x);
      occupancy_grid->data.push_back(intensity);
    }
  }
}

}  // namespace

namespace lane_change_planner
{
namespace util
{
using autoware_perception_msgs::PredictedPath;
using autoware_planning_msgs::PathWithLaneId;

double normalizeRadian(const double radian)
{
  double normalized = radian;
  while (normalized > M_PI) {
    normalized -= (2 * M_PI);
  }
  while (normalized < -M_PI) {
    normalized += 2 * M_PI;
  }
  return normalized;
}

double l2Norm(const geometry_msgs::Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt)
{
  return Eigen::Vector3d(geom_pt.x, geom_pt.y, geom_pt.z);
}

// returns false when search point is off the linestring
bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point search_point_geom, FrenetCoordinate3d * frenet_coordinate)
{
  if (linestring.empty()) {
    return false;
  }

  const auto search_pt = convertToEigenPt(search_point_geom);
  bool found = false;
  double min_distance = std::numeric_limits<double>::max();

  // get frenet coordinate based on points
  // this is done because linestring is not differentiable at vertices
  {
    double accumulated_length = 0;

    for (std::size_t i = 0; i < linestring.size(); i++) {
      const auto geom_pt = linestring.at(i);
      const auto current_pt = convertToEigenPt(geom_pt);
      const auto current2search_pt = (search_pt - current_pt);
      // update accumulated length
      if (i != 0) {
        const auto p1 = convertToEigenPt(linestring.at(i - 1));
        const auto p2 = current_pt;
        accumulated_length += (p2 - p1).norm();
      }
      // update frenet coordinate

      const double tmp_distance = current2search_pt.norm();
      if (tmp_distance < min_distance) {
        found = true;
        min_distance = tmp_distance;
        frenet_coordinate->distance = tmp_distance;
        frenet_coordinate->length = accumulated_length;
      }
    }
  }

  // get frenet coordinate based on lines
  {
    auto prev_geom_pt = linestring.front();
    double accumulated_length = 0;
    for (const auto & geom_pt : linestring) {
      const auto start_pt = convertToEigenPt(prev_geom_pt);
      const auto end_pt = convertToEigenPt(geom_pt);

      const auto line_segment = end_pt - start_pt;
      const double line_segment_length = line_segment.norm();
      const auto direction = line_segment / line_segment_length;
      const auto start2search_pt = (search_pt - start_pt);

      double tmp_length = direction.dot(start2search_pt);
      if (tmp_length >= 0 && tmp_length <= line_segment_length) {
        double tmp_distance = direction.cross(start2search_pt).norm();
        if (tmp_distance < min_distance) {
          found = true;
          min_distance = tmp_distance;
          frenet_coordinate->distance = tmp_distance;
          frenet_coordinate->length = accumulated_length + tmp_length;
        }
      }
      accumulated_length += line_segment_length;
      prev_geom_pt = geom_pt;
    }
  }
  return found;
}

std::vector<geometry_msgs::Point> convertToGeometryPointArray(const PathWithLaneId & path)
{
  std::vector<geometry_msgs::Point> converted_path;
  converted_path.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_path.push_back(point_with_id.point.pose.position);
  }
  return converted_path;
}

std::vector<geometry_msgs::Point> convertToGeometryPointArray(const PredictedPath & path)
{
  std::vector<geometry_msgs::Point> converted_path;

  converted_path.reserve(path.path.size());
  for (const auto & pose_with_cov_stamped : path.path) {
    converted_path.push_back(pose_with_cov_stamped.pose.pose.position);
  }
  return converted_path;
}

geometry_msgs::PoseArray convertToGeometryPoseArray(const PathWithLaneId & path)
{
  geometry_msgs::PoseArray converted_array;
  converted_array.header = path.header;

  converted_array.poses.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_array.poses.push_back(point_with_id.point.pose);
  }
  return converted_array;
}

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const geometry_msgs::Twist & vehicle_twist,
  const geometry_msgs::Pose & vehicle_pose, const double duration, const double resolution,
  const double acceleration)
{
  PredictedPath predicted_path;
  predicted_path.path.reserve(path.points.size());
  if (path.points.empty()) {
    return predicted_path;
  }

  const auto & geometry_points = convertToGeometryPointArray(path);
  FrenetCoordinate3d vehicle_pose_frenet;
  convertToFrenetCoordinate3d(geometry_points, vehicle_pose.position, &vehicle_pose_frenet);
  ros::Time start_time = ros::Time::now();
  double vehicle_speed = std::abs(vehicle_twist.linear.x);
  constexpr double min_speed = 1.0;
  if (vehicle_speed < min_speed) {
    vehicle_speed = min_speed;
    ROS_DEBUG_STREAM_THROTTLE(
      1, "cannot convert PathWithLaneId with zero velocity, using minimum value "
           << min_speed << " [m/s] instead");
  }

  double length = 0;
  double prev_vehicle_speed = vehicle_speed;

  // first point
  const auto pt = lerpByLength(geometry_points, vehicle_pose_frenet.length);
  geometry_msgs::PoseWithCovarianceStamped predicted_pose;
  predicted_pose.header.stamp = start_time;
  predicted_pose.pose.pose.position = pt;
  predicted_path.path.push_back(predicted_pose);

  for (double t = resolution; t < duration; t += resolution) {
    double accelerated_velocity = prev_vehicle_speed + acceleration * t;
    double travel_distance = 0;
    if (accelerated_velocity < min_speed) {
      travel_distance = min_speed * resolution;
    } else {
      travel_distance =
        prev_vehicle_speed * resolution + 0.5 * acceleration * resolution * resolution;
    }

    length += travel_distance;
    const auto pt = lerpByLength(geometry_points, vehicle_pose_frenet.length + length);
    geometry_msgs::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.stamp = safeAddition(start_time, t);
    predicted_pose.pose.pose.position = pt;
    predicted_path.path.push_back(predicted_pose);
    prev_vehicle_speed = accelerated_velocity;
  }
  return predicted_path;
}

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution, const double duration)
{
  PredictedPath resampled_path;

  ros::Duration t_delta(resolution);
  ros::Duration prediction_duration(duration);

  double min_distance = std::numeric_limits<double>::max();
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = ros::Time::now() + prediction_duration;

  for (auto t = start_time; t < end_time; t += t_delta) {
    geometry_msgs::Pose pose;
    if (!lerpByTimeStamp(input_path, t, &pose)) {
      continue;
    }
    geometry_msgs::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.frame_id = "map";
    predicted_pose.header.stamp = t;
    predicted_pose.pose.pose = pose;
    resampled_path.path.push_back(predicted_pose);
  }

  return resampled_path;
}

geometry_msgs::Pose lerpByPose(
  const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::Pose pose;
  pose.position = tf2::toMsg(tf_point, pose.position);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

geometry_msgs::Point lerpByPoint(
  const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, const double t)
{
  tf2::Vector3 v1, v2;
  v1.setValue(p1.x, p1.y, p1.z);
  v2.setValue(p2.x, p2.y, p2.z);

  const auto lerped_point = v1.lerp(v2, t);

  geometry_msgs::Point point;
  point.x = lerped_point.x();
  point.y = lerped_point.y();
  point.z = lerped_point.z();
  return point;
}

geometry_msgs::Point lerpByLength(
  const std::vector<geometry_msgs::Point> & point_array, const double length)
{
  geometry_msgs::Point lerped_point;
  if (point_array.empty()) {
    return lerped_point;
  }
  geometry_msgs::Point prev_pt = point_array.front();
  double accumulated_length = 0;
  for (const auto & pt : point_array) {
    const double distance = getDistance3d(prev_pt, pt);
    if (accumulated_length + distance > length) {
      return lerpByPoint(prev_pt, pt, (length - accumulated_length) / distance);
    }
    accumulated_length += distance;
    prev_pt = pt;
  }

  return point_array.back();
}

bool lerpByTimeStamp(
  const PredictedPath & path, const ros::Time & t, geometry_msgs::Pose * lerped_pt)
{
  if (lerped_pt == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "failed to lerp by time due to nullptr pt");
    return false;
  }
  if (path.path.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "Empty path. Failed to interpolate path by time!");
    return false;
  }
  if (t < path.path.front().header.stamp) {
    ROS_DEBUG_STREAM(
      "failed to interpolate path by time!"
      << std::endl
      << "path start time: " << path.path.front().header.stamp << std::endl
      << "path end time  : " << path.path.back().header.stamp << std::endl
      << "query time     : " << t);
    *lerped_pt = path.path.front().pose.pose;
    return false;
  }

  if (t > path.path.back().header.stamp) {
    ROS_DEBUG_STREAM(
      "failed to interpolate path by time!"
      << std::endl
      << "path start time: " << path.path.front().header.stamp << std::endl
      << "path end time  : " << path.path.back().header.stamp << std::endl
      << "query time     : " << t);
    *lerped_pt = path.path.back().pose.pose;

    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    if (t <= pt.header.stamp) {
      const ros::Duration duration = safeSubtraction(pt.header.stamp, prev_pt.header.stamp);
      const auto offset = t - prev_pt.header.stamp;
      const auto ratio = offset.toSec() / duration.toSec();
      *lerped_pt = lerpByPose(prev_pt.pose.pose, pt.pose.pose, ratio);
      return true;
    }
  }

  ROS_ERROR_STREAM("Something failed in function: " << __func__);
  return false;
}

double getDistance3d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double getDistanceBetweenPredictedPaths(
  const PredictedPath & object_path, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution)
{
  ros::Duration t_delta(resolution);
  double min_distance = std::numeric_limits<double>::max();
  ros::Time ros_start_time = ros::Time::now() + ros::Duration(start_time);
  ros::Time ros_end_time = ros::Time::now() + ros::Duration(end_time);
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (auto t = ros_start_time; t < ros_end_time; t += t_delta) {
    geometry_msgs::Pose object_pose, ego_pose;
    if (!lerpByTimeStamp(object_path, t, &object_pose)) {
      continue;
    }
    if (!lerpByTimeStamp(ego_path, t, &ego_pose)) {
      continue;
    }
    double distance = getDistance3d(object_pose.position, ego_pose.position);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double getDistanceBetweenPredictedPathAndObject(
  const autoware_perception_msgs::DynamicObject & object, const PredictedPath & ego_path,
  const double start_time, const double end_time, const double resolution)
{
  ros::Duration t_delta(resolution);
  double min_distance = std::numeric_limits<double>::max();
  ros::Time ros_start_time = ros::Time::now() + ros::Duration(start_time);
  ros::Time ros_end_time = ros::Time::now() + ros::Duration(end_time);
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  Polygon obj_polygon;
  if (!calcObjectPolygon(object, &obj_polygon)) {
    return min_distance;
  }
  for (auto t = ros_start_time; t < ros_end_time; t += t_delta) {
    geometry_msgs::Pose ego_pose;
    if (!lerpByTimeStamp(ego_path, t, &ego_pose)) {
      continue;
    }
    Point ego_point = boost::geometry::make<Point>(ego_pose.position.x, ego_pose.position.y);

    double distance = boost::geometry::distance(obj_polygon, ego_point);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

// only works with consecutive lanes
std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & target_lanelets, const double start_arc_length,
  const double end_arc_length)
{
  std::vector<size_t> indices;
  if (target_lanelets.empty()) {
    return {};
  }
  const auto polygon =
    lanelet::utils::getPolygonFromArcLength(target_lanelets, start_arc_length, end_arc_length);
  const auto polygon2d = lanelet::utils::to2D(polygon).basicPolygon();
  if (polygon2d.empty()) {
    // no lanelet polygon
    return {};
  }

  for (size_t i = 0; i < objects.objects.size(); i++) {
    const auto obj = objects.objects.at(i);
    // create object polygon
    Polygon obj_polygon;
    if (!calcObjectPolygon(obj, &obj_polygon)) {
      continue;
    }
    // create lanelet polygon
    Polygon lanelet_polygon;
    for (const auto & lanelet_point : polygon2d) {
      lanelet_polygon.outer().push_back(Point(lanelet_point.x(), lanelet_point.y()));
    }

    lanelet_polygon.outer().push_back(lanelet_polygon.outer().front());

    // check the object does not intersect the lanelet
    if (!boost::geometry::disjoint(lanelet_polygon, obj_polygon)) {
      indices.push_back(i);
      continue;
    }
  }
  return indices;
}

// works with random lanelets
std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & target_lanelets)
{
  std::vector<size_t> indices;
  if (target_lanelets.empty()) {
    return {};
  }

  for (size_t i = 0; i < objects.objects.size(); i++) {
    // create object polygon
    const auto obj = objects.objects.at(i);
    // create object polygon
    Polygon obj_polygon;
    if (!calcObjectPolygon(obj, &obj_polygon)) {
      continue;
    }

    for (const auto & llt : target_lanelets) {
      // create lanelet polygon
      const auto polygon2d = llt.polygon2d().basicPolygon();
      if (polygon2d.empty()) {
        // no lanelet polygon
        continue;
      }
      Polygon lanelet_polygon;
      for (const auto & lanelet_point : polygon2d) {
        lanelet_polygon.outer().push_back(
          boost::geometry::make<Point>(lanelet_point.x(), lanelet_point.y()));
      }

      lanelet_polygon.outer().push_back(lanelet_polygon.outer().front());

      // check the object does not intersect the lanelet
      if (!boost::geometry::disjoint(lanelet_polygon, obj_polygon)) {
        indices.push_back(i);
        continue;
      }
    }
  }
  return indices;
}

bool calcObjectPolygon(
  const autoware_perception_msgs::DynamicObject & object, Polygon * object_polygon)
{
  const double obj_x = object.state.pose_covariance.pose.position.x;
  const double obj_y = object.state.pose_covariance.pose.position.y;
  if (object.shape.type == autoware_perception_msgs::Shape::BOUNDING_BOX) {
    const double len_x = object.shape.dimensions.x;
    const double len_y = object.shape.dimensions.y;

    tf2::Transform tf_map2obj;
    tf2::fromMsg(object.state.pose_covariance.pose, tf_map2obj);

    // set vertices at map coordinate
    tf2::Vector3 p1_map, p2_map, p3_map, p4_map;

    p1_map.setX(len_x / 2);
    p1_map.setY(len_y / 2);
    p1_map.setZ(0.0);
    p1_map.setW(1.0);

    p2_map.setX(-len_x / 2);
    p2_map.setY(len_y / 2);
    p2_map.setZ(0.0);
    p2_map.setW(1.0);

    p3_map.setX(-len_x / 2);
    p3_map.setY(-len_y / 2);
    p3_map.setZ(0.0);
    p3_map.setW(1.0);

    p4_map.setX(len_x / 2);
    p4_map.setY(-len_y / 2);
    p4_map.setZ(0.0);
    p4_map.setW(1.0);

    // transform vertices from map coordinate to object coordinate
    tf2::Vector3 p1_obj, p2_obj, p3_obj, p4_obj;

    p1_obj = tf_map2obj * p1_map;
    p2_obj = tf_map2obj * p2_map;
    p3_obj = tf_map2obj * p3_map;
    p4_obj = tf_map2obj * p4_map;

    object_polygon->outer().push_back(Point(p1_obj.x(), p1_obj.y()));
    object_polygon->outer().push_back(Point(p2_obj.x(), p2_obj.y()));
    object_polygon->outer().push_back(Point(p3_obj.x(), p3_obj.y()));
    object_polygon->outer().push_back(Point(p4_obj.x(), p4_obj.y()));

  } else if (object.shape.type == autoware_perception_msgs::Shape::CYLINDER) {
    const size_t N = 20;
    const double r = object.shape.dimensions.x / 2;
    for (size_t i = 0; i < N; ++i) {
      object_polygon->outer().push_back(
        Point(obj_x + r * std::cos(2.0 * M_PI / N * i), obj_y + r * std::sin(2.0 * M_PI / N * i)));
    }
  } else if (object.shape.type == autoware_perception_msgs::Shape::POLYGON) {
    const auto obj_points = object.shape.footprint.points;
    for (const auto & obj_point : obj_points) {
      object_polygon->outer().push_back(Point(obj_point.x, obj_point.y));
    }
  } else {
    ROS_WARN("Object shape unknown!");
    return false;
  }
  object_polygon->outer().push_back(object_polygon->outer().front());

  return true;
}

std::vector<size_t> filterObjectsByPath(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const std::vector<size_t> & object_indices,
  const autoware_planning_msgs::PathWithLaneId & ego_path, const double vehicle_width)
{
  std::vector<size_t> indices;
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (const auto & i : object_indices) {
    Polygon obj_polygon;
    if (!calcObjectPolygon(objects.objects.at(i), &obj_polygon)) {
      continue;
    }
    LineString ego_path_line;
    for (size_t j = 0; j < ego_path_point_array.size(); ++j) {
      boost::geometry::append(
        ego_path_line, Point(ego_path_point_array.at(j).x, ego_path_point_array.at(j).y));
    }
    const double distance = boost::geometry::distance(obj_polygon, ego_path_line);
    if (distance < vehicle_width) {
      indices.push_back(i);
    }
  }
  return indices;
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double min_dist = 0.001;
    if (
      getDistance3d(filtered_path.points.back().point.pose.position, pt.point.pose.position) <
      min_dist) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
    } else {
      filtered_path.points.push_back(pt);
    }
  }
  filtered_path.drivable_area = input_path.drivable_area;
  return filtered_path;
}

template <typename T>
bool exists(std::vector<T> vec, T element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

bool setGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::Pose & goal, const int64_t goal_lane_id, PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }
    size_t min_dist_index;
    double min_dist = std::numeric_limits<double>::max();
    double goal_z;
    {
      bool found = false;
      for (size_t i = 0; i < input.points.size(); ++i) {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        const double dist = sqrt(x * x + y * y);
        if (
          dist < search_radius_range && dist < min_dist &&
          exists(input.points.at(i).lane_ids, goal_lane_id)) {
          min_dist_index = i;
          min_dist = dist;
          found = true;
        }
      }
      if (!found) {
        return false;
      }
    }

    size_t min_dist_out_of_range_index;
    {
      for (size_t i = min_dist_index; 0 <= i; --i) {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        goal_z = input.points.at(i).point.pose.position.z;
        const double dist = sqrt(x * x + y * y);
        min_dist_out_of_range_index = i;
        if (search_radius_range < dist) {
          break;
        }
        if (i == 0) {
          break;
        }
      }
    }
    autoware_planning_msgs::PathPointWithLaneId refined_goal;
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z = goal_z;
    refined_goal.point.twist.linear.x = 0.0;
    refined_goal.lane_ids = input.points.back().lane_ids;

    autoware_planning_msgs::PathPointWithLaneId pre_refined_goal;
    double roll, pitch, yaw;
    pre_refined_goal.point.pose = goal;
    tf2::Quaternion tf2_quaternion(
      goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
    pre_refined_goal.point.pose.position.x -= std::cos(yaw);
    pre_refined_goal.point.pose.position.y -= std::sin(yaw);
    pre_refined_goal.point.pose.position.z = goal_z;
    pre_refined_goal.point.twist.linear.x =
      input.points.at(min_dist_out_of_range_index).point.twist.linear.x;
    pre_refined_goal.lane_ids = input.points.back().lane_ids;

    for (size_t i = 0; i <= min_dist_out_of_range_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    output_ptr->drivable_area = input.drivable_area;
    return true;
  } catch (std::out_of_range & ex) {
    ROS_ERROR_STREAM("failed to set goal" << ex.what() << std::endl);
    return false;
  }
}

const geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  // return goal;
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
  if (segment.empty()) {
    return goal;
  }

  geometry_msgs::Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refinePath(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path, path_with_goal;
  filtered_path = removeOverlappingPoints(input);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.twist.linear.x = 0.0;
  }

  if (setGoal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  } else {
    return filtered_path;
  }
}

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id)
{
  for (const auto & lane : lanes) {
    if (lane.id() == goal_id) {
      return true;
    }
  }
  return false;
}

// input lanes must be in sequence
nav_msgs::OccupancyGrid generateDrivableArea(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::PoseStamped & current_pose,
  const double width, const double height, const double resolution, const double vehicle_length,
  const RouteHandler & route_handler)
{
  // get drivable lanes
  lanelet::ConstLanelets drivable_lanes = lanes;
  if (containsGoal(lanes, route_handler.getGoalLaneId())) {
    const auto lanes_after_goal = route_handler.getLanesAfterGoal(vehicle_length);
    drivable_lanes.insert(drivable_lanes.end(), lanes_after_goal.begin(), lanes_after_goal.end());
  }

  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::PoseStamped grid_origin;

  // calculate grid origin
  {
    grid_origin.header = current_pose.header;
    const double yaw = tf2::getYaw(current_pose.pose.orientation);
    const double origin_offset_x_m = (-width / 4) * cos(yaw) - (-height / 2) * sin(yaw);
    const double origin_offset_y_m = (-width / 4) * sin(yaw) + (-height / 2) * cos(yaw);
    grid_origin.pose.orientation = current_pose.pose.orientation;
    grid_origin.pose.position.x = current_pose.pose.position.x + origin_offset_x_m;
    grid_origin.pose.position.y = current_pose.pose.position.y + origin_offset_y_m;
    grid_origin.pose.position.z = current_pose.pose.position.z;
  }

  // header
  {
    occupancy_grid.header.stamp = current_pose.header.stamp;
    occupancy_grid.header.frame_id = "map";
  }

  // info
  {
    const int width_cell = width / resolution;
    const int height_cell = height / resolution;

    occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width_cell;
    occupancy_grid.info.height = height_cell;
    occupancy_grid.info.origin = grid_origin.pose;
  }

  // occupancy_grid.data = image;
  {
    constexpr uint8_t free_space = 0;
    constexpr uint8_t occupied_space = 100;
    // get transform
    tf2::Stamped<tf2::Transform> tf_grid2map, tf_map2grid;
    tf2::fromMsg(grid_origin, tf_grid2map);
    tf_map2grid.setData(tf_grid2map.inverse());
    const auto geom_tf_map2grid = tf2::toMsg(tf_map2grid);

    // convert lane polygons into cv type
    cv::Mat cv_image(
      occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1, cv::Scalar(occupied_space));
    for (std::size_t i = 0; i < drivable_lanes.size(); i++) {
      const auto lane = drivable_lanes.at(i);

      // skip if it overlaps with past lane
      bool overlaps_with_past_lane = false;
      for (std::size_t j = 0; j < i; j++) {
        const auto past_lane = drivable_lanes.at(j);
        if (boost::geometry::overlaps(
              lane.polygon2d().basicPolygon(), past_lane.polygon2d().basicPolygon())) {
          overlaps_with_past_lane = true;
          break;
        }
      }
      if (overlaps_with_past_lane) {
        continue;
      }

      // create drivable area using opencv
      std::vector<std::vector<cv::Point>> cv_polygons;
      std::vector<int> cv_polygon_sizes;
      cv::Mat cv_image_single_lane(
        occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1, cv::Scalar(occupied_space));
      std::vector<cv::Point> cv_polygon;
      for (const auto & llt_pt : lane.polygon3d()) {
        geometry_msgs::Point geom_pt = lanelet::utils::conversion::toGeomMsgPt(llt_pt);
        geometry_msgs::Point transformed_geom_pt;
        tf2::doTransform(geom_pt, transformed_geom_pt, geom_tf_map2grid);
        cv_polygon.push_back(toCVPoint(transformed_geom_pt, width, height, resolution));
      }
      cv_polygons.push_back(cv_polygon);
      cv_polygon_sizes.push_back(cv_polygon.size());
      // fill in drivable area and copy to occupancy grid
      cv::fillPoly(cv_image_single_lane, cv_polygons, cv::Scalar(free_space));
      cv::bitwise_and(cv_image, cv_image_single_lane, cv_image);
    }

    const auto & cv_image_reshaped = cv_image.reshape(1, 1);
    imageToOccupancyGrid(cv_image, &occupancy_grid);
    occupancy_grid.data[0] = 0;
    // cv_image_reshaped.copyTo(occupancy_grid.data);
  }
  return occupancy_grid;
}

double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const double lanelet_length = lanelet::utils::getLaneletLength3d(lanelets);
  return lanelet_length - arc_coordinates.length;
}

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    if (is_after_current_lanelet && llt.hasAttribute("turn_direction")) {
      bool is_lane_change_yes = false;
      const auto right_line = llt.rightBound();
      if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      const auto left_line = llt.leftBound();
      if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      if (!is_lane_change_yes) return distance - arc_coordinates.length;
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getDistanceToCrosswalk(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    // check lane change tag
    bool is_lane_change_yes = false;
    const auto right_line = llt.rightBound();
    if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
      const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
      if (attr.value() == std::string("yes")) {
        is_lane_change_yes = true;
      }
    }
    const auto left_line = llt.leftBound();
    if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
      const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
      if (attr.value() == std::string("yes")) {
        is_lane_change_yes = true;
      }
    }

    if (is_after_current_lanelet && !is_lane_change_yes) {
      const auto conflicting_crosswalks = overall_graphs.conflictingInGraph(llt, 1);
      if (!(conflicting_crosswalks.empty())) {
        // create centerline
        const lanelet::ConstLineString2d lanelet_centerline = llt.centerline2d();
        LineString centerline;
        for (const auto & point : lanelet_centerline) {
          boost::geometry::append(centerline, Point(point.x(), point.y()));
        }

        //create crosswalk polygon and calculate distance
        double min_distance_to_crosswalk = std::numeric_limits<double>::max();
        for (const auto & crosswalk : conflicting_crosswalks) {
          lanelet::CompoundPolygon2d lanelet_crosswalk_polygon = crosswalk.polygon2d();
          Polygon polygon;
          for (const auto & point : lanelet_crosswalk_polygon) {
            polygon.outer().push_back(Point(point.x(), point.y()));
          }
          polygon.outer().push_back(polygon.outer().front());

          std::vector<Point> points_intersection;
          boost::geometry::intersection(centerline, polygon, points_intersection);

          for (const auto & point : points_intersection) {
            lanelet::ConstLanelets lanelets = {llt};
            geometry_msgs::Pose pose_point;
            pose_point.position.x = point.x();
            pose_point.position.y = point.y();
            const lanelet::ArcCoordinates & arc_crosswalk =
              lanelet::utils::getArcCoordinates(lanelets, pose_point);

            const double distance_to_crosswalk = arc_crosswalk.length;
            if (distance_to_crosswalk < min_distance_to_crosswalk) {
              min_distance_to_crosswalk = distance_to_crosswalk;
            }
          }
        }
        if (distance + min_distance_to_crosswalk > arc_coordinates.length) {
          return distance + min_distance_to_crosswalk - arc_coordinates.length;
        }
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getSignedDistance(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets)
{
  const auto arc_current = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const auto arc_goal = lanelet::utils::getArcCoordinates(lanelets, goal_pose);

  return arc_goal.length - arc_current.length;
}

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets)
{
  std::vector<uint64_t> ids;
  for (const auto & llt : lanelets) {
    ids.push_back(llt.id());
  }
  return ids;
}

autoware_planning_msgs::Path convertToPathFromPathWithLaneId(
  const autoware_planning_msgs::PathWithLaneId & path_with_lane_id)
{
  autoware_planning_msgs::Path path;
  path.header = path_with_lane_id.header;
  path.drivable_area = path_with_lane_id.drivable_area;
  for (const auto & pt_with_lane_id : path_with_lane_id.points) {
    path.points.push_back(pt_with_lane_id.point);
  }
  return path;
}

lanelet::Polygon3d getVehiclePolygon(
  const geometry_msgs::Pose & vehicle_pose, const double vehicle_width,
  const double base_link2front)
{
  tf2::Vector3 front_left, front_right, rear_left, rear_right;
  front_left.setValue(base_link2front, vehicle_width / 2, 0);
  front_right.setValue(base_link2front, -vehicle_width / 2, 0);
  rear_left.setValue(0, vehicle_width / 2, 0);
  rear_right.setValue(0, -vehicle_width / 2, 0);

  tf2::Transform tf;
  tf2::fromMsg(vehicle_pose, tf);
  const auto front_left_transformed = tf * front_left;
  const auto front_right_transformed = tf * front_right;
  const auto rear_left_transformed = tf * rear_left;
  const auto rear_right_transformed = tf * rear_right;

  lanelet::Polygon3d llt_poly;
  llt_poly.push_back(lanelet::Point3d(
    0, front_left_transformed.x(), front_left_transformed.y(), front_left_transformed.z()));
  llt_poly.push_back(lanelet::Point3d(
    0, front_right_transformed.x(), front_right_transformed.y(), front_right_transformed.z()));
  llt_poly.push_back(lanelet::Point3d(
    0, rear_right_transformed.x(), rear_right_transformed.y(), rear_right_transformed.z()));
  llt_poly.push_back(lanelet::Point3d(
    0, rear_left_transformed.x(), rear_left_transformed.y(), rear_left_transformed.z()));
  return llt_poly;
}

autoware_planning_msgs::PathPointWithLaneId insertStopPoint(
  double length, autoware_planning_msgs::PathWithLaneId * path)
{
  if (path->points.empty()) {
    return autoware_planning_msgs::PathPointWithLaneId();
  }

  double accumulated_length = 0;
  double insert_idx = 0;
  geometry_msgs::Pose stop_pose;
  for (int i = 1; i < path->points.size(); i++) {
    const auto prev_pose = path->points.at(i - 1).point.pose;
    const auto curr_pose = path->points.at(i).point.pose;
    const double segment_length = getDistance3d(prev_pose.position, curr_pose.position);
    accumulated_length += segment_length;
    if (accumulated_length > length) {
      insert_idx = i;
      const double ratio = 1 - (accumulated_length - length) / segment_length;
      stop_pose = lerpByPose(prev_pose, curr_pose, ratio);
      break;
    }
  }

  autoware_planning_msgs::PathPointWithLaneId stop_point;
  stop_point.lane_ids = path->points.at(insert_idx).lane_ids;
  stop_point.point.pose = stop_pose;
  stop_point.point.type = path->points.at(insert_idx).point.type;
  path->points.insert(path->points.begin() + insert_idx, stop_point);
  for (int i = insert_idx; i < path->points.size(); i++) {
    geometry_msgs::Twist zero_velocity;
    path->points.at(insert_idx).point.twist = zero_velocity;
  }
  return stop_point;
}

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const geometry_msgs::Pose & pose)
{
  const auto arc_pose = lanelet::utils::getArcCoordinates(current_lanes, pose);

  const auto target_center_line = target_lane.centerline().basicLineString();

  geometry_msgs::Pose front_pose, back_pose;

  {
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.front());
    const double front_yaw = lanelet::utils::getLaneletAngle(target_lane, front_point);
    front_pose.position = front_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(tf_quat);
  }

  {
    const auto back_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.back());
    const double back_yaw = lanelet::utils::getLaneletAngle(target_lane, back_point);
    back_pose.position = back_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, back_yaw);
    back_pose.orientation = tf2::toMsg(tf_quat);
  }

  const auto arc_front = lanelet::utils::getArcCoordinates(current_lanes, front_pose);
  const auto arc_back = lanelet::utils::getArcCoordinates(current_lanes, back_pose);

  return std::max(
    std::min(arc_front.length - arc_pose.length, arc_back.length - arc_pose.length), 0.0);
}

std::vector<Polygon> getTargetLaneletPolygons(
  const lanelet::PolygonLayer & map_polygons, lanelet::ConstLanelets & lanelets,
  const geometry_msgs::Pose & pose, const double check_length, const std::string & target_type)
{
  std::vector<Polygon> polygons;

  // create lanelet polygon
  const auto arclength = lanelet::utils::getArcCoordinates(lanelets, pose);
  const auto llt_polygon = lanelet::utils::getPolygonFromArcLength(
    lanelets, arclength.length, arclength.length + check_length);
  const auto llt_polygon_2d = lanelet::utils::to2D(llt_polygon).basicPolygon();

  // If the number of vertices is not enough to create polygon, return empty polygon container
  if (llt_polygon_2d.size() < 3) return polygons;

  Polygon llt_polygon_bg;
  for (const auto & llt_pt : llt_polygon_2d) {
    llt_polygon_bg.outer().push_back(Point(llt_pt.x(), llt_pt.y()));
  }
  llt_polygon_bg.outer().push_back(llt_polygon_bg.outer().front());

  for (const auto & map_polygon : map_polygons) {
    const std::string type = map_polygon.attributeOr(lanelet::AttributeName::Type, "");
    // If the target_type is different or the number of vertices is not enough to create polygon, skip the loop
    if (type == target_type && map_polygon.size() > 2) {
      // create map polygon
      Polygon map_polygon_bg;
      for (const auto & pt : map_polygon) {
        map_polygon_bg.outer().push_back(Point(pt.x(), pt.y()));
      }
      map_polygon_bg.outer().push_back(map_polygon_bg.outer().front());
      if (boost::geometry::intersects(llt_polygon_bg, map_polygon_bg))
        polygons.push_back(map_polygon_bg);
    }
  }
  return polygons;
}

std::vector<Polygon> filterObstaclePolygons(
  const std::vector<Polygon> & obstacle_polygons,
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const double static_obstacle_velocity_thresh)
{
  std::vector<Polygon> filtered_obstacle_polygons;
  for (const auto & obstacle_polygon : obstacle_polygons) {
    for (const auto & obj : objects.objects) {
      const auto velocity = l2Norm(obj.state.twist_covariance.twist.linear);
      if (
        velocity > static_obstacle_velocity_thresh ||
        (obj.semantic.type != autoware_perception_msgs::Semantic::CAR &&
         obj.semantic.type != autoware_perception_msgs::Semantic::TRUCK &&
         obj.semantic.type != autoware_perception_msgs::Semantic::BUS))
        continue;

      // create object polygon
      Polygon obj_polygon;
      if (!calcObjectPolygon(obj, &obj_polygon)) continue;

      // check the object is within the polygon
      if (boost::geometry::within(obj_polygon, obstacle_polygon)) {
        filtered_obstacle_polygons.push_back(obstacle_polygon);
        break;
      }
    }
  }
  return filtered_obstacle_polygons;
}

double getDistanceToNearestObstaclePolygon(
  const std::vector<Polygon> & obstacle_polygons, const geometry_msgs::Pose & pose)
{
  double min_distance = std::numeric_limits<double>::max();
  Point pt(pose.position.x, pose.position.y);
  for (const auto & polygon : obstacle_polygons) {
    const double distance = boost::geometry::distance(polygon, pt);
    if (distance < min_distance) min_distance = distance;
  }
  return min_distance;
}

/*
 * spline interpolation
 */
SplineInterpolate::SplineInterpolate() {}

void SplineInterpolate::generateSpline(
  const std::vector<double> & base_index, const std::vector<double> & base_value)
{
  int N = base_value.size();

  a_.clear();
  b_.clear();
  c_.clear();
  d_.clear();
  h_.clear();

  a_ = base_value;

  for (size_t i = 0; i < N - 1; ++i) {
    h_.push_back(base_index[i + 1] - base_index[i]);
  }

  c_ = solveLinearSystem(1.8, 100);

  for (int i = 0; i < N - 1; i++) {
    d_.push_back((c_[i + 1] - c_[i]) / (3.0 * h_[i]));
    b_.push_back((a_[i + 1] - a_[i]) / h_[i] - h_[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0);
  }

  d_.push_back(0.0);
  b_.push_back(0.0);

  initialized_ = true;
};

double SplineInterpolate::getValue(
  const double & query, const std::vector<double> & base_index) const
{
  if (!initialized_) {
    std::cerr << "[interpolate] spline is uninitialized" << std::endl;
    return 0.0;
  }

  size_t j = 0;
  while (base_index[j] <= query) ++j;
  --j;
  const double ds = query - base_index[j];
  return a_[j] + (b_[j] + (c_[j] + d_[j] * ds) * ds) * ds;
}

bool SplineInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  if (!isValidInput(base_index, base_value, return_index, return_value)) {
    std::cerr << "[interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  // calculate spline coefficients
  generateSpline(base_index, base_value);

  // interpolate values at query points
  for (size_t i = 0; i < return_index.size(); ++i) {
    return_value.push_back(getValue(return_index[i], base_index));
  }
  return true;
}

bool SplineInterpolate::isIncrease(const std::vector<double> & x) const
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] > x[i + 1]) return false;
  }
  return true;
};

bool SplineInterpolate::isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value) const
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cout << "bad index : some vector is empty. base_index: " << base_index.size()
              << ", base_value: " << base_value.size() << ", return_index: " << return_index.size()
              << std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. base_index = ["
              << base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isIncrease(return_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. return_index = ["
              << return_index.front() << ", " << return_index.back() << "]" << std::endl;
    return false;
  }
  if (return_index.front() < base_index.front()) {
    std::cout << "bad index : return_index.front() < base_index.front()" << std::endl;
    return false;
  }
  if (base_index.back() < return_index.back()) {
    std::cout << "bad index : base_index.back() < return_index.back()" << std::endl;
    return false;
  }
  if (base_index.size() != base_value.size()) {
    std::cout << "bad index : base_index.size() != base_value.size()" << std::endl;
    return false;
  }

  return true;
}

std::vector<double> SplineInterpolate::solveLinearSystem(
  const double omega, const size_t max_iter) const
{
  // solve A * ans = rhs by SOR method
  constexpr double converge_range = 0.00001;
  std::vector<double> ans(a_.size(), 1.0);
  std::vector<double> ans_next(a_.size(), 0.0);
  size_t num_iter = 0;

  while (!isConvergeL1(ans, ans_next, converge_range) && num_iter <= max_iter) {
    ans = ans_next;
    for (size_t i = 1; i < a_.size() - 1; ++i) {
      const double rhs = 3.0 / h_[i] * (a_[i + 1] - a_[i]) - 3.0 / h_[i - 1] * (a_[i] - a_[i - 1]);
      ans_next[i] += omega / (2.0 * (h_[i - 1] + h_[i])) *
                     (rhs - (h_[i - 1] * ans_next[i - 1] + 2.0 * (h_[i - 1] + h_[i]) * ans[i] +
                             h_[i] * ans[i + 1]));
    }
    ++num_iter;
  }

  if (num_iter > max_iter) ROS_WARN("[interpolate] unconverged!");
  return ans_next;
}

bool SplineInterpolate::isConvergeL1(
  const std::vector<double> & r1, const std::vector<double> & r2, const double converge_range) const
{
  double d = 0.0;
  for (size_t i = 0; i < r1.size(); ++i) {
    d += std::fabs(r1.at(i) - r2.at(i));
  }
  return d < converge_range;
}

}  // namespace util
}  // namespace lane_change_planner
