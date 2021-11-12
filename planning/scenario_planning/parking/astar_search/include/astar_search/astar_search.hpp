// Copyright 2021 The Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef ASTAR_SEARCH__ASTAR_SEARCH_HPP_
#define ASTAR_SEARCH__ASTAR_SEARCH_HPP_

#include <astar_search/visibility_control.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <functional>
#include <queue>
#include <string>
#include <tuple>
#include <vector>


namespace autoware
{
namespace planning
{
namespace astar_search
{

enum class NodeStatus : uint8_t { None, Open, Closed, Obstacle };

struct IndexXYT
{
  int x;
  int y;
  int theta;
};

struct IndexXY
{
  int x;
  int y;
};

struct AstarNode
{
  NodeStatus status = NodeStatus::None;  // node status
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  double gc = 0;                         // actual cost
  double hc = 0;                         // heuristic cost
  bool is_back;                          // true if the current direction of the vehicle is back
  AstarNode * parent = nullptr;          // parent node

  double cost() const {return gc + hc;}
};

struct NodeComparison
{
  bool operator()(const AstarNode * lhs, const AstarNode * rhs)
  {
    return lhs->cost() > rhs->cost();
  }
};

struct AstarWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  bool is_back = false;
};

/// \brief Trajectory points representation as an algorithms output
struct ASTAR_SEARCH_PUBLIC AstarWaypoints
{
  std_msgs::msg::Header header;           ///< Mostly timestamp and frame information
  std::vector<AstarWaypoint> waypoints;   ///< Vector of trajectory waypoints
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double shift_theta;
  double step;
  bool is_back;

  NodeUpdate rotated(const double theta) const
  {
    NodeUpdate result = *this;
    result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
    result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
    return result;
  }

  NodeUpdate flipped() const
  {
    NodeUpdate result = *this;
    result.shift_y = -result.shift_y;
    result.shift_theta = -result.shift_theta;
    return result;
  }

  NodeUpdate reversed() const
  {
    NodeUpdate result = *this;
    result.shift_x = -result.shift_x;
    result.shift_theta = -result.shift_theta;
    result.is_back = !result.is_back;
    return result;
  }
};

/// \brief Definition of essential robot dimensions
struct ASTAR_SEARCH_PUBLIC RobotShape
{
  double length;     ///< Robot's length (bound with X axis direction) [m]
  double width;      ///< Robot's length (bound with Y axis direction)  [m]
  double cg2back;    ///< Robot's distance from center of gravity to back [m]
};

/// \brief Parameters defining algorithm configuration
struct ASTAR_SEARCH_PUBLIC AstarParam
{
  // base configs
  /// Indicate if should search for solutions in backward direction
  bool use_back;
  /// Indicate if solutions should be exclusively behind the goal
  bool only_behind_solutions;
  /// Planning time limit [msec]
  double time_limit;

  // robot configs
  /// Definition of robot shape
  RobotShape robot_shape;
  /// Minimum possible turning radius to plan trajectory [m]
  double minimum_turning_radius;
  /// Maximum possible turning radius to plan trajectory [m]
  double maximum_turning_radius;
  /// Number of levels of discretization between minimum and maximum turning radius [-]
  size_t turning_radius_size;

  // search configs
  /// Number of possible headings, discretized between <0, 2pi> [-]
  size_t theta_size;
  /// Cost of changing moving direction [-]
  double reverse_weight;
  /// Distance weight for trajectory cost estimation
  double distance_heuristic_weight;
  /// Lateral tolerance of goal pose [m]
  double goal_lateral_tolerance;
  /// Longitudinal tolerance of goal pose [m]
  double goal_longitudinal_tolerance;
  /// Angular tolerance of goal pose [rad]
  double goal_angular_tolerance;

  // costmap configs
  /// Threshold value of costmap cell to be regarded as an obstacle [-]
  int64_t obstacle_threshold;
};

/// \brief Possible planning results
enum class SearchStatus
{
  SUCCESS,                      ///< Planning successful
  FAILURE_COLLISION_AT_START,   ///< Collision at start position detected
  FAILURE_COLLISION_AT_GOAL,    ///< Collision at goal position detected
  FAILURE_TIMEOUT_EXCEEDED,     ///< Planning timeout exceeded
  FAILURE_NO_PATH_FOUND         ///< Planner didn't manage to find path
};

/// \brief Determines if passed status is a success status
bool ASTAR_SEARCH_PUBLIC isSuccess(const SearchStatus & status)
{
  return status == SearchStatus::SUCCESS;
}

/// \class AstarSearch
/// \brief A* Hybrid algorithm implementation using ROS2 typical structures
class ASTAR_SEARCH_PUBLIC AstarSearch
{
public:
  using TransitionTable = std::vector<std::vector<NodeUpdate>>;

  /// \brief Default and only constructor for AstarSearch class
  /// \param[in] astar_param Hybrid A* algorithm configuration parameters
  explicit AstarSearch(const AstarParam & astar_param);

  /// \brief Robot dimensions setter
  /// \param[in] robot_shape RobotShape object
  void setRobotShape(const RobotShape & robot_shape) {astar_param_.robot_shape = robot_shape;}

  /// \brief Set occupancy grid for planning
  /// \param[in] costmap nav_msgs::msg::OccupancyGrid type object
  void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap);

  /// \brief Create trajectory plan
  /// \param[in] start_pose Start position
  /// \param[in] goal_pose Goal position
  /// \return SearchStatus flag showing if planning succeeded or not
  SearchStatus makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose);

  /// \brief Check if there will be collision on generated trajectory
  /// \param[in] trajectory Generated trajectory
  /// \return True if detected collision
  bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;

  /// \brief Fetch algorithm solution
  /// \return AstarWaypoints created trajectory
  const AstarWaypoints & getWaypoints() const {return waypoints_;}

private:
  SearchStatus search();
  void setPath(const AstarNode & goal);
  bool setStartNode();
  bool setGoalNode() const;
  double estimateCost(const geometry_msgs::msg::Pose & pose) const;

  bool detectCollision(const IndexXYT & index) const;
  bool isOutOfRange(const IndexXYT & index) const;
  bool isObs(const IndexXYT & index) const;
  bool isGoal(const AstarNode & node) const;

  AstarNode * getNodeRef(const IndexXYT & index);

  AstarParam astar_param_;

  // hybrid astar variables
  TransitionTable transition_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison> openlist_;

  // costmap as occupancy grid
  nav_msgs::msg::OccupancyGrid costmap_;

  // pose in costmap frame
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // result path
  AstarWaypoints waypoints_;
};

}  // namespace astar_search
}  // namespace planning
}  // namespace autoware

#endif  // ASTAR_SEARCH__ASTAR_SEARCH_HPP_
