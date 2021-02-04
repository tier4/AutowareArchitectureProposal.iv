#pragma once
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

class TrajectoryLoaderNode : public rclcpp::Node
{
private:
  using csv = std::vector<std::vector<std::string>>;
  using association = std::map<std::string /* label */, int /* row */>;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_;

  bool publish(
    const std_msgs::msg::Header & header, const association & label_row_association_map,
    const csv & file_data);
  bool loadData(
    const std::string & file_name, association & label_row_association_map, csv & file_data);
  std::vector<std::string> split(const std::string & input, char delimiter);
  void deleteHeadSpace(std::string & string);
  void deleteUnit(std::string & string);

public:
  TrajectoryLoaderNode();
  ~TrajectoryLoaderNode(){};
};