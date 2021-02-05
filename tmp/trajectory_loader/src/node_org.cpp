#include "trajectory_loader/node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>

TrajectoryLoaderNode::TrajectoryLoaderNode() : nh_(), pnh_("~") {
  pub_ = nh_.advertise<autoware_planning_msgs::Trajectory>("trajectory", 1, true);

  // param
  std::string file_name, frame_id;
  pnh_.getParam("trajectory_file_name", file_name);
  nh_.param("frame_id", frame_id, std::string("map"));

  // load csv file
  ros::Rate loop_rate(1);
  csv file_data;
  association label_row_association_map;
  while (!loadData(file_name, label_row_association_map, file_data) && ros::ok()) {
    loop_rate.sleep();
  }

  // publish
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  publish(header, label_row_association_map, file_data);
}

bool TrajectoryLoaderNode::publish(const std_msgs::Header& header, const association& label_row_association_map,
                                   const csv& file_data) {
  autoware_planning_msgs::Trajectory msg;
  msg.header = header;
  for (size_t i = 0; i < file_data.size(); ++i) {
    autoware_planning_msgs::TrajectoryPoint point;
    double roll, pitch, yaw;

    for (size_t j = 0; j < file_data.at(i).size(); ++j) {
      if (label_row_association_map.at("x") == (int)j) {
        point.pose.position.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("y") == (int)j) {
        point.pose.position.y = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("z") == (int)j) {
        point.pose.position.z = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("roll") == (int)j) {
        roll = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("pitch") == (int)j) {
        pitch = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("yaw") == (int)j) {
        yaw = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("linear_velocity") == (int)j) {
        point.twist.linear.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("angular_velocity") == (int)j) {
        point.twist.angular.z = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("linear_acceleration") == (int)j) {
        point.accel.linear.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("angular_acceleration") == (int)j) {
        point.accel.angular.z = std::stof(file_data.at(i).at(j));
      }
    }
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);
    tf2::convert(point.pose.orientation, tf2_quaternion);
    msg.points.push_back(point);
  }

  pub_.publish(msg);
  return true;
}

bool TrajectoryLoaderNode::loadData(const std::string& file_name, association& label_row_association_map,
                                    csv& file_data) {
  file_data.clear();
  label_row_association_map.clear();
  std::ifstream ifs(file_name);

  /*
   * open file
   */
  if (!ifs) {
    ROS_ERROR("%s cannot load", file_name.c_str());
    return false;
  }

  /*
   * create label-row association map
   */
  std::string line;
  if (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    for (size_t i = 0; i < str_vec.size(); ++i) {
      deleteUnit(str_vec.at(i));
      deleteHeadSpace(str_vec.at(i));
      label_row_association_map[str_vec.at(i)] = (int)i;
    }
  } else {
    ROS_ERROR("cannot create association map");
    return false;
  }

  /*
   * create file data
   */
  while (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    file_data.push_back(str_vec);
  }

  return true;
}

std::vector<std::string> TrajectoryLoaderNode::split(const std::string& input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void TrajectoryLoaderNode::deleteHeadSpace(std::string& string) {
  while (string.find_first_of(' ') == 0) {
    string.erase(string.begin());
    if (string.empty()) break;
  }
}

void TrajectoryLoaderNode::deleteUnit(std::string& string) {
  size_t start_pos, end_pos;
  start_pos = string.find_first_of('[');
  end_pos = string.find_last_of(']');
  if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos) {
    string.erase(start_pos, (end_pos + 1) - start_pos);
  }
}