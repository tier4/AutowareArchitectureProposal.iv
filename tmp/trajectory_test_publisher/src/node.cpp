#include "trajectory_test_publisher/node.hpp"

TrajectoryTestPublisherNode::TrajectoryTestPublisherNode() : Node("trajectory_test_publisher_node")
{
  // register publisher
  traj_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>("trajectory", 1);

  // register timer
  auto timer_callback = std::bind(&TrajectoryTestPublisherNode::timerCallback, this);
  const auto period = std::chrono::milliseconds(100);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void TrajectoryTestPublisherNode::timerCallback()
{
  autoware_planning_msgs::msg::Trajectory trajectory_msg;
  trajectory_msg.header.frame_id = "map";
  trajectory_msg.header.stamp = this->get_clock()->now();

  size_t trajectory_size = 100;
  for (size_t i = 0; i < trajectory_size; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = (double)i / 100.0;
    point.pose.position.y = (double)0.0;
    point.pose.position.z = (double)0.0;
    point.pose.orientation.x = (double)0.0;
    point.pose.orientation.y = (double)0.0;
    point.pose.orientation.z = (double)0.0;
    point.pose.orientation.w = (double)1.0;
    point.twist.linear.x = (double)10.0 * ((((double)trajectory_size - 1.0) - (double)i) /
                                           ((double)trajectory_size - 1.0));
    point.twist.linear.y = (double)0.0;
    point.twist.linear.z = (double)0.0;

    trajectory_msg.points.push_back(point);
  }
  traj_pub_->publish(trajectory_msg);
}