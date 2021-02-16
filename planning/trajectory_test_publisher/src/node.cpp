#include "trajectory_test_publisher/node.hpp"

TrajectoriTestPublisherNode::TrajectoriTestPublisherNode() : nh_(), pnh_("~") {
  pub_ = nh_.advertise<autoware_planning_msgs::Trajectory>("trajectory", 1);
  timer_ = nh_.createTimer(ros::Duration(0.1), &TrajectoriTestPublisherNode::timerCallback, this);
}

void TrajectoriTestPublisherNode::timerCallback(const ros::TimerEvent&) {
  autoware_planning_msgs::Trajectory trajectory_msg;
  trajectory_msg.header.frame_id = "map";
  trajectory_msg.header.stamp = ros::Time::now();

  size_t trajectory_size = 100;
  for (size_t i = 0; i < trajectory_size; ++i) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = (double)i / 100.0;
    point.pose.position.y = (double)0.0;
    point.pose.position.z = (double)0.0;
    point.pose.orientation.x = (double)0.0;
    point.pose.orientation.y = (double)0.0;
    point.pose.orientation.z = (double)0.0;
    point.pose.orientation.w = (double)1.0;
    point.twist.linear.x =
        (double)10.0 * ((((double)trajectory_size - 1.0) - (double)i) / ((double)trajectory_size - 1.0));
    point.twist.linear.y = (double)0.0;
    point.twist.linear.z = (double)0.0;

    trajectory_msg.points.push_back(point);
  }
  pub_.publish(trajectory_msg);
}