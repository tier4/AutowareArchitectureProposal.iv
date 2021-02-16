#include "path_test_publisher/node.hpp"

PathTestPublisherNode::PathTestPublisherNode() : nh_(), pnh_("~") {
  pub_ = nh_.advertise<autoware_planning_msgs::Path>("path", 1);
  timer_ = nh_.createTimer(ros::Duration(0.1), &PathTestPublisherNode::timerCallback, this);
}

void PathTestPublisherNode::timerCallback(const ros::TimerEvent&) {
  autoware_planning_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();

  size_t path_size = 100;
  for (size_t i = 0; i < path_size; ++i) {
    autoware_planning_msgs::PathPoint point;
    point.pose.position.x = (double)i;
    point.pose.position.y = (double)0.0;
    point.pose.position.z = (double)0.0;
    point.pose.orientation.x = (double)0.0;
    point.pose.orientation.y = (double)0.0;
    point.pose.orientation.z = (double)0.0;
    point.pose.orientation.w = (double)1.0;
    point.twist.linear.x = (double)10.0 * ((((double)path_size - 1.0) - (double)i) / ((double)path_size - 1.0));
    point.twist.linear.y = (double)0.0;
    point.twist.linear.z = (double)0.0;

    path_msg.points.push_back(point);
  }
  pub_.publish(path_msg);
}