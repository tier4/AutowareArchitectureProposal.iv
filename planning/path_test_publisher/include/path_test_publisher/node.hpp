#pragma once
#include <autoware_planning_msgs/Path.h>
#include <ros/ros.h>

class PathTestPublisherNode {
 private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent&);

 public:
  PathTestPublisherNode();
  ~PathTestPublisherNode(){};
};