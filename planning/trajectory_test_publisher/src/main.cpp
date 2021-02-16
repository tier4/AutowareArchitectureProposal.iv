#include <ros/ros.h>
#include "trajectory_test_publisher/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_test_publisher");
  TrajectoriTestPublisherNode node;

  ros::spin();

  return 0;
};