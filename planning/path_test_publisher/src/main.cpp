#include <ros/ros.h>
#include "path_test_publisher/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_test_publisher");
  PathTestPublisherNode node;

  ros::spin();

  return 0;
};