#include "rclcpp/rclcpp.hpp"
#include "trajectory_loader/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryLoaderNode>());
  rclcpp::shutdown();
  return 0;
}