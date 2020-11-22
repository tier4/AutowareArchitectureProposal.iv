#include <rclcpp/rclcpp.hpp>

#include "pointcloud_preprocessor/passthrough_filter/passthrough_filter_nodelet.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<pointcloud_preprocessor::PassThroughFilterComponent>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
