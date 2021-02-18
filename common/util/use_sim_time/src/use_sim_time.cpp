// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class UseSimTime : public rclcpp::Node
{
public:
  UseSimTime(const rclcpp::NodeOptions & node_options, const bool use_sim_time);

private:
};

UseSimTime::UseSimTime(const rclcpp::NodeOptions & node_options, const bool use_sim_time)
: rclcpp::Node("use_sim_time", node_options)
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(get_logger(), "start searching nodes");
  rclcpp::Rate(1.0).sleep();
  std::vector<std::string> node_names = get_node_names();

  // remove myself
  node_names.erase(
    std::remove(node_names.begin(), node_names.end(), std::string("/use_sim_time")),
    node_names.end());

  // remove daemon
  node_names.erase(
    std::remove_if(
      node_names.begin(), node_names.end(),
      [](std::string s) {return s.find(std::string("ros2cli_daemon")) != std::string::npos;}),
    node_names.end());

  // remove transform_listener_impl
  node_names.erase(
    std::remove_if(
      node_names.begin(), node_names.end(),
      [](std::string s) {
        return s.find(std::string("transform_listener_impl")) != std::string::npos;
      }),
    node_names.end());

  RCLCPP_INFO(get_logger(), "found %lu nodes.", node_names.size());
  RCLCPP_INFO(get_logger(), "--------------------------------------------");

  for (const auto & n : node_names) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      break;
    }
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, n);
    if (parameters_client->wait_for_service(1s)) {
      if (parameters_client->has_parameter("use_sim_time")) {
        auto set_parameters_results = parameters_client->set_parameters(
        {
          rclcpp::Parameter("use_sim_time", use_sim_time),
        });
        if (set_parameters_results.front().successful) {
          RCLCPP_INFO(get_logger(), "Success, %s", n.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Failure, %s", n.c_str());
        }
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Timeout, %s", n.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "--------------------------------------------");
  rclcpp::shutdown();
}

void usage()
{
  using std::cerr;
  cerr << "Invalid command line.\n\n";
  cerr << "This command will take a bool\n";
  cerr << "Usage:\n";
  cerr << " > use_sim_time true/false ...\n";
  std::exit(1);
}

int main(int argc, char ** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() != 2 || !(args.at(1) == "true" || args.at(1) == "false")) {
    usage();
  }
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);
  const auto use_sim_time = (args.at(1) == "true");
  auto node = std::make_shared<UseSimTime>(options, use_sim_time);
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  exec.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
