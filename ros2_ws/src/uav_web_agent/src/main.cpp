#include <rclcpp/rclcpp.hpp>
#include "uav_web_agent/web_agent_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_web_agent::WebAgentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

