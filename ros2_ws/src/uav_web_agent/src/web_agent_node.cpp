#include "uav_web_agent/web_agent_node.hpp"

namespace uav_web_agent
{

WebAgentNode::WebAgentNode(const rclcpp::NodeOptions& options)
  : Node("uav_web_agent", options)
{
  // 1) 来自 Web 的订单 → 转发给 Orchestrator (via /uav/mission/new)
  web_mission_sub_ = this->create_subscription<DeliveryMission>(
    "web/mission_in", 10,
    std::bind(&WebAgentNode::on_web_mission, this, std::placeholders::_1));

  bt_mission_pub_ = this->create_publisher<DeliveryMission>("/uav/mission/new", 10);

  // 2) 来自 flight_core 的无人机状态 → 提供给 Web
  flight_state_sub_ = this->create_subscription<flight_core::msg::UavState>(
    "flight/state", 10,
    std::bind(&WebAgentNode::on_flight_state, this, std::placeholders::_1));

  web_uav_state_pub_ = this->create_publisher<flight_core::msg::UavState>(
    "web/uav_state", 10);

  // 3) 来自 Web 的操作指令 → 转发给 Orchestrator
  web_command_sub_ = this->create_subscription<OperatorCommand>(
    "web/command_in", 10,
    std::bind(&WebAgentNode::on_web_command, this, std::placeholders::_1));

  uav_command_pub_ = this->create_publisher<OperatorCommand>("/uav/command", 10);

  RCLCPP_INFO(get_logger(),
    "uav_web_agent started: bridging Web <-> Orchestrator/FlightCore topics.");
}

void WebAgentNode::on_web_mission(const DeliveryMission::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Forward mission %s from web to /uav/mission/new",
              msg->mission_id.c_str());
  bt_mission_pub_->publish(*msg);
}

void WebAgentNode::on_flight_state(const flight_core::msg::UavState::SharedPtr msg)
{
  web_uav_state_pub_->publish(*msg);
}

void WebAgentNode::on_web_command(const OperatorCommand::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Forward command '%s' (mission=%s) to /uav/command",
              msg->command.c_str(), msg->mission_id.c_str());
  uav_command_pub_->publish(*msg);
}

}  // namespace uav_web_agent
