#include "uav_web_agent/web_agent_node.hpp"

namespace uav_web_agent
{

WebAgentNode::WebAgentNode(const rclcpp::NodeOptions& options)
  : Node("uav_web_agent", options)
{
  // 1) 来自 Web 的订单 → 转发给行为树
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

  // 3) 行为树上报的任务状态 → 提供给 Web
  bt_mission_status_sub_ = this->create_subscription<MissionStatus>(
    "bt/mission_status", 10,
    std::bind(&WebAgentNode::on_bt_mission_status, this, std::placeholders::_1));

  web_mission_status_pub_ = this->create_publisher<MissionStatus>(
    "web/mission_status", 10);

  RCLCPP_INFO(get_logger(), "uav_web_agent started: bridging Web <-> BT/FlightCore topics.");
}

void WebAgentNode::on_web_mission(const DeliveryMission::SharedPtr msg)
{
  // 简单转发给行为树使用
  RCLCPP_INFO(get_logger(), "Forward mission %s from web to /uav/mission/new",
              msg->mission_id.c_str());
  bt_mission_pub_->publish(*msg);

  // 也可以在这里发一条初始状态给 Web（PENDING）
  MissionStatus status;
  status.mission_id = msg->mission_id;
  status.uav_id     = msg->uav_id;
  status.status     = "PENDING";
  status.reason     = "";
  web_mission_status_pub_->publish(status);
}

void WebAgentNode::on_flight_state(const flight_core::msg::UavState::SharedPtr msg)
{
  // 目前直接透传给 Web 侧，后续你可以按需筛选字段
  web_uav_state_pub_->publish(*msg);
}

void WebAgentNode::on_bt_mission_status(const MissionStatus::SharedPtr msg)
{
  // 行为树（或任务管理节点）上报的任务状态，转发给 Web
  web_mission_status_pub_->publish(*msg);
}

}  // namespace uav_web_agent

