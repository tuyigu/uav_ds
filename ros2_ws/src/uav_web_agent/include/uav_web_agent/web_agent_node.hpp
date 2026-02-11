#pragma once

#include <rclcpp/rclcpp.hpp>
#include "uav_web_agent/msg/delivery_mission.hpp"
#include "uav_web_agent/msg/mission_status.hpp"
#include "uav_web_agent/msg/operator_command.hpp"
#include "flight_core/msg/uav_state.hpp"

namespace uav_web_agent
{

class WebAgentNode : public rclcpp::Node
{
public:
  using DeliveryMission  = uav_web_agent::msg::DeliveryMission;
  using MissionStatus   = uav_web_agent::msg::MissionStatus;
  using OperatorCommand = uav_web_agent::msg::OperatorCommand;

  explicit WebAgentNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // --- 话题接口 ---
  // 1) 来自 Web 的订单 (例如通过 HTTP → ROS 桥写入)
  rclcpp::Subscription<DeliveryMission>::SharedPtr web_mission_sub_;

  // 2) 提供给行为树读取的订单
  rclcpp::Publisher<DeliveryMission>::SharedPtr bt_mission_pub_;

  // 3) 来自 FlightCore 的无人机状态
  rclcpp::Subscription<flight_core::msg::UavState>::SharedPtr flight_state_sub_;

  // 4) 来自 Web 的操作指令 → 转发给 Orchestrator
  rclcpp::Subscription<OperatorCommand>::SharedPtr web_command_sub_;
  rclcpp::Publisher<OperatorCommand>::SharedPtr uav_command_pub_;

  // 5) 提供给 Web 的无人机状态
  rclcpp::Publisher<flight_core::msg::UavState>::SharedPtr web_uav_state_pub_;
  // Note: /web/mission_status is now published by Orchestrator directly

  // --- 回调 ---
  void on_web_mission(const DeliveryMission::SharedPtr msg);
  void on_web_command(const OperatorCommand::SharedPtr msg);
  void on_flight_state(const flight_core::msg::UavState::SharedPtr msg);
};

}  // namespace uav_web_agent

