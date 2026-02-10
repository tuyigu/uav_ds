#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "flight_core/action/land.hpp"

namespace uav_bt_agent
{
class LandAction : public BT::StatefulActionNode
{
public:
  using Land = flight_core::action::Land;
  using GoalHandleLand = rclcpp_action::ClientGoalHandle<Land>;

  LandAction(const std::string& name,
             const BT::NodeConfig& config,
             const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Land>::SharedPtr action_client_;
  std::shared_future<GoalHandleLand::SharedPtr> future_goal_handle_;
  std::shared_future<GoalHandleLand::WrappedResult> future_result_;
};
}  // namespace uav_bt_agent
