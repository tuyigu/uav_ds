#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "flight_core/action/takeoff.hpp"

namespace uav_bt_agent
{
class TakeoffAction : public BT::StatefulActionNode
{
public:
  using Takeoff = flight_core::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ClientGoalHandle<Takeoff>;

  TakeoffAction(const std::string& name,
                const BT::NodeConfiguration& config,
                const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Takeoff>::SharedPtr action_client_;
  std::shared_future<GoalHandleTakeoff::SharedPtr> future_goal_handle_;
  std::shared_future<GoalHandleTakeoff::WrappedResult> future_result_;
};
}  // namespace uav_bt_agent
