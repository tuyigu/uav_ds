#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "flight_core/action/move_to.hpp"

namespace uav_bt_agent
{
class MoveToAction : public BT::StatefulActionNode
{
public:
  using MoveTo = flight_core::action::MoveTo;
  using GoalHandleMoveTo = rclcpp_action::ClientGoalHandle<MoveTo>;

  MoveToAction(const std::string& name,
               const BT::NodeConfig& config,
               const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  
  void handle_feedback(
    GoalHandleMoveTo::SharedPtr,
    const std::shared_ptr<const MoveTo::Feedback> feedback);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveTo>::SharedPtr action_client_;
  std::shared_future<GoalHandleMoveTo::SharedPtr> future_goal_handle_;
  std::shared_future<GoalHandleMoveTo::WrappedResult> future_result_;
  
  float tolerance_;
  float current_distance_;
  bool got_first_feedback_;
};
}  // namespace uav_bt_agent
