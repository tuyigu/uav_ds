#include "uav_bt_agent/plugins/action/takeoff_action.hpp"

namespace uav_bt_agent
{

TakeoffAction::TakeoffAction(const std::string& name,
                             const BT::NodeConfig& config,
                             const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<Takeoff>(node_, "flight/takeoff");
}

BT::PortsList TakeoffAction::providedPorts()
{
  return {
    BT::InputPort<float>("height")
  };
}

BT::NodeStatus TakeoffAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  float height;
  if (!getInput("height", height)) {
    RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: Missing height input");
    return BT::NodeStatus::FAILURE;
  }

  Takeoff::Goal goal;
  goal.height = height;

  auto send_goal_options = rclcpp_action::Client<Takeoff>::SendGoalOptions();
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffAction::onRunning()
{
  // 1. Check Goal Handle
  if (future_goal_handle_.valid())
  {
    if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto goal_handle = future_goal_handle_.get();
      future_goal_handle_ = {}; // invalidate
      
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Takeoff goal rejected");
        return BT::NodeStatus::FAILURE;
      }
      
      future_result_ = action_client_->async_get_result(goal_handle);
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  // 2. Check Result
  if (future_result_.valid())
  {
    if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto result = future_result_.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Takeoff action failed");
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void TakeoffAction::onHalted()
{
  if (action_client_) {
    action_client_->async_cancel_all_goals();
  }
}

}  // namespace uav_bt_agent
