#include "uav_bt_agent/plugins/action/takeoff_action.hpp"

using namespace std::chrono_literals;

namespace uav_bt_agent
{

TakeoffAction::TakeoffAction(const std::string& name,
                             const BT::NodeConfiguration& config,
                             const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config),
    node_(node)
{
  action_client_ = rclcpp_action::create_client<Takeoff>(node_, "flight/takeoff");
}

BT::PortsList TakeoffAction::providedPorts()
{
  return { BT::InputPort<float>("height") };
}

BT::NodeStatus TakeoffAction::onStart()
{
  float height;
  if (!getInput("height", height))
  {
    RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: missing input 'height'");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: action server not available");
    return BT::NodeStatus::FAILURE;
  }

  Takeoff::Goal goal;
  goal.height = height;

  auto send_goal_options = rclcpp_action::Client<Takeoff>::SendGoalOptions();
  
  // Send goal
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffAction::onRunning()
{
  // Check if goal was accepted
  if (future_goal_handle_.valid())
  {
    auto status = future_goal_handle_.wait_for(0s);
    if (status == std::future_status::ready)
    {
      auto goal_handle = future_goal_handle_.get();
      if (!goal_handle)
      {
        RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: Goal rejected");
        return BT::NodeStatus::FAILURE;
      }
      
      // Goal accepted, start waiting for result
      future_result_ = action_client_->async_get_result(goal_handle);
      // Invalidate goal handle future so we don't check it again
      future_goal_handle_ = {}; 
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  // Check result
  if (future_result_.valid())
  {
    auto status = future_result_.wait_for(0s);
    if (status == std::future_status::ready)
    {
      auto wrapped_result = future_result_.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result->success)
      {
        RCLCPP_INFO(node_->get_logger(), "TakeoffAction: Succeeded");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "TakeoffAction: Failed. Message: %s", wrapped_result.result->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

void TakeoffAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "TakeoffAction halted");
  // Ideally, we should cancel the goal here
}

}  // namespace uav_bt_agent
