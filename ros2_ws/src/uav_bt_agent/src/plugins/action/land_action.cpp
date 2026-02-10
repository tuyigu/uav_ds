#include "uav_bt_agent/plugins/action/land_action.hpp"

namespace uav_bt_agent
{

LandAction::LandAction(const std::string& name,
                       const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<Land>(node_, "flight/land");
}

BT::PortsList LandAction::providedPorts()
{
  return {};
}

BT::NodeStatus LandAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "LandAction: Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  Land::Goal goal;
  auto send_goal_options = rclcpp_action::Client<Land>::SendGoalOptions();
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onRunning()
{
  // 1. Check Goal Handle
  if (future_goal_handle_.valid())
  {
    if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto goal_handle = future_goal_handle_.get();
      future_goal_handle_ = {}; 
      
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Land goal rejected");
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
        RCLCPP_ERROR(node_->get_logger(), "Land action failed");
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

void LandAction::onHalted()
{
  if (action_client_) {
    action_client_->async_cancel_all_goals();
  }
}

}  // namespace uav_bt_agent
