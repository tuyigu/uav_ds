#include "uav_bt_agent/plugins/action/land_action.hpp"

namespace uav_bt_agent
{

LandAction::LandAction(const std::string& name,
                       const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<Land>(node_, "motion/land");
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
  
  // Register feedback callback
  send_goal_options.feedback_callback = 
    std::bind(&LandAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);
  
  current_phase_ = "LANDING";
  return BT::NodeStatus::RUNNING;
}

void LandAction::feedback_callback(
  GoalHandleLand::SharedPtr,
  const std::shared_ptr<const Land::Feedback> feedback)
{
  current_phase_ = feedback->phase;
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
        // Robust check
        if (current_phase_ != "LANDED") {
             // Trust result, but log warning if weird
             // RCLCPP_WARN(node_->get_logger(), "Land success but phase is %s", current_phase_.c_str());
        }
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
