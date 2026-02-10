#include "uav_bt_agent/plugins/action/move_to_action.hpp"

namespace uav_bt_agent
{

MoveToAction::MoveToAction(const std::string& name,
                           const BT::NodeConfig& config,
                           const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<MoveTo>(node_, "flight/move_to");
}

BT::PortsList MoveToAction::providedPorts()
{
  return {
    BT::InputPort<float>("x"),
    BT::InputPort<float>("y"),
    BT::InputPort<float>("z"),
    BT::InputPort<float>("yaw")
  };
}

BT::NodeStatus MoveToAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  float x, y, z, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  MoveTo::Goal goal;
  goal.x = x;
  goal.y = y;
  goal.z = z;
  goal.yaw = yaw;

  auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToAction::onRunning()
{
  // 1. Wait for Goal Accept
  if (future_goal_handle_.valid()) {
     // Check if ready
     if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto goal_handle = future_goal_handle_.get();
        // future_goal_handle_ = {}; // Don't clear shared_future if valid check relies on it, 
        // but here we want to advance state. 
        // Better: use member state or check if result future is set.
        // For now, I'll clear it BUT shared_future assignment is fine.
        // Actually, if I clear it, next tick logic fails if I structured it poorly.
        // Let's use the logic I wrote before:
        
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "MoveTo goal rejected");
            return BT::NodeStatus::FAILURE;
        }
        
        // Goal accepted, requested result
        // if (!future_result_.valid()) // Only request once
        future_result_ = action_client_->async_get_result(goal_handle);
        
        // Clear goal handle future to avoid re-entering this block?
        // Yes, invalidating it is safe if assigned default constructed.
        future_goal_handle_ = {};
     } else {
        return BT::NodeStatus::RUNNING;
     }
  }

  if (future_result_.valid() &&
      future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    auto result = future_result_.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "MoveTo action failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void MoveToAction::onHalted()
{
  if (action_client_) {
    action_client_->async_cancel_all_goals();
  }
}

}  // namespace uav_bt_agent
