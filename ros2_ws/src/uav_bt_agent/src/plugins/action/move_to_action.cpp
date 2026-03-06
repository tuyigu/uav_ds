#include "uav_bt_agent/plugins/action/move_to_action.hpp"

namespace uav_bt_agent
{

MoveToAction::MoveToAction(const std::string& name,
                           const BT::NodeConfig& config,
                           const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<MoveTo>(node_, "motion/move_to");
}

BT::PortsList MoveToAction::providedPorts()
{
  return {
  BT::InputPort<float>("x"),
    BT::InputPort<float>("y"),
    BT::InputPort<float>("z"),
    BT::InputPort<float>("yaw"),
    BT::InputPort<float>("tolerance")
  };
}

BT::NodeStatus MoveToAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Action server not available");
    return BT::NodeStatus::FAILURE;
  }
  
  float x, y, z, yaw;

  if (!getInput("x", x)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Missing input port 'x'");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Missing input port 'y'");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("z", z)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Missing input port 'z'");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("yaw", yaw)) {
    // Yaw optional? No, let's require it for now to match interface
    // OR we can make it optional and default to current yaw if not provided.
    // For now, strict.
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Missing input port 'yaw'");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("tolerance", tolerance_)) {
    tolerance_ = 0.5f; // Default tolerance
  }
  
  got_first_feedback_ = false;
  current_distance_ = 9999.9f;

  MoveTo::Goal goal;
  goal.x = x;
  goal.y = y;
  goal.z = z;
  goal.yaw = yaw;

  auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
  send_goal_options.feedback_callback = 
      std::bind(&MoveToAction::handle_feedback, this, std::placeholders::_1, std::placeholders::_2);
      
  future_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToAction::onRunning()
{
  // 0. Pre-emptive success check (if close enough)
  if (got_first_feedback_ && current_distance_ < tolerance_) {
      RCLCPP_INFO(node_->get_logger(), 
          "MoveTo: Target reached within tolerance (%.2f < %.2f). Pre-empting success.", 
          current_distance_, tolerance_);
      // Cancel the goal? Or just return SUCCESS?
      // If we return SUCCESS, BT moves on. The Action Client might still be running.
      // We should probably cancel it to be clean, but async_cancel takes time.
      // Simply returning SUCCESS means onHalted won't be called.
      // However, if we don't cancel, the drone might keep trying to refine position.
      // That's acceptable.
      // But if the next action is another MoveTo, it will send a new goal which cancels the old one.
      return BT::NodeStatus::SUCCESS;
  }

  // 1. Wait for Goal Accept
  if (future_goal_handle_.valid()) {
     // Check if ready
     if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto goal_handle = future_goal_handle_.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "MoveTo goal rejected");
            return BT::NodeStatus::FAILURE;
        }
        
        // Goal accepted, requested result
        future_result_ = action_client_->async_get_result(goal_handle);
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

void MoveToAction::handle_feedback(
    GoalHandleMoveTo::SharedPtr,
    const std::shared_ptr<const MoveTo::Feedback> feedback)
{
    current_distance_ = feedback->distance_to_goal;
    got_first_feedback_ = true;
}

}  // namespace uav_bt_agent
