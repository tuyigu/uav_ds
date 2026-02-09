#include "uav_bt_agent/plugins/action/move_to_action.hpp"

using namespace std::chrono_literals;

namespace uav_bt_agent
{

MoveToAction::MoveToAction(const std::string& name,
                           const BT::NodeConfiguration& config,
                           const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, config),
    node_(node)
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
  float x, y, z, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) || !getInput("yaw", yaw))
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: missing inputs");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToAction: action server not available");
    return BT::NodeStatus::FAILURE;
  }

  MoveTo::Goal goal;
  goal.x = x;
  goal.y = y;
  goal.z = z;
  goal.yaw = yaw;

  future_goal_handle_ = action_client_->async_send_goal(goal);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToAction::onRunning()
{
  if (future_goal_handle_.valid())
  {
    auto status = future_goal_handle_.wait_for(0s);
    if (status == std::future_status::ready)
    {
      auto goal_handle = future_goal_handle_.get();
      if (!goal_handle)
      {
        RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Goal rejected");
        return BT::NodeStatus::FAILURE;
      }
      
      future_result_ = action_client_->async_get_result(goal_handle);
      future_goal_handle_ = {}; 
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  if (future_result_.valid())
  {
    auto status = future_result_.wait_for(0s);
    if (status == std::future_status::ready)
    {
      auto wrapped_result = future_result_.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result->success)
      {
        RCLCPP_INFO(node_->get_logger(), "MoveToAction: Succeeded");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "MoveToAction: Failed. Message: %s", wrapped_result.result->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

void MoveToAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "MoveToAction halted");
}

}  // namespace uav_bt_agent
