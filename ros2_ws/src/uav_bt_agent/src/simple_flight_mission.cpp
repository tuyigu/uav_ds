#include "uav_bt_agent/simple_flight_mission.hpp"

using namespace std::chrono_literals;

namespace uav_bt_agent
{

SimpleFlightMission::SimpleFlightMission(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         const rclcpp::Node::SharedPtr& ros_node)
  : BT::StatefulActionNode(name, config),
    node_(ros_node)
{
  takeoff_client_ = rclcpp_action::create_client<Takeoff>(node_, "flight/takeoff");
  move_client_    = rclcpp_action::create_client<MoveTo>(node_, "flight/move_to");
  land_client_    = rclcpp_action::create_client<Land>(node_, "flight/land");
}

BT::PortsList SimpleFlightMission::providedPorts()
{
  return {
    BT::InputPort<float>("takeoff_height"),
    BT::InputPort<float>("target_x"),
    BT::InputPort<float>("target_y"),
    BT::InputPort<float>("target_z"),
    BT::InputPort<float>("target_yaw")
  };
}

BT::NodeStatus SimpleFlightMission::onStart()
{
  // 读取输入端口
  if (!getInput("takeoff_height", takeoff_height_) ||
      !getInput("target_x", target_x_) ||
      !getInput("target_y", target_y_) ||
      !getInput("target_z", target_z_) ||
      !getInput("target_yaw", target_yaw_))
  {
    RCLCPP_ERROR(node_->get_logger(), "SimpleFlightMission: missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  phase_ = Phase::IDLE;

  // 等待 Action Server
  if (!takeoff_client_->wait_for_action_server(2s) ||
      !move_client_->wait_for_action_server(2s) ||
      !land_client_->wait_for_action_server(2s))
  {
    RCLCPP_ERROR(node_->get_logger(), "SimpleFlightMission: action servers not available");
    return BT::NodeStatus::FAILURE;
  }

  // 1. 发送起飞目标
  Takeoff::Goal goal;
  goal.height = takeoff_height_;
  takeoff_future_ = takeoff_client_->async_send_goal(goal);
  phase_ = Phase::TAKING_OFF;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SimpleFlightMission::onRunning()
{
  // 让 ROS2 处理一次回调（简单版）
  rclcpp::spin_some(node_);

  if (phase_ == Phase::TAKING_OFF)
  {
    if (takeoff_future_.valid())
    {
      auto status = takeoff_future_.wait_for(0s);
      if (status == std::future_status::ready)
      {
        auto goal_handle = takeoff_future_.get();
        if (!goal_handle)
        {
          RCLCPP_ERROR(node_->get_logger(), "Takeoff goal rejected");
          return BT::NodeStatus::FAILURE;
        }

        if (!takeoff_result_future_.valid())
        {
          takeoff_result_future_ = takeoff_client_->async_get_result(goal_handle);
          return BT::NodeStatus::RUNNING;
        }

        auto res_status = takeoff_result_future_.wait_for(0s);
        if (res_status == std::future_status::ready)
        {
          auto wrapped_result = takeoff_result_future_.get();
          if (!wrapped_result.result->success)
          {
            RCLCPP_ERROR(node_->get_logger(), "Takeoff failed: %s",
                         wrapped_result.result->message.c_str());
            return BT::NodeStatus::FAILURE;
          }

          // 起飞完成，进入移动阶段
          RCLCPP_INFO(node_->get_logger(), "Takeoff done, start MoveTo");
          MoveTo::Goal move_goal;
          move_goal.x   = target_x_;
          move_goal.y   = target_y_;
          move_goal.z   = target_z_;
          move_goal.yaw = target_yaw_;

          move_future_ = move_client_->async_send_goal(move_goal);
          phase_ = Phase::MOVING;
        }
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::MOVING)
  {
    if (move_future_.valid())
    {
      auto status = move_future_.wait_for(0s);
      if (status == std::future_status::ready)
      {
        auto goal_handle = move_future_.get();
        if (!goal_handle)
        {
          RCLCPP_ERROR(node_->get_logger(), "MoveTo goal rejected");
          return BT::NodeStatus::FAILURE;
        }

        if (!move_result_future_.valid())
        {
          move_result_future_ = move_client_->async_get_result(goal_handle);
          return BT::NodeStatus::RUNNING;
        }

        auto res_status = move_result_future_.wait_for(0s);
        if (res_status == std::future_status::ready)
        {
          auto wrapped_result = move_result_future_.get();
          if (!wrapped_result.result->success)
          {
            RCLCPP_ERROR(node_->get_logger(), "MoveTo failed: %s",
                         wrapped_result.result->message.c_str());
            return BT::NodeStatus::FAILURE;
          }

          // 移动完成，进入降落
          RCLCPP_INFO(node_->get_logger(), "MoveTo done, start Land");
          Land::Goal land_goal;
          land_future_ = land_client_->async_send_goal(land_goal);
          phase_ = Phase::LANDING;
        }
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::LANDING)
  {
    if (land_future_.valid())
    {
      auto status = land_future_.wait_for(0s);
      if (status == std::future_status::ready)
      {
        auto goal_handle = land_future_.get();
        if (!goal_handle)
        {
          RCLCPP_ERROR(node_->get_logger(), "Land goal rejected");
          return BT::NodeStatus::FAILURE;
        }

        if (!land_result_future_.valid())
        {
          land_result_future_ = land_client_->async_get_result(goal_handle);
          return BT::NodeStatus::RUNNING;
        }

        auto res_status = land_result_future_.wait_for(0s);
        if (res_status == std::future_status::ready)
        {
          auto wrapped_result = land_result_future_.get();
          if (!wrapped_result.result->success)
          {
            RCLCPP_ERROR(node_->get_logger(), "Land failed: %s",
                         wrapped_result.result->message.c_str());
            return BT::NodeStatus::FAILURE;
          }

          RCLCPP_INFO(node_->get_logger(), "SimpleFlightMission succeeded");
          phase_ = Phase::IDLE;
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::FAILURE;
}

void SimpleFlightMission::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "SimpleFlightMission halted");
  phase_ = Phase::IDLE;
}

}  // namespace uav_bt_agent

