#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "flight_core/action/takeoff.hpp"
#include "flight_core/action/move_to.hpp"
#include "flight_core/action/land.hpp"

namespace uav_bt_agent
{

class SimpleFlightMission : public BT::StatefulActionNode
{
public:
  using Takeoff = flight_core::action::Takeoff;
  using MoveTo  = flight_core::action::MoveTo;
  using Land    = flight_core::action::Land;

  SimpleFlightMission(const std::string& name,
                      const BT::NodeConfiguration& config,
                      const rclcpp::Node::SharedPtr& ros_node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { IDLE, TAKING_OFF, MOVING, LANDING };

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr  move_client_;
  rclcpp_action::Client<Land>::SharedPtr    land_client_;

  Phase phase_{Phase::IDLE};

  float takeoff_height_{5.0f};
  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_z_{0.0f};
  float target_yaw_{0.0f};

  // Futures for actions
  std::shared_future<typename rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr> takeoff_future_;
  std::shared_future<typename rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult> takeoff_result_future_;

  std::shared_future<typename rclcpp_action::ClientGoalHandle<MoveTo>::SharedPtr> move_future_;
  std::shared_future<typename rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult> move_result_future_;

  std::shared_future<typename rclcpp_action::ClientGoalHandle<Land>::SharedPtr> land_future_;
  std::shared_future<typename rclcpp_action::ClientGoalHandle<Land>::WrappedResult> land_result_future_;
};

}  // namespace uav_bt_agent

