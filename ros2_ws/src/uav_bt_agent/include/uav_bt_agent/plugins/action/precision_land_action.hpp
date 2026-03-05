#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "flight_core/msg/aruco_markers.hpp"
#include "flight_core/action/move_to.hpp"
#include "flight_core/action/land.hpp"

namespace uav_bt_agent
{
class PrecisionLandAction : public BT::StatefulActionNode
{
public:
    using MoveTo = flight_core::action::MoveTo;
    using Land = flight_core::action::Land;
    
    PrecisionLandAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void msgCallback(const flight_core::msg::ArucoMarkers::SharedPtr msg);
    void sendMoveCommand(double x, double y, double z, double yaw);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<flight_core::msg::ArucoMarkers>::SharedPtr sub_;
    rclcpp_action::Client<MoveTo>::SharedPtr move_client_;
    rclcpp_action::Client<Land>::SharedPtr land_client_;
    
    flight_core::msg::ArucoMarkers::SharedPtr last_msg_;
    
    // State variables
    // State variables
    bool landing_started_;
    rclcpp::Time last_command_time_;
    
    // std::shared_future<rclcpp_action::ClientGoalHandle<MoveTo>::SharedPtr> move_handle_future_; (Unused?)
};
}
