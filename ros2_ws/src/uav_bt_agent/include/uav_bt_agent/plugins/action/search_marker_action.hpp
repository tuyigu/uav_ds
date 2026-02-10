#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "flight_core/msg/aruco_markers.hpp"

namespace uav_bt_agent
{
class SearchMarkerAction : public BT::StatefulActionNode
{
public:
    SearchMarkerAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void msgCallback(const flight_core::msg::ArucoMarkers::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<flight_core::msg::ArucoMarkers>::SharedPtr sub_;
    flight_core::msg::ArucoMarkers::SharedPtr last_msg_;
};
}
