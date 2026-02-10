#include "uav_bt_agent/plugins/action/search_marker_action.hpp"

namespace uav_bt_agent
{

SearchMarkerAction::SearchMarkerAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    sub_ = node_->create_subscription<flight_core::msg::ArucoMarkers>(
        "/perception/aruco_markers", 10,
        std::bind(&SearchMarkerAction::msgCallback, this, std::placeholders::_1));
}

BT::PortsList SearchMarkerAction::providedPorts()
{
    return {
        BT::InputPort<int>("marker_id"),
        BT::OutputPort<double>("marker_x"),
        BT::OutputPort<double>("marker_y"),
        BT::OutputPort<double>("marker_z")
    };
}

BT::NodeStatus SearchMarkerAction::onStart()
{
    last_msg_ = nullptr;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchMarkerAction::onRunning()
{
    if (!last_msg_) {
        // Keep running until we get a message
        return BT::NodeStatus::RUNNING;
    }

    int target_id;
    if (!getInput("marker_id", target_id)) {
        RCLCPP_ERROR(node_->get_logger(), "SearchMarkerAction: Missing marker_id");
        return BT::NodeStatus::FAILURE;
    }

    for (const auto& marker : last_msg_->markers) {
        if (marker.marker_id == (uint64_t)target_id) {
            RCLCPP_INFO(node_->get_logger(), "Marker %d found at (%.2f, %.2f, %.2f)",
                        target_id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            
            setOutput("marker_x", marker.pose.position.x);
            setOutput("marker_y", marker.pose.position.y);
            setOutput("marker_z", marker.pose.position.z);
            return BT::NodeStatus::SUCCESS;
        }
    }

    // Processed latest message but didn't find marker, keep running
    // (A timeout node in the tree will handle long waits)
    return BT::NodeStatus::RUNNING;
}

void SearchMarkerAction::onHalted()
{
}

void SearchMarkerAction::msgCallback(const flight_core::msg::ArucoMarkers::SharedPtr msg)
{
    last_msg_ = msg;
}

}
