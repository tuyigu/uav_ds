#include "uav_bt_agent/plugins/action/search_marker_action.hpp"

namespace uav_bt_agent
{

SearchMarkerAction::SearchMarkerAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    // Subscribe to fused landing target (from marker_fuser)
    // This transparently receives either YOLO (coarse) or ArUco (precise) detections
    sub_ = node_->create_subscription<flight_core::msg::LandingTarget>(
        "/perception/landing_target", 10,
        std::bind(&SearchMarkerAction::targetCallback, this, std::placeholders::_1));
}

BT::PortsList SearchMarkerAction::providedPorts()
{
    return {
        BT::InputPort<int>("marker_id"),
        BT::OutputPort<double>("marker_x"),
        BT::OutputPort<double>("marker_y"),
        BT::OutputPort<double>("marker_z"),
        BT::OutputPort<bool>("is_precise")  // Whether source is ArUco (precise) or YOLO (coarse)
    };
}

BT::NodeStatus SearchMarkerAction::onStart()
{
    last_target_ = nullptr;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchMarkerAction::onRunning()
{
    if (!last_target_) {
        // No detection yet, keep running
        return BT::NodeStatus::RUNNING;
    }

    // Check staleness (2.0 second limit)
    auto now = node_->get_clock()->now();
    auto msg_time = rclcpp::Time(last_target_->header.stamp);
    if ((now - msg_time).seconds() > 2.0) {
        RCLCPP_WARN(node_->get_logger(), "SearchMarker: Detection data is stale (%.1fs), ignoring",
                    (now - msg_time).seconds());
        last_target_ = nullptr;
        return BT::NodeStatus::RUNNING;
    }

    auto& pose = last_target_->target_pose.pose;

    RCLCPP_INFO(node_->get_logger(),
        "Landing target found via %s at (%.2f, %.2f, %.2f), conf=%.2f, precise=%s",
        last_target_->source.c_str(),
        pose.position.x, pose.position.y, pose.position.z,
        last_target_->confidence,
        last_target_->is_precise ? "yes" : "no");

    setOutput("marker_x", pose.position.x);
    setOutput("marker_y", pose.position.y);
    setOutput("marker_z", pose.position.z);
    setOutput("is_precise", last_target_->is_precise);

    return BT::NodeStatus::SUCCESS;
}

void SearchMarkerAction::onHalted()
{
}

void SearchMarkerAction::targetCallback(const flight_core::msg::LandingTarget::SharedPtr msg)
{
    // Accept any detection with sufficient confidence
    if (msg->confidence >= 0.3f) {
        last_target_ = msg;
    }
}

}
