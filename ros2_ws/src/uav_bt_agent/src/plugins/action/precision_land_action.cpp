#include "uav_bt_agent/plugins/action/precision_land_action.hpp"

namespace uav_bt_agent
{

PrecisionLandAction::PrecisionLandAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    sub_ = node_->create_subscription<flight_core::msg::ArucoMarkers>(
        "/perception/aruco_markers", 10,
        std::bind(&PrecisionLandAction::msgCallback, this, std::placeholders::_1));
        
    move_client_ = rclcpp_action::create_client<MoveTo>(node_, "flight/move_to");
    land_client_ = rclcpp_action::create_client<Land>(node_, "flight/land");
}

BT::PortsList PrecisionLandAction::providedPorts()
{
    return {
        BT::InputPort<int>("marker_id")
    };
}

BT::NodeStatus PrecisionLandAction::onStart()
{
    landing_started_ = false;
    last_msg_ = nullptr;
    
    if (!move_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "MoveTo server not available");
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PrecisionLandAction::onRunning()
{
    int target_id;
    if (!getInput("marker_id", target_id)) {
        return BT::NodeStatus::FAILURE;
    }

    // Check if we found the marker recently
    if (!last_msg_) {
        // No data yet, wait (or fail if too long, handled by Timeout decorator)
        return BT::NodeStatus::RUNNING;
    }

    // Find the specific marker
    bool found = false;
    for (const auto& marker : last_msg_->markers) {
        if (marker.marker_id == (uint64_t)target_id) {
            found = true;
            
            // Marker Position in Map Frame
            double target_x = marker.pose.position.x;
            double target_y = marker.pose.position.y;
            double target_z = marker.pose.position.z; // This is the marker's Z (ground level usually)
            (void)target_z; // Suppress unused var warning
            
            // Visual Servoing Logic
            // We want to be directly above the marker, descending.
            // Current height? We don't have it directly here unless we subscribe to odometry or check marker Z relative to camera.
            // But marker.pose is in MAP frame.
            
            // How high are we?
            // If we trust the map frame, our current Z is roughly (marker_z + relative_z).
            // But wait, aruco_detector transform puts the marker in the MAP frame based on the UAV's current estimate.
            // So marker.pose.z should be close to 0.0 (ground).
            // We want to command the drone to go to (target_x, target_y, current_z - step).
            
            // Getting current Z from TF or subscribing to Odom is best.
            // For now, let's assume we descend blindly towards the marker's Z + offset.
            // But we need to know "current z" to descend smoothly.
            
            // Alternative: Just command (target_x, target_y, 0.0) but with a slow velocity?
            // FlightCore `MoveTo` goes to position. If we say `z=0`, it will descend.
            // But we want to align X/Y first?
            
            // Simple Logic:
            // Continually update the MoveTo goal to (target_x, target_y, 0.0).
            // FlightCore will handle the trajectory.
            // Updating it frequently (visual servoing) corrects for drift.
            
            // Check if we are close enough to land
            // This is hard without knowing our current position.
            // We can check if the marker is centered in the image (requires image coords)
            // Or check if the marker translation is small (but we transformed to map).
            
            // Let's rely on FlightCore. We just update the goal.
            // If FlightCore detects "reached goal", `MoveTo` succeeds.
            
            // BUT, `MoveTo` is an action. If we spam `send_goal`, it might be inefficient.
            // However, 10Hz updates are usually fine.
            
            // Actually, if we just call `Land` action, PX4 typically lands VERTICALLY.
            // Precision landing implies correcting XY during descent.
            // So `MoveTo(x, y, 0)` is effectively a precision land if we keep updating x,y.
            
            sendMoveCommand(target_x, target_y, 0.0, 0.0); // Yaw 0 for now
            
            // If we are very low (e.g. from TF or some other source), we can switch to Land.
            // Since we don't have current Z easily here, let's rely on "MoveTo Z=0" to land
            // or we assume if the marker disappears (too close), we trigger Land.
            
            break;
        }
    }

    if (!found) {
        // Marker lost?
        // If we were tracking it, maybe we are too close?
        // Trigger blind Land?
        RCLCPP_WARN(node_->get_logger(), "PrecisionLand: Marker lost, switching to blind Land");
        
        Land::Goal goal; // empty goal
        auto opts = rclcpp_action::Client<Land>::SendGoalOptions();
        land_client_->async_send_goal(goal, opts);
        
        return BT::NodeStatus::SUCCESS; // Hand off to Land
    }

    return BT::NodeStatus::RUNNING;
}

void PrecisionLandAction::onHalted()
{
    // Cancel moves
    move_client_->async_cancel_all_goals();
}

void PrecisionLandAction::msgCallback(const flight_core::msg::ArucoMarkers::SharedPtr msg)
{
    last_msg_ = msg;
}

void PrecisionLandAction::sendMoveCommand(double x, double y, double z, double yaw)
{
    // Throttle commands (e.g. 5Hz) if needed
    // For now, just send.
    
    MoveTo::Goal goal;
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal.yaw = yaw;
    
    auto opts = rclcpp_action::Client<MoveTo>::SendGoalOptions();
    // We don't wait for result, fire and forget (visual servoing style)
    move_client_->async_send_goal(goal, opts);
}

}
