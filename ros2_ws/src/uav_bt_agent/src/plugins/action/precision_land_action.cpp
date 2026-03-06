#include "uav_bt_agent/plugins/action/precision_land_action.hpp"

namespace uav_bt_agent
{

PrecisionLandAction::PrecisionLandAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), current_z_(0.0f)
{
    // Subscribe to fused landing target (from marker_fuser)
    // This transparently receives either YOLO (coarse) or ArUco (precise) detections
    target_sub_ = node_->create_subscription<flight_core::msg::LandingTarget>(
        "/perception/landing_target", 10,
        std::bind(&PrecisionLandAction::targetCallback, this, std::placeholders::_1));

    // Subscribe to UAV state for current height (needed for descent logic)
    state_sub_ = node_->create_subscription<flight_core::msg::UavState>(
        "/flight/uav_state", 10,
        std::bind(&PrecisionLandAction::uavStateCallback, this, std::placeholders::_1));
        
    move_client_ = rclcpp_action::create_client<MoveTo>(node_, "motion/move_to");
    land_client_ = rclcpp_action::create_client<Land>(node_, "motion/land");
}

BT::PortsList PrecisionLandAction::providedPorts()
{
    return {
        BT::InputPort<int>("marker_id"),
        BT::InputPort<float>("descent_step", 1.0f, "Meters to descend per step"),
        BT::InputPort<float>("final_land_height", 1.5f, "Height at which to switch to blind Land")
    };
}

BT::NodeStatus PrecisionLandAction::onStart()
{
    landing_started_ = false;
    last_target_ = nullptr;
    
    if (!move_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "MoveTo server not available");
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PrecisionLandAction::onRunning()
{
    // Check if we have a landing target
    if (!last_target_) {
        return BT::NodeStatus::RUNNING;
    }

    // Check staleness (2.0 second limit)
    auto now = node_->get_clock()->now();
    auto msg_time = rclcpp::Time(last_target_->header.stamp);
    if ((now - msg_time).seconds() > 2.0) {
        RCLCPP_WARN(node_->get_logger(), "PrecisionLand: Target data is stale (%.1fs)",
                    (now - msg_time).seconds());
        last_target_ = nullptr;
        return BT::NodeStatus::RUNNING;
    }

    float descent_step = 1.0f;
    float final_land_height = 1.5f;
    getInput("descent_step", descent_step);
    getInput("final_land_height", final_land_height);

    auto& pose = last_target_->target_pose.pose;
    double target_x = pose.position.x;
    double target_y = pose.position.y;

    // Height-aware descent logic
    float abs_z = std::abs(current_z_);

    if (abs_z <= final_land_height) {
        // We're very low — switch to PX4 native Land for safety
        RCLCPP_INFO(node_->get_logger(),
            "PrecisionLand: Height %.1fm <= %.1fm, switching to blind Land",
            abs_z, final_land_height);

        Land::Goal goal;
        auto opts = rclcpp_action::Client<Land>::SendGoalOptions();
        land_client_->async_send_goal(goal, opts);
        return BT::NodeStatus::SUCCESS;
    }

    // Visual servoing: command drone to be directly above target, descending gradually
    // Compute the next descent altitude
    float next_z = abs_z - descent_step;
    if (next_z < final_land_height) {
        next_z = final_land_height;
    }

    RCLCPP_INFO(node_->get_logger(),
        "PrecisionLand: Servoing to (%.2f, %.2f, %.1f) via %s, current_z=%.1f, precise=%s",
        target_x, target_y, next_z,
        last_target_->source.c_str(), abs_z,
        last_target_->is_precise ? "yes" : "no");

    sendMoveCommand(target_x, target_y, next_z, 0.0);

    return BT::NodeStatus::RUNNING;
}

void PrecisionLandAction::onHalted()
{
    move_client_->async_cancel_all_goals();
}

void PrecisionLandAction::targetCallback(const flight_core::msg::LandingTarget::SharedPtr msg)
{
    // Accept any detection with sufficient confidence
    if (msg->confidence >= 0.3f) {
        last_target_ = msg;
    }
}

void PrecisionLandAction::uavStateCallback(const flight_core::msg::UavState::SharedPtr msg)
{
    current_z_ = msg->z;
}

void PrecisionLandAction::sendMoveCommand(double x, double y, double z, double yaw)
{
    // Throttle commands (2Hz)
    auto now = node_->get_clock()->now();
    if ((now - last_command_time_).seconds() < 0.5) {
        return;
    }
    last_command_time_ = now;
    
    MoveTo::Goal goal;
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal.yaw = yaw;
    
    auto opts = rclcpp_action::Client<MoveTo>::SendGoalOptions();
    move_client_->async_send_goal(goal, opts);
}

}
