#include "uav_bt_agent/plugins/action/plan_path_action.hpp"

BT::NodeStatus PlanPathAction::onStart()
{
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "PlanPath service not available");
        return BT::NodeStatus::FAILURE;
    }

    double goal_x, goal_y, goal_z;
    if (!getInput("goal_x", goal_x) ||
        !getInput("goal_y", goal_y) ||
        !getInput("goal_z", goal_z)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing goal inputs for PlanPathAction");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<uav_navigation::srv::PlanPath::Request>();
    request->goal.x = goal_x;
    request->goal.y = goal_y;
    request->goal.z = goal_z;
    // Start position will be handled by the service (using current odometry) if we leave it 0 or pass current pos.
    // Ideally we should pass current pos here, but for now relying on service's odometry sub is easier.

    RCLCPP_INFO(node_->get_logger(), "Planning path to (%.1f, %.1f, %.1f)...", goal_x, goal_y, goal_z);

    future_ = client_->async_send_request(request).share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanPathAction::onRunning()
{
    if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto response = future_.get(); // safe to get now
    if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "Path planned with %zu waypoints", response->waypoints.size());
        setOutput("waypoints", response->waypoints);
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Path planning failed: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

void PlanPathAction::onHalted()
{
    // Cancellation logic if needed
}
