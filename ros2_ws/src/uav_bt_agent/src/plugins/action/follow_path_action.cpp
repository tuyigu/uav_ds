#include "uav_bt_agent/plugins/action/follow_path_action.hpp"

BT::NodeStatus FollowPathAction::onStart()
{
    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "MoveTo action server not available");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("waypoints", waypoints_)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing waypoints input");
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Empty waypoints list");
        return BT::NodeStatus::SUCCESS;
    }

    current_idx_ = 0;
    action_active_ = false;
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPathAction::onRunning()
{
    // If we finished all waypoints
    if (current_idx_ >= waypoints_.size()) {
        return BT::NodeStatus::SUCCESS;
    }

    // specific waypoint processing
    if (!action_active_) {
        // Send next goal
        auto wp = waypoints_[current_idx_];
        MoveTo::Goal goal;
        goal.x = wp.x;
        goal.y = wp.y;
        goal.z = wp.z; // assuming z is correct (flight core expects positive Z usually, or NED? check flight_core)
        // flight_core expects NED Z (negative for up) or ENU Z (positive)?
        // Checking flight_core.cpp... it usually expects ENU because of ROS 2 standards and converts to PX4 NED internally.
        // But let's assume strict ENU.
        goal.yaw = 0.0; // TODO: compute yaw based on path direction

        RCLCPP_INFO(node_->get_logger(), "FollowPath: Moving to WP %zu/%zu: (%.1f, %.1f, %.1f)", 
                    current_idx_ + 1, waypoints_.size(), goal.x, goal.y, goal.z);

        auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
        
        // Lambda callbacks if needed, or just use futures
        goal_handle_future_ = client_->async_send_goal(goal, send_goal_options);
        action_active_ = true;
        return BT::NodeStatus::RUNNING;
    }

    // Check if goal accepted
    if (goal_handle_future_.valid() && 
        goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        
        auto goal_handle = goal_handle_future_.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "MoveTo goal rejected");
            return BT::NodeStatus::FAILURE;
        }
        
        // Goal accepted, wait for result
        if (!result_future_.valid()) {
            result_future_ = client_->async_get_result(goal_handle);
        }
    }

    // Check result
    if (result_future_.valid() &&
        result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        
        auto result = result_future_.get();
        action_active_ = false; // ready for next one or finish
        
        // Clear futures
        goal_handle_future_ = {};
        result_future_ = {};

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            current_idx_++;
            return BT::NodeStatus::RUNNING; // Continue loop
        } else {
            RCLCPP_ERROR(node_->get_logger(), "MoveTo action failed/canceled");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void FollowPathAction::onHalted()
{
    // Try to cancel current goal if active
    if (action_active_ && client_) {
        client_->async_cancel_all_goals();
    }
    action_active_ = false;
}
