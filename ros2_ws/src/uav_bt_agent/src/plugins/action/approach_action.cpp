#include "uav_bt_agent/plugins/action/approach_action.hpp"
#include <chrono>

namespace uav_bt_agent {

using FollowPath = flight_core_v2::action::FollowPath;

BT::NodeStatus ApproachAction::onStart()
{
    phase_ = Phase::PLANNING;
    result_ready_ = false;
    success_ = false;

    if (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "[ApproachAction] PlanPath service not available");
        return BT::NodeStatus::FAILURE;
    }

    float gx, gy, gz;
    if (!getInput("goal_x", gx) || !getInput("goal_y", gy) || !getInput("goal_z", gz)) {
        RCLCPP_ERROR(node_->get_logger(), "[ApproachAction] Missing goal inputs");
        return BT::NodeStatus::FAILURE;
    }

    getInput("start_x", start_x_);
    getInput("start_y", start_y_);
    getInput("start_z", start_z_);

    // Send A* plan request
    auto req = std::make_shared<uav_navigation::srv::PlanPath::Request>();
    req->start.x = start_x_; req->start.y = start_y_; req->start.z = start_z_;
    req->goal.x  = gx;       req->goal.y  = gy;       req->goal.z  = gz;

    plan_future_ = plan_client_->async_send_request(req).share();

    RCLCPP_INFO(node_->get_logger(), "[ApproachAction] Sent plan request to (%.1f, %.1f, %.1f)",
                gx, gy, gz);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachAction::onRunning()
{
    // ── Phase 1: Wait for A* plan ────────────────────────────────────────
    if (phase_ == Phase::PLANNING) {
        if (plan_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            return BT::NodeStatus::RUNNING;

        auto response = plan_future_.get();
        if (!response->success || response->waypoints.size() < 2) {
            RCLCPP_ERROR(node_->get_logger(), "[ApproachAction] PlanPath failed: %s",
                         response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "[ApproachAction] Path planned: %zu waypoints",
                    response->waypoints.size());

        // Send FollowPath goal
        if (!follow_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "[ApproachAction] /motion/follow_path not available");
            return BT::NodeStatus::FAILURE;
        }

        FollowPath::Goal goal_msg;
        for (const auto& wp : response->waypoints)
            goal_msg.waypoints.push_back(wp);

        float v_max = 1.5f, j_max = 2.0f;
        getInput("v_max", v_max);
        getInput("j_max", j_max);
        goal_msg.v_max = v_max;
        goal_msg.a_max = 1.0f;
        goal_msg.j_max = j_max;
        goal_msg.arrival_radius = 0.3f;

        auto send_opts = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_opts.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult& wr) {
                success_ = (wr.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                           wr.result->success;
                if (wr.result) {
                    setOutput("final_x", static_cast<float>(wr.result->final_position.x));
                    setOutput("final_y", static_cast<float>(wr.result->final_position.y));
                    setOutput("final_z", static_cast<float>(wr.result->final_position.z));
                }
                result_ready_ = true;
            };

        follow_goal_future_ = follow_action_client_->async_send_goal(goal_msg, send_opts);
        phase_ = Phase::FOLLOWING;
        return BT::NodeStatus::RUNNING;
    }

    // ── Phase 2: Wait for goal handle to be accepted ─────────────────────
    if (phase_ == Phase::FOLLOWING) {
        if (!follow_goal_handle_) {
            if (follow_goal_future_.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready)
                return BT::NodeStatus::RUNNING;
            follow_goal_handle_ = follow_goal_future_.get();
            if (!follow_goal_handle_) {
                RCLCPP_ERROR(node_->get_logger(), "[ApproachAction] FollowPath goal rejected");
                return BT::NodeStatus::FAILURE;
            }
        }

        // Wait for result callback to fire
        if (!result_ready_) return BT::NodeStatus::RUNNING;

        return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

void ApproachAction::onHalted()
{
    if (follow_goal_handle_) {
        follow_action_client_->async_cancel_goal(follow_goal_handle_);
        follow_goal_handle_ = nullptr;
    }
    phase_ = Phase::PLANNING;
    result_ready_ = false;
}

} // namespace uav_bt_agent
