#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>

#include "uav_navigation/srv/plan_path.hpp"
#include "flight_core_v2/action/follow_path.hpp"

namespace uav_bt_agent {

/**
 * @brief ApproachAction — one BT node encapsulating:
 *   1. Call PlanPath Service (A*) → get waypoints
 *   2. Send waypoints to /motion/follow_path Action Server
 *   3. Wait for FollowPath to complete
 *
 * Replaces the old PlanPathAction + FollowPathAction two-step BT sequence.
 *
 * BT Ports:
 *   Input:  goal_x, goal_y, goal_z   — target ENU position
 *           v_max                    — max speed (default 1.5 m/s)
 *           j_max                    — max jerk  (default 2.0 m/s³)
 *   Output: final_x, final_y, final_z
 */
class ApproachAction : public BT::StatefulActionNode {
public:
    ApproachAction(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        plan_client_ = node_->create_client<uav_navigation::srv::PlanPath>(
            "/planning/plan_path");
        follow_action_client_ = rclcpp_action::create_client<FollowPath>(
            node_, "motion/follow_path");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("goal_x"),
            BT::InputPort<float>("goal_y"),
            BT::InputPort<float>("goal_z"),
            BT::InputPort<float>("v_max", 1.5f, "Max speed m/s"),
            BT::InputPort<float>("j_max", 2.0f, "Max jerk m/s3"),
            BT::InputPort<float>("start_x", 0.f, "Start x (0=use current)"),
            BT::InputPort<float>("start_y", 0.f, "Start y"),
            BT::InputPort<float>("start_z", 0.f, "Start z"),
            BT::OutputPort<float>("final_x"),
            BT::OutputPort<float>("final_y"),
            BT::OutputPort<float>("final_z"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using FollowPath = flight_core_v2::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    enum class Phase { PLANNING, FOLLOWING, DONE };
    Phase phase_ = Phase::PLANNING;

    rclcpp::Node::SharedPtr node_;

    // A* planning service client
    rclcpp::Client<uav_navigation::srv::PlanPath>::SharedPtr plan_client_;
    std::shared_future<uav_navigation::srv::PlanPath::Response::SharedPtr> plan_future_;

    // FollowPath action client
    rclcpp_action::Client<FollowPath>::SharedPtr follow_action_client_;
    std::shared_future<GoalHandleFollowPath::SharedPtr> follow_goal_future_;
    GoalHandleFollowPath::SharedPtr follow_goal_handle_;
    std::shared_future<rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult> follow_result_future_;
    bool result_ready_ = false;
    bool success_      = false;

    // Cached start position
    float start_x_ = 0.f, start_y_ = 0.f, start_z_ = 0.f;
};

} // namespace uav_bt_agent
