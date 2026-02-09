#include "flight_core/flight_core.hpp"
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace flight_core {

// ======================================================================================
// 构造与初始化
// ======================================================================================

FlightCore::FlightCore() : Node("flight_core"), px4_(this)
{
    // 1. 定时器 (20Hz)：控制回路 + 状态发布
    timer_ = this->create_wall_timer(
        50ms, std::bind(&FlightCore::on_timer, this));

    // 2. UAV 状态发布
    state_pub_ = this->create_publisher<flight_core::msg::UavState>(
        "flight/state", 10);

    // 3. 创建 Action Servers

    // 3.1 Takeoff Action
    takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
        this,
        "flight/takeoff",
        std::bind(&FlightCore::handle_takeoff_goal, this, _1, _2),
        std::bind(&FlightCore::handle_takeoff_cancel, this, _1),
        std::bind(&FlightCore::handle_takeoff_accepted, this, _1)
    );

    // 3.2 Land Action
    land_action_server_ = rclcpp_action::create_server<Land>(
        this,
        "flight/land",
        std::bind(&FlightCore::handle_land_goal, this, _1, _2),
        std::bind(&FlightCore::handle_land_cancel, this, _1),
        std::bind(&FlightCore::handle_land_accepted, this, _1)
    );

    // 3.3 MoveTo Action（沿用原有接口）
    move_action_server_ = rclcpp_action::create_server<MoveTo>(
        this,
        "flight/move_to",
        std::bind(&FlightCore::handle_move_goal, this, _1, _2),
        std::bind(&FlightCore::handle_move_cancel, this, _1),
        std::bind(&FlightCore::handle_move_accepted, this, _1)
    );

    RCLCPP_INFO(get_logger(),
                "Flight Core Initialized (Takeoff/Land as Actions, with UAV state publisher).");
}

// ======================================================================================
// 飞行阶段转字符串
// ======================================================================================

std::string FlightCore::phase_to_string(FlightPhase phase) const
{
    switch (phase) {
        case FlightPhase::IDLE:       return "IDLE";
        case FlightPhase::TAKING_OFF: return "TAKING_OFF";
        case FlightPhase::HOLDING:    return "HOLDING";
        case FlightPhase::MOVING:     return "MOVING";
        case FlightPhase::LANDING:    return "LANDING";
        case FlightPhase::LANDED:     return "LANDED";
        default:                      return "UNKNOWN";
    }
}

// ======================================================================================
// UAV 状态发布
// ======================================================================================

void FlightCore::publish_uav_state(const CurrentState& current, FlightPhase phase)
{
    if (!state_pub_) return;

    flight_core::msg::UavState msg;
    msg.phase = phase_to_string(phase);
    msg.sub_phase = "";  // 目前先留空，后续可由行为树/上层逻辑填充更细粒度状态

    msg.x = static_cast<float>(current.x);
    msg.y = static_cast<float>(current.y);
    msg.z = static_cast<float>(current.z);
    msg.yaw = static_cast<float>(current.yaw);

    msg.connected = current.connected;
    msg.armed     = current.armed;

    // 先给个占位值，后续你可以从 PX4Interface 加电量
    msg.battery   = 0.0f;

    state_pub_->publish(msg);
}

// ======================================================================================
// Takeoff Action 回调
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_takeoff_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal)
{
    (void)uuid;
    (void)goal;

    std::lock_guard<std::mutex> lock(fsm_mutex_);

    // 只允许在 IDLE 或 LANDED 时起飞
    if (fsm_.phase() == FlightPhase::IDLE || fsm_.phase() == FlightPhase::LANDED) {
        RCLCPP_INFO(get_logger(), "Received Takeoff goal: height=%.2f", goal->height);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(get_logger(), "Rejecting Takeoff goal: invalid phase %s",
                phase_to_string(fsm_.phase()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse FlightCore::handle_takeoff_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Takeoff cancel requested (not supported, ignoring).");
    return rclcpp_action::CancelResponse::REJECT;
}

void FlightCore::handle_takeoff_accepted(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    std::thread{std::bind(&FlightCore::execute_takeoff, this, _1), goal_handle}.detach();
}

void FlightCore::execute_takeoff(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Takeoff::Feedback>();
    auto result   = std::make_shared<Takeoff::Result>();

    auto current = px4_.get_state();
    if (!current.connected) {
        result->success = false;
        result->message = "PX4 Not Connected";
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);

        if (!fsm_.trigger_takeoff(goal->height, current.x, current.y)) {
            result->success = false;
            result->message = "FSM rejected takeoff";
            goal_handle->abort(result);
            return;
        }

        // 起飞流程：先 Offboard，再 Arm
        px4_.command_switch_offboard();
        px4_.command_arm();
    }

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        current = px4_.get_state();

        FlightPhase phase;
        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();
        }

        feedback->current_z = static_cast<float>(current.z);
        feedback->phase     = phase_to_string(phase);
        goal_handle->publish_feedback(feedback);

        if (phase == FlightPhase::HOLDING) {
            result->success = true;
            result->message = "Takeoff succeeded";
            goal_handle->succeed(result);
            return;
        }

        if (!goal_handle->is_active()) {
            // 被外部终止
            result->success = false;
            result->message = "Takeoff aborted";
            return;
        }

        rate.sleep();
    }

    result->success = false;
    result->message = "Node shutting down";
    goal_handle->abort(result);
}

// ======================================================================================
// Land Action 回调
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_land_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal)
{
    (void)uuid;
    (void)goal;

    std::lock_guard<std::mutex> lock(fsm_mutex_);

    // 只要不在 IDLE/LANDED，就允许降落
    if (fsm_.phase() == FlightPhase::IDLE || fsm_.phase() == FlightPhase::LANDED) {
        RCLCPP_WARN(get_logger(), "Rejecting Land goal: already on ground.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Received Land goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightCore::handle_land_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Land cancel requested (not supported, ignoring).");
    return rclcpp_action::CancelResponse::REJECT;
}

void FlightCore::handle_land_accepted(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    std::thread{std::bind(&FlightCore::execute_land, this, _1), goal_handle}.detach();
}

void FlightCore::execute_land(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    auto feedback = std::make_shared<Land::Feedback>();
    auto result   = std::make_shared<Land::Result>();

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        if (!fsm_.trigger_land()) {
            result->success = false;
            result->message = "FSM rejected land";
            goal_handle->abort(result);
            return;
        }

        // 切 PX4 Land 模式
        px4_.command_switch_land();
    }

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        auto current = px4_.get_state();
        FlightPhase phase;
        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();
        }

        feedback->current_z = static_cast<float>(current.z);
        feedback->phase     = phase_to_string(phase);
        goal_handle->publish_feedback(feedback);

        if (phase == FlightPhase::LANDED) {
            result->success = true;
            result->message = "Land succeeded";
            goal_handle->succeed(result);
            return;
        }

        if (!goal_handle->is_active()) {
            result->success = false;
            result->message = "Land aborted";
            return;
        }

        rate.sleep();
    }

    result->success = false;
    result->message = "Node shutting down";
    goal_handle->abort(result);
}

// ======================================================================================
// MoveTo Action
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_move_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTo::Goal> goal)
{
    (void)uuid;
    (void)goal;
    std::lock_guard<std::mutex> lock(fsm_mutex_);

    if (fsm_.phase() == FlightPhase::HOLDING || fsm_.phase() == FlightPhase::MOVING) {
        RCLCPP_INFO(get_logger(), "Received MoveTo goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(get_logger(), "Rejecting MoveTo goal: invalid phase %s",
                phase_to_string(fsm_.phase()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse FlightCore::handle_move_cancel(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "MoveTo cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightCore::handle_move_accepted(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    std::thread{std::bind(&FlightCore::execute_move, this, _1), goal_handle}.detach();
}

void FlightCore::execute_move(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveTo::Feedback>();
    auto result   = std::make_shared<MoveTo::Result>();

    RCLCPP_INFO(get_logger(), "Executing MoveTo: (%.2f, %.2f, %.2f, yaw=%.2f)",
                goal->x, goal->y, goal->z, goal->yaw);

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        if (!fsm_.trigger_move(goal->x, goal->y, goal->z, goal->yaw)) {
            result->success = false;
            result->message = "FSM rejected move";
            goal_handle->abort(result);
            return;
        }
    }

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            auto current = px4_.get_state();
            {
                std::lock_guard<std::mutex> lock(fsm_mutex_);
                fsm_.trigger_hold(current);
            }
            RCLCPP_INFO(get_logger(),
                        "MoveTo canceled: hold at (%.2f, %.2f, %.2f)",
                        current.x, current.y, current.z);
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        auto current = px4_.get_state();
        float dist_to_goal;
        FlightPhase phase;

        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();

            float dx = goal->x - current.x;
            float dy = goal->y - current.y;
            float dz = (-std::abs(goal->z)) - current.z;
            dist_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        feedback->current_x = current.x;
        feedback->current_y = current.y;
        feedback->current_z = current.z;
        feedback->distance_to_goal = dist_to_goal;
        goal_handle->publish_feedback(feedback);

        if (phase == FlightPhase::HOLDING) {
            RCLCPP_INFO(get_logger(), "MoveTo succeeded");
            result->success = true;
            goal_handle->succeed(result);
            return;
        }

        rate.sleep();
    }

    result->success = false;
    result->message = "Node shutting down";
    goal_handle->abort(result);
}

// ======================================================================================
// 核心定时器：控制回路 + 状态发布
// ======================================================================================

void FlightCore::on_timer()
{
    // 1. Offboard 心跳
    px4_.publish_heartbeat();

    // 2. 读取当前状态
    auto current = px4_.get_state();
    Target target_to_pub;
    FlightPhase phase;

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        fsm_.update_state(current);
        phase = fsm_.phase();
        target_to_pub = fsm_.track_target();
    }

    // 3. 发布 UAV 状态（给行为树、监控等用）
    publish_uav_state(current, phase);

    // 4. 根据阶段决定是否给 PX4 发送轨迹
    if (phase == FlightPhase::IDLE || phase == FlightPhase::LANDED) {
        return;
    }

    if (phase == FlightPhase::LANDING) {
        // 假设 LANDING 完全由 PX4 原生 Land 模式接管
        return;
    }

    // TAKING_OFF / HOLDING / MOVING
    px4_.set_trajectory(target_to_pub);
}

} // namespace flight_core

