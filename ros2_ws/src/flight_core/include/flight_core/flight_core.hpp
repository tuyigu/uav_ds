#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mutex>
#include <memory>
#include <thread>

// 基础类型 & PX4 接口 & FSM
#include "flight_core/types.hpp"
#include "flight_core/px4_interface.hpp"
#include "flight_core/flight_fsm.hpp"

// 接口：Actions + UAV 状态消息
#include "flight_core/action/takeoff.hpp"
#include "flight_core/action/land.hpp"
#include "flight_core/action/move_to.hpp"
#include "flight_core/msg/uav_state.hpp"

namespace flight_core {

class FlightCore : public rclcpp::Node {
public:
    FlightCore();

private:
    using Takeoff = flight_core::action::Takeoff;
    using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

    using Land = flight_core::action::Land;
    using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

    using MoveTo = flight_core::action::MoveTo;
    using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

    // --- 核心组件 ---
    FlightFSM fsm_;
    PX4Interface px4_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex fsm_mutex_;

    // --- Action Servers ---
    rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_;
    rclcpp_action::Server<Land>::SharedPtr    land_action_server_;
    rclcpp_action::Server<MoveTo>::SharedPtr  move_action_server_;

    // --- 无人机状态发布 ---
    rclcpp::Publisher<flight_core::msg::UavState>::SharedPtr state_pub_;

    // --- 核心逻辑 ---
    void on_timer();  // 20Hz 控制回路 + 状态发布

    // MoveTo 执行线程
    void execute_move(const std::shared_ptr<GoalHandleMoveTo> goal_handle);

    // Takeoff / Land 执行线程
    void execute_takeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
    void execute_land(const std::shared_ptr<GoalHandleLand> goal_handle);

    // Action 回调：Takeoff
    rclcpp_action::GoalResponse handle_takeoff_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Takeoff::Goal> goal);
    rclcpp_action::CancelResponse handle_takeoff_cancel(
        const std::shared_ptr<GoalHandleTakeoff> goal_handle);
    void handle_takeoff_accepted(
        const std::shared_ptr<GoalHandleTakeoff> goal_handle);

    // Action 回调：Land
    rclcpp_action::GoalResponse handle_land_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Land::Goal> goal);
    rclcpp_action::CancelResponse handle_land_cancel(
        const std::shared_ptr<GoalHandleLand> goal_handle);
    void handle_land_accepted(
        const std::shared_ptr<GoalHandleLand> goal_handle);

    // Action 回调：MoveTo
    rclcpp_action::GoalResponse handle_move_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveTo::Goal> goal);
    rclcpp_action::CancelResponse handle_move_cancel(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle);
    void handle_move_accepted(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle);

    // 状态发布辅助
    void publish_uav_state(const CurrentState& current, FlightPhase phase);

    // 飞行阶段转字符串（给消息和反馈用）
    std::string phase_to_string(FlightPhase phase) const;

    // --- 当前活跃的 Action 句柄 (用于抢占/Preemption) ---
    std::shared_ptr<GoalHandleMoveTo> current_move_goal_handle_;
};

} // namespace flight_core

