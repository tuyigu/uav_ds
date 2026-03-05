#include "flight_core/flight_core.hpp"
#include <cmath>
// 移除了不必要的 <iostream> 和 std::placeholders

using namespace std::chrono_literals;

namespace flight_core {

// ======================================================================================
// 构造与初始化
// ======================================================================================

FlightCore::FlightCore() : Node("flight_core"), px4_(this)
{
    // 1. 定时器 (20Hz)：控制回路 + 状态发布
    // [优化] 使用 Lambda 捕获 this 指针替代 std::bind，代码更直观且利于编译器内联优化
    timer_ = this->create_wall_timer(
        50ms, [this]() { on_timer(); });

    // 2. UAV 状态发布器：用于向行为树或其他节点广播当前飞行状态
    state_pub_ = this->create_publisher<flight_core::msg::UavState>(
        "flight/uav_state", 10);

    // 3. 创建 Action Servers
    // [优化] 统一使用 C++14 泛型 Lambda (auto 参数) 处理回调，彻底消除 std::bind 的开销

    // 3.1 Takeoff (起飞) Action 服务器
    takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
        this,
        "flight/takeoff",
        [this](auto uuid, auto goal) { return handle_takeoff_goal(uuid, goal); },
        [this](auto handle) { return handle_takeoff_cancel(handle); },
        [this](auto handle) { handle_takeoff_accepted(handle); }
    );

    // 3.2 Land (降落) Action 服务器
    land_action_server_ = rclcpp_action::create_server<Land>(
        this,
        "flight/land",
        [this](auto uuid, auto goal) { return handle_land_goal(uuid, goal); },
        [this](auto handle) { return handle_land_cancel(handle); },
        [this](auto handle) { handle_land_accepted(handle); }
    );

    // 3.3 MoveTo (移动到目标点) Action 服务器
    move_action_server_ = rclcpp_action::create_server<MoveTo>(
        this,
        "flight/move_to",
        [this](auto uuid, auto goal) { return handle_move_goal(uuid, goal); },
        [this](auto handle) { return handle_move_cancel(handle); },
        [this](auto handle) { handle_move_accepted(handle); }
    );

    RCLCPP_INFO(get_logger(),
                "Flight Core Initialized (基于现代 C++ Lambda 注册 Action 和定时器).");
}

// ======================================================================================
// 辅助函数：飞行阶段枚举转字符串 (用于日志和消息发布)
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
// 核心逻辑：UAV 状态发布
// ======================================================================================

void FlightCore::publish_uav_state(const CurrentState& current, FlightPhase phase)
{
    if (!state_pub_) return;

    flight_core::msg::UavState msg;
    msg.phase = phase_to_string(phase);
    msg.sub_phase = "";  // 目前先留空，后续可由行为树/上层逻辑填充更细粒度状态

    // 坐标与姿态同步
    msg.x = static_cast<float>(current.x);
    msg.y = static_cast<float>(current.y);
    msg.z = static_cast<float>(current.z);

    msg.yaw = static_cast<float>(current.yaw);

    // 硬件连接与解锁状态
    msg.connected = current.connected;
    msg.armed     = current.armed;

    // 从 px4_.get_state() 获取的真实电量百分比
    msg.battery   = current.battery;

    state_pub_->publish(msg);
}

// ======================================================================================
// Takeoff Action 回调组
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_takeoff_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal)
{
    (void)uuid;
    std::lock_guard<std::mutex> lock(fsm_mutex_); // 保护状态机读取

    // 只允许在 IDLE 或 LANDED (地面状态) 时起飞
    if (fsm_.phase() == FlightPhase::IDLE || fsm_.phase() == FlightPhase::LANDED) {
        RCLCPP_INFO(get_logger(), "接收到起飞请求: 目标高度=%.2f", goal->height);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(get_logger(), "拒绝起飞请求: 当前处于非法阶段 %s",
                phase_to_string(fsm_.phase()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse FlightCore::handle_takeoff_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    (void)goal_handle;
    // 起飞过程通常很短且关乎安全，一般不支持中途取消，直接拒绝取消请求
    RCLCPP_WARN(get_logger(), "收到取消起飞的请求 (不支持中断，已忽略).");
    return rclcpp_action::CancelResponse::REJECT;
}

void FlightCore::handle_takeoff_accepted(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    // [优化] 使用 Lambda 启动新线程执行实际起飞逻辑，避免阻塞 ROS 2 调度器
    std::thread{[this, goal_handle]() { execute_takeoff(goal_handle); }}.detach();
}

void FlightCore::execute_takeoff(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    auto current = px4_.get_state();
    if (!current.connected) {
        auto abort_res = std::make_shared<Takeoff::Result>();
        abort_res->success = false;
        abort_res->message = "PX4 未连接，起飞中止";
        goal_handle->abort(abort_res);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);

        // 尝试在状态机中触发起飞
        if (!fsm_.trigger_takeoff(goal->height, current.x, current.y)) {
            auto abort_res = std::make_shared<Takeoff::Result>();
            abort_res->success = false;
            abort_res->message = "状态机 (FSM) 拒绝了起飞操作";
            goal_handle->abort(abort_res);
            return;
        }

        // 起飞物理流程：必须先切入 Offboard 模式，然后再执行 Arm (解锁)g
        px4_.command_switch_offboard();
        px4_.command_arm();
    }

    rclcpp::Rate rate(10); // 10Hz 监控循环

    while (rclcpp::ok()) {
        current = px4_.get_state();
        FlightPhase phase;
        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();
        }

        // 发布进度反馈
        auto feedback = std::make_shared<Takeoff::Feedback>();
        feedback->current_z = static_cast<float>(current.z);
        feedback->phase     = phase_to_string(phase);
        goal_handle->publish_feedback(feedback);

        // 如果状态机切入 HOLDING，说明已经达到目标起飞高度
        if (phase == FlightPhase::HOLDING) {
            auto succ_res = std::make_shared<Takeoff::Result>();
            succ_res->success = true;
            succ_res->message = "起飞成功，进入悬停模式";
            goal_handle->succeed(succ_res);
            return;
        }

        // 处理 Goal 被外部强制 Abort 的情况
        if (!goal_handle->is_active()) {
            return;
        }

        rate.sleep();
    }

    // 节点关闭时的异常退出
    // 取消了不再这里自动 abort() 减少不必要的并发冲突

}

// ======================================================================================
// Land Action 回调组
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_land_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal)
{
    (void)uuid;
    (void)goal;
    std::lock_guard<std::mutex> lock(fsm_mutex_);

    // 只要飞机不在地面上 (IDLE/LANDED)，就允许触发降落
    if (fsm_.phase() == FlightPhase::IDLE || fsm_.phase() == FlightPhase::LANDED) {
        RCLCPP_WARN(get_logger(), "拒绝降落请求: 飞机已经在地面上了.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "接收到降落请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightCore::handle_land_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "收到取消降落的请求 (出于安全考虑不支持取消，已忽略).");
    return rclcpp_action::CancelResponse::REJECT;
}

void FlightCore::handle_land_accepted(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    // [优化] 使用 Lambda 启动新线程执行实际降落逻辑
    std::thread{[this, goal_handle]() { execute_land(goal_handle); }}.detach();
}

void FlightCore::execute_land(
    const std::shared_ptr<GoalHandleLand> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        if (!fsm_.trigger_land()) {
            auto abort_res = std::make_shared<Land::Result>();
            abort_res->success = false;
            abort_res->message = "状态机 (FSM) 拒绝了降落操作";
            goal_handle->abort(abort_res);
            return;
        }

        // 直接调用 PX4 原生的 Auto Land 模式接管降落，更加安全可靠
        px4_.command_switch_land();
    }

    rclcpp::Rate rate(10); // 10Hz 监控循环

    while (rclcpp::ok()) {
        auto current = px4_.get_state();
        FlightPhase phase;
        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();
        }

        auto feedback = std::make_shared<Land::Feedback>();
        feedback->current_z = static_cast<float>(current.z);
        feedback->phase     = phase_to_string(phase);
        goal_handle->publish_feedback(feedback);

        // 如果状态变为 LANDED，说明飞机已经触地并自动上锁 (Disarm)
        if (phase == FlightPhase::LANDED) {
            auto result = std::make_shared<Land::Result>();
            result->success = true;
            result->message = "降落成功";
            goal_handle->succeed(result);
            return;
        }

        if (!goal_handle->is_active()) {
            return;
        }

        rate.sleep();
    }

    // 关闭时不在这里 abort
}

// ======================================================================================
// MoveTo Action 回调组
// ======================================================================================

rclcpp_action::GoalResponse FlightCore::handle_move_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTo::Goal> goal)
{
    (void)uuid;
    (void)goal;
    std::lock_guard<std::mutex> lock(fsm_mutex_);

    // 只有在空中悬停 (HOLDING) 或正在移动 (MOVING) 时才能接收新的移动目标
    if (fsm_.phase() == FlightPhase::HOLDING || fsm_.phase() == FlightPhase::MOVING) {
        RCLCPP_INFO(get_logger(), "接收到 MoveTo (移动) 请求");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_WARN(get_logger(), "拒绝 MoveTo 请求: 当前处于非法阶段 %s",
                phase_to_string(fsm_.phase()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse FlightCore::handle_move_cancel(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    (void)goal_handle;
    // 允许随时取消移动，取消后无人机会在当前位置刹车悬停
    RCLCPP_INFO(get_logger(), "收到取消 MoveTo 的请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightCore::handle_move_accepted(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        if (current_move_goal_handle_ && current_move_goal_handle_->is_active()) {
            RCLCPP_INFO(get_logger(), "中止旧的 MoveTo 任务，执行新目标...");
            // Let the executing thread handle its own abortion.
        }
        current_move_goal_handle_ = goal_handle;
    }
    // [优化] 使用 Lambda 启动新线程
    std::thread{[this, goal_handle]() { execute_move(goal_handle); }}.detach();
}

void FlightCore::execute_move(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(get_logger(), "开始执行 MoveTo: 目标点(%.2f, %.2f, %.2f, yaw=%.2f)",
                goal->x, goal->y, goal->z, goal->yaw);

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        if (!fsm_.trigger_move(goal->x, goal->y, goal->z, goal->yaw)) {
            auto abort_res = std::make_shared<MoveTo::Result>();
            abort_res->success = false;
            abort_res->message = "状态机 (FSM) 拒绝了移动请求";
            goal_handle->abort(abort_res);
            return;
        }
    }

    rclcpp::Rate rate(10); // 10Hz 监控循环

    while (rclcpp::ok()) {
        if (!goal_handle->is_active()) {
            return; // 任务已被终止（如被抢占）
        }

        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            if (current_move_goal_handle_ != goal_handle) {
                // 被新的移动目标抢占，由当前执行线程自行安全中止目标
                auto result = std::make_shared<MoveTo::Result>();
                result->success = false;
                result->message = "被新的移动目标抢占";
                goal_handle->abort(result);
                return;
            }
        }

        // 处理客户端发起的取消请求
        if (goal_handle->is_canceling()) {
            auto current = px4_.get_state();
            {
                std::lock_guard<std::mutex> lock(fsm_mutex_);
                fsm_.trigger_hold(current); // 触发刹车悬停
            }
            RCLCPP_INFO(get_logger(),
                        "MoveTo 已取消: 无人机正在 (%.2f, %.2f, %.2f) 处刹车悬停",
                        current.x, current.y, current.z);
            auto cancel_res = std::make_shared<MoveTo::Result>();
            cancel_res->success = false;
            goal_handle->canceled(cancel_res);
            return;
        }

        auto current = px4_.get_state();
        float dist_to_goal;
        FlightPhase phase;

        {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            phase = fsm_.phase();

            // 计算到目标点的欧氏距离
            float dx = goal->x - current.x;
            float dy = goal->y - current.y;
            float dz = goal->z - current.z;
            dist_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        // 发布距离反馈
        auto feedback = std::make_shared<MoveTo::Feedback>();
        feedback->current_x = current.x;
        feedback->current_y = current.y;
        feedback->current_z = current.z;
        feedback->distance_to_goal = dist_to_goal;
        goal_handle->publish_feedback(feedback);

        // 到达目标点，状态机自动切回 HOLDING
        if (phase == FlightPhase::HOLDING) {
            RCLCPP_INFO(get_logger(), "MoveTo 抵达目标点");
            auto succ_res = std::make_shared<MoveTo::Result>();
            succ_res->success = true;
            goal_handle->succeed(succ_res);
            return;
        }

        rate.sleep();
    }

    // 取消 ROS 节点关闭时的 abort
}

// ======================================================================================
// 核心定时器：控制回路 + 状态发布 (执行频率: 20Hz)
// ======================================================================================

void FlightCore::on_timer()
{
    // 1. Offboard 心跳机制 (PX4 要求在切入 Offboard 前及过程中，必须持续发送设定点)
    px4_.publish_heartbeat();

    // 2. 读取当前状态并更新状态机
    auto current = px4_.get_state();
    Target target_to_pub;
    FlightPhase phase;

    {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        // fsm_ 会在内部计算出一条平滑逼近目标的“虚拟兔子 (Track Target)”轨迹
        fsm_.update_state(current);
        phase = fsm_.phase();
        target_to_pub = fsm_.track_target();
    }

    // 3. 发布 UAV 当前状态（给行为树、前端监控等业务逻辑使用）
    publish_uav_state(current, phase);

    // 4. 根据当前阶段决定是否给 PX4 发送位置设定点
    if (phase == FlightPhase::IDLE || phase == FlightPhase::LANDED) {
        // 在地面不发控制指令
        return;
    }

    if (phase == FlightPhase::LANDING) {
        // LANDING 完全由 PX4 原生 Land 模式接管，不需要下发期望轨迹
        return;
    }

    // TAKING_OFF / HOLDING / MOVING 状态下，向 PX4 下发平滑轨迹目标
    px4_.set_trajectory(target_to_pub);
}

} // namespace flight_core