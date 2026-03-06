#include "flight_core_v2/flight_core_v2.hpp"
#include "flight_core/msg/uav_state.hpp"
#include <chrono>
#include <Eigen/Core>

using namespace std::chrono_literals;

namespace fc2 {

// ─── Constructor ──────────────────────────────────────────────────────────────

FlightCoreV2::FlightCoreV2()
: Node("flight_core_v2"), px4_(this)
{
    // ── State publisher ─────────────────────────────────────────────────
    state_pub_ = create_publisher<flight_core::msg::UavState>("/flight/uav_state", 10);

    // ── Velocity command subscriber (for VelocityServo mode) ────────────
    vel_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/motion/velocity_cmd", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            traj::VelocityCommand cmd;
            // Twist linear is ENU
            cmd.linear  = {msg->linear.x, msg->linear.y, msg->linear.z};
            cmd.yaw_rate = msg->angular.z;
            cmd.valid   = true;
            motion_.setVelocityCommand(cmd);
        });

    // ── Action Servers ──────────────────────────────────────────────────
    using namespace rclcpp_action;

    as_takeoff_ = create_server<Takeoff>(
        this, "motion/takeoff",
        [this](auto& uuid, auto goal)  { return handleTakeoffGoal(uuid, goal); },
        [this](auto handle)            { return handleTakeoffCancel(handle); },
        [this](auto handle)            { handleTakeoffAccepted(handle); });

    as_land_ = create_server<Land>(
        this, "motion/land",
        [this](auto& uuid, auto goal)  { return handleLandGoal(uuid, goal); },
        [this](auto handle)            { return handleLandCancel(handle); },
        [this](auto handle)            { handleLandAccepted(handle); });

    as_moveto_ = create_server<MoveTo>(
        this, "motion/move_to",
        [this](auto& uuid, auto goal)  { return handleMoveToGoal(uuid, goal); },
        [this](auto handle)            { return handleMoveToCancel(handle); },
        [this](auto handle)            { handleMoveToAccepted(handle); });

    as_follow_path_ = create_server<FollowPath>(
        this, "motion/follow_path",
        [this](auto& uuid, auto goal)  { return handleFollowPathGoal(uuid, goal); },
        [this](auto handle)            { return handleFollowPathCancel(handle); },
        [this](auto handle)            { handleFollowPathAccepted(handle); });

    as_vservo_ = create_server<VelocityServo>(
        this, "motion/velocity_servo",
        [this](auto& uuid, auto goal)  { return handleVServoGoal(uuid, goal); },
        [this](auto handle)            { return handleVServoCancel(handle); },
        [this](auto handle)            { handleVServoAccepted(handle); });

    // ── 50 Hz main timer ────────────────────────────────────────────────
    timer_ = create_wall_timer(20ms, [this]() { onTick(); });

    RCLCPP_INFO(get_logger(),
        "FlightCoreV2 initialized — 50Hz single-loop, B-spline+S-curve trajectory");
}

// ─── 50 Hz main loop ──────────────────────────────────────────────────────────

void FlightCoreV2::onTick()
{
    // 1. Read PX4 state
    auto state = px4_.getState();

    // 2. Process any queued Action goals (non-blocking)
    processPendingGoals();

    // 3. Update motion state machine → get setpoint
    auto sp = motion_.update(state, DT);

    // 4. Publish to PX4
    bool vel_only = (motion_.motionState() == MotionState::VELOCITY_SERVO);
    px4_.publishHeartbeat(vel_only);
    px4_.publishSetpoint(sp);

    // 5. Every 5 ticks (10 Hz): feedback + UavState
    if (++tick_count_ % 5 == 0) {
        publishFeedbackAndResults(motion_.progress());
        publishUavState(state);
    }
}

// ─── Process queued goals ─────────────────────────────────────────────────────

void FlightCoreV2::processPendingGoals()
{
    std::lock_guard<std::mutex> lk(q_mutex_);

    // Takeoff
    if (!q_takeoff_.empty()) {
        auto pending = q_takeoff_.front(); q_takeoff_.pop();
        gh_takeoff_ = pending.gh;
        motion_.requestTakeoff(pending.gh->get_goal()->height);
    }

    // Land
    if (!q_land_.empty()) {
        auto pending = q_land_.front(); q_land_.pop();
        gh_land_ = pending.gh;
        motion_.requestLand();
    }

    // MoveTo
    if (!q_moveto_.empty()) {
        auto pending = q_moveto_.front(); q_moveto_.pop();
        gh_moveto_ = pending.gh;
        auto g = pending.gh->get_goal();
        motion_.requestMoveTo(
            {g->x, g->y, g->z}, g->yaw,
            3.0, 1.5, 3.0);
    }

    // FollowPath
    if (!q_follow_path_.empty()) {
        auto pending = q_follow_path_.front(); q_follow_path_.pop();
        gh_follow_path_ = pending.gh;
        auto g = pending.gh->get_goal();
        TrajectoryGoal tg;
        for (const auto& wp : g->waypoints)
            tg.waypoints.emplace_back(wp.x, wp.y, wp.z);
        tg.v_max = g->v_max; tg.a_max = g->a_max; tg.j_max = g->j_max;
        tg.arrival_radius = g->arrival_radius;
        motion_.requestFollowPath(tg);
    }

    // VelocityServo
    if (!q_vservo_.empty()) {
        auto pending = q_vservo_.front(); q_vservo_.pop();
        gh_vservo_ = pending.gh;
        auto g = pending.gh->get_goal();
        motion_.requestVelocityServo(g->timeout_sec, g->z_land_threshold);
    }
}

// ─── Publish feedback and results ─────────────────────────────────────────────

void FlightCoreV2::publishFeedbackAndResults(const MotionProgress& prog)
{
    // ── Takeoff ─────────────────────────────────────────────────────────
    if (gh_takeoff_ && gh_takeoff_->is_active()) {
        if (prog.goal_complete &&
            motion_.motionState() == MotionState::HOVERING) {
            auto result = std::make_shared<Takeoff::Result>();
            result->success = true;
            result->message = "Takeoff complete";
            gh_takeoff_->succeed(result);
            gh_takeoff_ = nullptr;
        } else {
            auto fb = std::make_shared<Takeoff::Feedback>();
            fb->current_z = static_cast<float>(prog.position.z());
            fb->phase = "TAKING_OFF";
            gh_takeoff_->publish_feedback(fb);
        }
    }

    // ── Land ─────────────────────────────────────────────────────────────
    if (gh_land_ && gh_land_->is_active()) {
        if (motion_.motionState() == MotionState::LANDED) {
            auto result = std::make_shared<Land::Result>();
            result->success = true; result->message = "Landed";
            gh_land_->succeed(result);
            gh_land_ = nullptr;
        } else {
            auto fb = std::make_shared<Land::Feedback>();
            fb->current_z = static_cast<float>(prog.position.z());
            fb->phase = "LANDING";
            gh_land_->publish_feedback(fb);
        }
    }

    // ── MoveTo ───────────────────────────────────────────────────────────
    if (gh_moveto_ && gh_moveto_->is_active()) {
        if (prog.goal_complete &&
            motion_.motionState() == MotionState::HOVERING) {
            auto result = std::make_shared<MoveTo::Result>();
            result->success = true;
            gh_moveto_->succeed(result);
            gh_moveto_ = nullptr;
        } else if (prog.goal_failed) {
            auto result = std::make_shared<MoveTo::Result>();
            result->success = false;
            gh_moveto_->abort(result);
            gh_moveto_ = nullptr;
        } else {
            auto fb = std::make_shared<MoveTo::Feedback>();
            fb->current_x = static_cast<float>(prog.position.x());
            fb->current_y = static_cast<float>(prog.position.y());
            fb->current_z = static_cast<float>(prog.position.z());
            fb->distance_to_goal = prog.distance_remaining;
            gh_moveto_->publish_feedback(fb);
        }
    }

    // ── FollowPath ───────────────────────────────────────────────────────
    if (gh_follow_path_ && gh_follow_path_->is_active()) {
        if (prog.goal_complete &&
            motion_.motionState() == MotionState::HOVERING) {
            auto result = std::make_shared<FollowPath::Result>();
            result->success = true;
            result->final_position.x = prog.position.x();
            result->final_position.y = prog.position.y();
            result->final_position.z = prog.position.z();
            gh_follow_path_->succeed(result);
            gh_follow_path_ = nullptr;
        } else if (prog.goal_failed) {
            auto result = std::make_shared<FollowPath::Result>();
            result->success = false;
            gh_follow_path_->abort(result);
            gh_follow_path_ = nullptr;
        } else {
            auto fb = std::make_shared<FollowPath::Feedback>();
            fb->current_waypoint_index = prog.waypoint_index;
            fb->distance_remaining     = prog.distance_remaining;
            fb->current_x = static_cast<float>(prog.position.x());
            fb->current_y = static_cast<float>(prog.position.y());
            fb->current_z = static_cast<float>(prog.position.z());
            gh_follow_path_->publish_feedback(fb);
        }
    }

    // ── VelocityServo ─────────────────────────────────────────────────────
    if (gh_vservo_ && gh_vservo_->is_active()) {
        if (prog.goal_complete) {
            auto result = std::make_shared<VelocityServo::Result>();
            result->success = true; result->message = "z threshold reached";
            gh_vservo_->succeed(result);
            gh_vservo_ = nullptr;
        } else if (prog.goal_failed) {
            auto result = std::make_shared<VelocityServo::Result>();
            result->success = false; result->message = "timeout";
            gh_vservo_->abort(result);
            gh_vservo_ = nullptr;
        } else {
            auto fb = std::make_shared<VelocityServo::Feedback>();
            fb->current_z = static_cast<float>(prog.position.z());
            gh_vservo_->publish_feedback(fb);
        }
    }
}

// ─── UavState publisher ────────────────────────────────────────────────────────

void FlightCoreV2::publishUavState(const DroneState& state)
{
    static const auto state_str = [](MotionState s) -> std::string {
        switch (s) {
        case MotionState::IDLE:              return "IDLE";
        case MotionState::ARMING:            return "ARMING";
        case MotionState::TAKING_OFF:        return "TAKING_OFF";
        case MotionState::HOVERING:          return "HOVERING";
        case MotionState::CRUISING:          return "CRUISING";
        case MotionState::PATH_FOLLOWING:    return "PATH_FOLLOWING";
        case MotionState::LOITER_SCANNING:   return "LOITER_SCANNING";
        case MotionState::VELOCITY_SERVO:    return "VELOCITY_SERVO";
        case MotionState::LANDING:           return "LANDING";
        case MotionState::LANDED:            return "LANDED";
        default:                             return "UNKNOWN";
        }
    };

    flight_core::msg::UavState msg;
    msg.phase     = state_str(motion_.motionState());
    msg.sub_phase = "";
    msg.x         = static_cast<float>(state.position.x());
    msg.y         = static_cast<float>(state.position.y());
    msg.z         = static_cast<float>(state.position.z());
    msg.yaw       = static_cast<float>(state.yaw);
    msg.connected = state.connected;
    msg.armed     = state.armed;
    msg.battery   = state.battery;
    state_pub_->publish(msg);
}

// ─── Action Server callbacks — only queue goals ───────────────────────────────

// TAKEOFF
rclcpp_action::GoalResponse FlightCoreV2::handleTakeoffGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Takeoff::Goal> goal)
{
    if (goal->height < 0.5f || goal->height > 100.f) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected: invalid height %.2f", goal->height);
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FlightCoreV2::handleTakeoffCancel(std::shared_ptr<GH_Takeoff>)
{ motion_.requestHold(); return rclcpp_action::CancelResponse::ACCEPT; }
void FlightCoreV2::handleTakeoffAccepted(std::shared_ptr<GH_Takeoff> gh)
{ std::lock_guard<std::mutex> lk(q_mutex_); q_takeoff_.push({gh}); }

// LAND
rclcpp_action::GoalResponse FlightCoreV2::handleLandGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Land::Goal>)
{ return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
rclcpp_action::CancelResponse FlightCoreV2::handleLandCancel(std::shared_ptr<GH_Land>)
{ return rclcpp_action::CancelResponse::REJECT; } // cannot cancel land for safety
void FlightCoreV2::handleLandAccepted(std::shared_ptr<GH_Land> gh)
{ std::lock_guard<std::mutex> lk(q_mutex_); q_land_.push({gh}); }

// MOVETO
rclcpp_action::GoalResponse FlightCoreV2::handleMoveToGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveTo::Goal>)
{ return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
rclcpp_action::CancelResponse FlightCoreV2::handleMoveToCancel(std::shared_ptr<GH_MoveTo>)
{ motion_.requestHold(); return rclcpp_action::CancelResponse::ACCEPT; }
void FlightCoreV2::handleMoveToAccepted(std::shared_ptr<GH_MoveTo> gh)
{ std::lock_guard<std::mutex> lk(q_mutex_); q_moveto_.push({gh}); }

// FOLLOW_PATH
rclcpp_action::GoalResponse FlightCoreV2::handleFollowPathGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowPath::Goal> goal)
{
    if (goal->waypoints.size() < 2)
        return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FlightCoreV2::handleFollowPathCancel(std::shared_ptr<GH_FollowPath>)
{ motion_.requestHold(); return rclcpp_action::CancelResponse::ACCEPT; }
void FlightCoreV2::handleFollowPathAccepted(std::shared_ptr<GH_FollowPath> gh)
{ std::lock_guard<std::mutex> lk(q_mutex_); q_follow_path_.push({gh}); }

// VELOCITY_SERVO
rclcpp_action::GoalResponse FlightCoreV2::handleVServoGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const VelocityServo::Goal>)
{ return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
rclcpp_action::CancelResponse FlightCoreV2::handleVServoCancel(std::shared_ptr<GH_VServo>)
{ motion_.requestHold(); return rclcpp_action::CancelResponse::ACCEPT; }
void FlightCoreV2::handleVServoAccepted(std::shared_ptr<GH_VServo> gh)
{ std::lock_guard<std::mutex> lk(q_mutex_); q_vservo_.push({gh}); }

} // namespace fc2
