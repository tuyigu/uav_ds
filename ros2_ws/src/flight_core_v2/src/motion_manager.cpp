#include "flight_core_v2/motion_manager.hpp"
#include <mutex>
#include <cmath>
#include <algorithm>

namespace fc2 {

// ─── Public control requests (called from Action callbacks) ───────────────────

void MotionManager::requestTakeoff(double height)
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g;
    g.type = PendingGoal::Type::TAKEOFF;
    g.target_z = height;
    pending_ = g;
}

void MotionManager::requestLand()
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::LAND;
    pending_ = g;
}

void MotionManager::requestMoveTo(const Eigen::Vector3d& target, double yaw,
                                   double v_max, double a_max, double j_max)
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::MOVE_TO;
    g.traj_goal.waypoints = {target};  // single point
    g.traj_goal.v_max = v_max;
    g.traj_goal.a_max = a_max;
    g.traj_goal.j_max = j_max;
    g.traj_goal.target_yaw = yaw;
    pending_ = g;
}

void MotionManager::requestFollowPath(const TrajectoryGoal& goal)
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::FOLLOW_PATH;
    g.traj_goal = goal;
    pending_ = g;
}

void MotionManager::requestVelocityServo(float timeout_sec, float z_land_threshold)
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::VELOCITY_SERVO;
    g.timeout_sec = timeout_sec;
    g.z_land_thresh = z_land_threshold;
    pending_ = g;
}

void MotionManager::requestHold()
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::HOLD;
    pending_ = g;
}

void MotionManager::requestAbort()
{
    std::lock_guard<std::mutex> lk(pending_mutex_);
    PendingGoal g; g.type = PendingGoal::Type::ABORT;
    pending_ = g;
}

void MotionManager::setVelocityCommand(const traj::VelocityCommand& cmd)
{
    std::lock_guard<std::mutex> lk(vel_cmd_mutex_);
    vel_cmd_ = cmd;
}

// ─── 50 Hz update() ───────────────────────────────────────────────────────────

MotionSetpoint MotionManager::update(const DroneState& state, double dt)
{
    processPending(state);

    MotionSetpoint sp;

    switch (state_) {
    // ── IDLE / LANDED ─────────────────────────────────────────────────────
    case MotionState::IDLE:
    case MotionState::LANDED:
        sp.valid = false;  // don't send any setpoint
        break;

    // ── ARMING: keep sending current position until confirmed ─────────────
    case MotionState::ARMING:
        sp = hoverSetpoint();
        break;

    // ── TAKING_OFF ────────────────────────────────────────────────────────
    case MotionState::TAKING_OFF: {
        // Drive sampler; transition when vertical target reached
        sp = trajectorySetpoint(state, dt);
        if (state.position.z() >= takeoff_target_z_ - 0.2) {
            transitionTo(MotionState::HOVERING, state);
            std::lock_guard<std::mutex> lk(progress_mutex_);
            progress_.goal_complete = true;
        }
        break;
    }

    // ── HOVERING ──────────────────────────────────────────────────────────
    case MotionState::HOVERING:
        sp = hoverSetpoint();
        break;

    // ── CRUISING / PATH_FOLLOWING / LOITER_SCANNING ───────────────────────
    case MotionState::CRUISING:
    case MotionState::PATH_FOLLOWING:
    case MotionState::LOITER_SCANNING: {
        sp = trajectorySetpoint(state, dt);

        if (sampler_.finished()) {
            transitionTo(MotionState::HOVERING, state);
            std::lock_guard<std::mutex> lk(progress_mutex_);
            progress_.goal_complete = true;
        }

        // Update waypoint progress for PATH_FOLLOWING feedback
        {
            std::lock_guard<std::mutex> lk(progress_mutex_);
            progress_.state    = state_;
            progress_.position = state.position;
            if (active_traj_) {
                double s_now = active_traj_->profile().position(sampler_.currentTime());
                double dist_rem = active_traj_->length() - s_now;
                progress_.distance_remaining = static_cast<float>(std::max(0.0, dist_rem));
            }
        }
        break;
    }

    // ── VELOCITY_SERVO ────────────────────────────────────────────────────
    case MotionState::VELOCITY_SERVO: {
        traj::VelocityCommand cmd;
        { std::lock_guard<std::mutex> lk(vel_cmd_mutex_); cmd = vel_cmd_; }

        sp.velocity_only = true;
        sp.valid = true;
        if (cmd.valid) {
            sp.velocity  = cmd.linear;
            sp.yaw_rate  = cmd.yaw_rate;
        } else {
            sp.velocity  = Eigen::Vector3d::Zero(); // hover if no command
        }

        vs_elapsed_ += dt;

        // Check landing threshold
        if (state.position.z() <= static_cast<double>(vs_z_land_thresh_)) {
            transitionTo(MotionState::LANDING, state);
            std::lock_guard<std::mutex> lk(progress_mutex_);
            progress_.goal_complete = true;
        } else if (vs_elapsed_ >= static_cast<double>(vs_timeout_sec_)) {
            transitionTo(MotionState::HOVERING, state);
            std::lock_guard<std::mutex> lk(progress_mutex_);
            progress_.goal_failed = true;
        }

        { std::lock_guard<std::mutex> lk(progress_mutex_);
          progress_.state    = state_;
          progress_.position = state.position; }
        break;
    }

    // ── LANDING ──────────────────────────────────────────────────────────
    // PX4 native Land mode is in charge — we publish nothing
    case MotionState::LANDING:
        sp.valid = false;
        // Detect landed: z close to ground
        if (state.position.z() < 0.15 && !state.armed) {
            transitionTo(MotionState::LANDED, state);
        }
        break;

    default:
        sp.valid = false;
        break;
    }

    return sp;
}

// ─── processPending — consume next goal request ────────────────────────────

void MotionManager::processPending(const DroneState& state)
{
    std::optional<PendingGoal> g;
    {
        std::lock_guard<std::mutex> lk(pending_mutex_);
        if (!pending_) return;
        g = pending_;
        pending_.reset();
    }

    switch (g->type) {
    case PendingGoal::Type::TAKEOFF:
        if (state_ == MotionState::IDLE || state_ == MotionState::LANDED) {
            takeoff_target_z_ = g->target_z;
            // Build a vertical trajectory: current pos → directly above
            TrajectoryGoal vg;
            vg.waypoints = { state.position,
                             Eigen::Vector3d(state.position.x(),
                                             state.position.y(),
                                             g->target_z) };
            vg.v_max = 1.5; vg.a_max = 0.8; vg.j_max = 1.5;
            buildAndStartTraj(vg, state);
            transitionTo(MotionState::TAKING_OFF, state);
        }
        break;

    case PendingGoal::Type::LAND:
        transitionTo(MotionState::LANDING, state);
        break;

    case PendingGoal::Type::MOVE_TO:
    case PendingGoal::Type::FOLLOW_PATH:
        if (state_ == MotionState::HOVERING ||
            state_ == MotionState::CRUISING  ||
            state_ == MotionState::PATH_FOLLOWING) {
            // Prepend current drone position so trajectory starts smoothly
            auto wps = g->traj_goal.waypoints;
            wps.insert(wps.begin(), state.position);
            g->traj_goal.waypoints = wps;
            buildAndStartTraj(g->traj_goal, state);
            transitionTo(g->type == PendingGoal::Type::MOVE_TO
                         ? MotionState::CRUISING
                         : MotionState::PATH_FOLLOWING, state);
        }
        break;

    case PendingGoal::Type::VELOCITY_SERVO:
        vs_timeout_sec_   = g->timeout_sec;
        vs_z_land_thresh_ = g->z_land_thresh;
        vs_elapsed_       = 0.0;
        { std::lock_guard<std::mutex> lk(vel_cmd_mutex_); vel_cmd_ = {}; }
        transitionTo(MotionState::VELOCITY_SERVO, state);
        break;

    case PendingGoal::Type::HOLD:
    case PendingGoal::Type::ABORT:
        transitionTo(MotionState::HOVERING, state);
        break;
    }
}

// ─── buildAndStartTraj ────────────────────────────────────────────────────────

void MotionManager::buildAndStartTraj(const TrajectoryGoal& goal, const DroneState&)
{
    auto traj = std::make_shared<traj::Trajectory>();
    traj->build(goal.waypoints, goal.v_max, goal.a_max, goal.j_max, goal.target_yaw);
    active_traj_ = traj;
    sampler_.setTrajectory(traj);

    std::lock_guard<std::mutex> lk(progress_mutex_);
    progress_.goal_complete = false;
    progress_.goal_failed   = false;
    progress_.waypoint_index = 0;
    progress_.distance_remaining = static_cast<float>(traj->length());
}

// ─── Setpoint helpers ─────────────────────────────────────────────────────────

MotionSetpoint MotionManager::hoverSetpoint() const
{
    MotionSetpoint sp;
    sp.position      = hover_pos_;
    sp.velocity      = Eigen::Vector3d::Zero();
    sp.yaw           = hover_yaw_;
    sp.velocity_only = false;
    sp.valid         = true;
    return sp;
}

MotionSetpoint MotionManager::trajectorySetpoint(const DroneState& state, double dt)
{
    auto ts = sampler_.tick(state.position, dt);
    if (!ts.valid) return hoverSetpoint();

    MotionSetpoint sp;
    sp.position      = ts.position;
    sp.velocity      = ts.velocity;
    sp.yaw           = ts.yaw;
    sp.velocity_only = false;
    sp.valid         = true;
    return sp;
}

// ─── transitionTo ─────────────────────────────────────────────────────────────

void MotionManager::transitionTo(MotionState ns, const DroneState& state)
{
    if (ns == MotionState::HOVERING) {
        // Latch hover position
        hover_pos_ = state.position;
        hover_yaw_ = state.yaw;
        sampler_.reset();
    }
    state_ = ns;
}

// ─── progress() ───────────────────────────────────────────────────────────────

MotionProgress MotionManager::progress() const
{
    std::lock_guard<std::mutex> lk(progress_mutex_);
    return progress_;
}

} // namespace fc2
