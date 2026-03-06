#pragma once
#include "flight_core_v2/types.hpp"
#include "flight_core_v2/trajectory/trajectory.hpp"
#include "flight_core_v2/trajectory/trajectory_sampler.hpp"
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <optional>
#include <deque>
#include <mutex>

namespace fc2 {

/// Parameters for a single trajectory request injected by an Action Server.
struct TrajectoryGoal {
    std::vector<Eigen::Vector3d> waypoints;
    double v_max = 2.0;
    double a_max = 1.0;
    double j_max = 2.0;
    double arrival_radius = 0.3;
    double target_yaw = std::numeric_limits<double>::quiet_NaN();
    // For takeoff: special single-waypoint vertical-only
    bool vertical_only = false;
};

/// Result feedback for the active action goal
struct MotionProgress {
    MotionState state = MotionState::IDLE;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    float distance_remaining = 0.f;
    uint32_t waypoint_index  = 0;
    bool goal_complete = false;
    bool goal_failed   = false;
};

/**
 * @brief Core motion state machine and trajectory executor.
 *
 * Called once per 50 Hz tick by FlightCoreV2::on_tick().
 * Thread-safe: external Action Server callbacks inject goals via
 * setNextGoal() which is mutex-protected.
 *
 * State transitions:
 *  IDLE → ARMING → TAKING_OFF → HOVERING
 *  HOVERING → CRUISING / PATH_FOLLOWING / LOITER_SCANNING / VELOCITY_SERVO
 *  * → HOVERING (when goal completes or cancelled)
 *  * → LANDING → LANDED
 */
class MotionManager {
public:
    MotionManager() = default;

    // ── Called from 50 Hz timer ───────────────────────────────────
    /// Main update.  Returns the setpoint to send to PX4.
    MotionSetpoint update(const DroneState& state, double dt);

    /// Current internal state (for UavState publishing)
    MotionState motionState() const { return state_; }

    /// Latest progress snapshot (for Action Feedback publishing)
    MotionProgress progress() const;

    // ── Called from Action Server callbacks (thread-safe) ─────────
    void requestTakeoff(double height);
    void requestLand();
    void requestMoveTo(const Eigen::Vector3d& target, double yaw,
                        double v_max = 3.0, double a_max = 1.5, double j_max = 3.0);
    void requestFollowPath(const TrajectoryGoal& goal);
    void requestVelocityServo(float timeout_sec, float z_land_threshold);
    void requestHold();     ///< Emergency hold at current position
    void requestAbort();    ///< Emergency abort → HOVERING at current pos

    /// Feed live velocity command for VELOCITY_SERVO mode.
    void setVelocityCommand(const traj::VelocityCommand& cmd);

private:
    MotionState state_ = MotionState::IDLE;

    // ── Trajectory execution ──────────────────────────────────────
    traj::TrajectorySampler sampler_;
    std::shared_ptr<traj::Trajectory> active_traj_;

    // ── Pending goal (set by Action callbacks, consumed by update()) ─
    struct PendingGoal {
        enum class Type { TAKEOFF, LAND, MOVE_TO, FOLLOW_PATH, VELOCITY_SERVO, HOLD, ABORT } type;
        TrajectoryGoal traj_goal;
        double target_z      = 0.0; // takeoff height
        float  timeout_sec   = 30.f;
        float  z_land_thresh = 0.4f;
    };
    std::optional<PendingGoal> pending_;
    mutable std::mutex pending_mutex_;

    // ── Live velocity servo ───────────────────────────────────────
    traj::VelocityCommand vel_cmd_;
    mutable std::mutex    vel_cmd_mutex_;
    float   vs_timeout_sec_    = 30.f;
    float   vs_z_land_thresh_  = 0.4f;
    double  vs_elapsed_        = 0.0;

    // ── Hover hold ────────────────────────────────────────────────
    Eigen::Vector3d hover_pos_ = Eigen::Vector3d::Zero();
    double          hover_yaw_ = 0.0;

    // ── Takeoff ───────────────────────────────────────────────────
    double takeoff_target_z_ = 5.0;
    bool   arm_issued_       = false;

    // ── Progress tracking ─────────────────────────────────────────
    mutable std::mutex progress_mutex_;
    MotionProgress progress_;

    // ── Internal helpers ──────────────────────────────────────────
    void processPending(const DroneState& state);
    void buildAndStartTraj(const TrajectoryGoal& goal, const DroneState& state);
    MotionSetpoint hoverSetpoint() const;
    MotionSetpoint trajectorySetpoint(const DroneState& state, double dt);
    void transitionTo(MotionState ns, const DroneState& state);
    std::string stateStr() const;
};

} // namespace fc2
