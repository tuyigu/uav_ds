#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <queue>
#include <memory>

#include "flight_core_v2/px4_interface.hpp"
#include "flight_core_v2/motion_manager.hpp"

// Re-use existing interface types where possible
#include "flight_core/action/takeoff.hpp"
#include "flight_core/action/land.hpp"
#include "flight_core/action/move_to.hpp"
#include "flight_core/msg/uav_state.hpp"
// New action types defined in this package
#include "flight_core_v2/action/follow_path.hpp"
#include "flight_core_v2/action/velocity_servo.hpp"

namespace fc2 {

/**
 * @brief flight_core_v2 main ROS 2 node.
 *
 * Runs a single 50 Hz timer (on_tick) that:
 *   1. Reads PX4 state
 *   2. Processes any pending Action goals (non-blocking queue)
 *   3. Calls MotionManager::update() → produces MotionSetpoint
 *   4. Publishes setpoint to PX4
 *   5. Publishes Action Feedback (every 5 ticks, 10 Hz) + UavState
 *
 * Zero detach threads — all control logic runs in the timer callback.
 * Action Server accept handlers ONLY store the handle + goal into
 * pending queues; the timer processes them.
 */
class FlightCoreV2 : public rclcpp::Node {
public:
    FlightCoreV2();

private:
    // ── Action type aliases ───────────────────────────────────────
    using Takeoff   = flight_core::action::Takeoff;
    using Land      = flight_core::action::Land;
    using MoveTo    = flight_core::action::MoveTo;
    using FollowPath    = flight_core_v2::action::FollowPath;
    using VelocityServo = flight_core_v2::action::VelocityServo;

    using GH_Takeoff     = rclcpp_action::ServerGoalHandle<Takeoff>;
    using GH_Land        = rclcpp_action::ServerGoalHandle<Land>;
    using GH_MoveTo      = rclcpp_action::ServerGoalHandle<MoveTo>;
    using GH_FollowPath  = rclcpp_action::ServerGoalHandle<FollowPath>;
    using GH_VServo      = rclcpp_action::ServerGoalHandle<VelocityServo>;

    // ── Core components ───────────────────────────────────────────
    PX4Interface   px4_;
    MotionManager  motion_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t tick_count_  = 0;
    static constexpr double DT = 0.02; // 50 Hz

    // ── Publishers ────────────────────────────────────────────────
    rclcpp::Publisher<flight_core::msg::UavState>::SharedPtr state_pub_;

    // ── Velocity servo velocity command subscriber ─────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;

    // ── Action Servers ────────────────────────────────────────────
    rclcpp_action::Server<Takeoff>::SharedPtr     as_takeoff_;
    rclcpp_action::Server<Land>::SharedPtr        as_land_;
    rclcpp_action::Server<MoveTo>::SharedPtr      as_moveto_;
    rclcpp_action::Server<FollowPath>::SharedPtr  as_follow_path_;
    rclcpp_action::Server<VelocityServo>::SharedPtr as_vservo_;

    // ── Active goal handles (maintained to send feedback/result) ───
    std::shared_ptr<GH_Takeoff>    gh_takeoff_;
    std::shared_ptr<GH_Land>       gh_land_;
    std::shared_ptr<GH_MoveTo>     gh_moveto_;
    std::shared_ptr<GH_FollowPath> gh_follow_path_;
    std::shared_ptr<GH_VServo>     gh_vservo_;

    // ── Pending goal queues (Action callbacks → timer) ─────────────
    struct PendingTakeoff  { std::shared_ptr<GH_Takeoff>    gh; };
    struct PendingLand     { std::shared_ptr<GH_Land>       gh; };
    struct PendingMoveTo   { std::shared_ptr<GH_MoveTo>     gh; };
    struct PendingFollowPath { std::shared_ptr<GH_FollowPath> gh; };
    struct PendingVServo   { std::shared_ptr<GH_VServo>     gh; };

    std::queue<PendingTakeoff>   q_takeoff_;
    std::queue<PendingLand>      q_land_;
    std::queue<PendingMoveTo>    q_moveto_;
    std::queue<PendingFollowPath> q_follow_path_;
    std::queue<PendingVServo>    q_vservo_;
    std::mutex                   q_mutex_;

    // ── 50 Hz main loop ───────────────────────────────────────────
    void onTick();
    void processPendingGoals();
    void publishFeedbackAndResults(const MotionProgress& prog);
    void publishUavState(const DroneState& state);

    // ── Action Server callbacks (only queue goals) ─────────────────
    // Takeoff
    rclcpp_action::GoalResponse  handleTakeoffGoal  (const rclcpp_action::GoalUUID&, std::shared_ptr<const Takeoff::Goal>);
    rclcpp_action::CancelResponse handleTakeoffCancel(std::shared_ptr<GH_Takeoff>);
    void                          handleTakeoffAccepted(std::shared_ptr<GH_Takeoff>);
    // Land
    rclcpp_action::GoalResponse  handleLandGoal     (const rclcpp_action::GoalUUID&, std::shared_ptr<const Land::Goal>);
    rclcpp_action::CancelResponse handleLandCancel  (std::shared_ptr<GH_Land>);
    void                          handleLandAccepted (std::shared_ptr<GH_Land>);
    // MoveTo
    rclcpp_action::GoalResponse  handleMoveToGoal   (const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveTo::Goal>);
    rclcpp_action::CancelResponse handleMoveToCancel(std::shared_ptr<GH_MoveTo>);
    void                          handleMoveToAccepted(std::shared_ptr<GH_MoveTo>);
    // FollowPath
    rclcpp_action::GoalResponse  handleFollowPathGoal   (const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowPath::Goal>);
    rclcpp_action::CancelResponse handleFollowPathCancel(std::shared_ptr<GH_FollowPath>);
    void                          handleFollowPathAccepted(std::shared_ptr<GH_FollowPath>);
    // VelocityServo
    rclcpp_action::GoalResponse  handleVServoGoal   (const rclcpp_action::GoalUUID&, std::shared_ptr<const VelocityServo::Goal>);
    rclcpp_action::CancelResponse handleVServoCancel(std::shared_ptr<GH_VServo>);
    void                          handleVServoAccepted(std::shared_ptr<GH_VServo>);

    bool alreadyActiveTakeoff() const;
};

} // namespace fc2
