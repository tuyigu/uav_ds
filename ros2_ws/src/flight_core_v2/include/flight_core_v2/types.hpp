#pragma once
#include <limits>
#include <Eigen/Core>

namespace fc2 {

// ── Flight Phase (MotionManager state machine) ───────────────────────────
enum class MotionState {
    IDLE,              ///< On ground, not armed
    ARMING,            ///< Waiting for arm + offboard confirmation
    TAKING_OFF,        ///< Ascending to target height
    HOVERING,          ///< Stable hold at current position
    CRUISING,          ///< Single-point direct flight (high speed, GPS)
    PATH_FOLLOWING,    ///< Multi-waypoint A* path with B-spline+S-curve
    LOITER_SCANNING,   ///< Slow spiral/raster for marker search (YOLO)
    VELOCITY_SERVO,    ///< Vision-based direct velocity command pass-through
    LANDING,           ///< PX4 native Land mode
    LANDED             ///< On ground after landing
};

// ── Shared position state from PX4 (ENU) ────────────────────────────────
struct DroneState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero(); ///< ENU m
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); ///< ENU m/s
    double yaw      = 0.0;   ///< rad
    float  battery  = 1.0f;  ///< 0→1
    bool   connected = false;
    bool   armed     = false;
};

// ── Unified setpoint sent by MotionManager to PX4Interface ───────────────
struct MotionSetpoint {
    Eigen::Vector3d position;        ///< ENU m   (NaN components = ignore)
    Eigen::Vector3d velocity;        ///< ENU m/s (NaN components = ignore)
    double yaw      = std::numeric_limits<double>::quiet_NaN();
    double yaw_rate = std::numeric_limits<double>::quiet_NaN();
    bool velocity_only = false;      ///< true → VelocityServo mode
    bool valid = false;
};

} // namespace fc2
