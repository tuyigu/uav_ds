#pragma once
#include <Eigen/Core>
#include <cmath>

namespace fc2::traj {

/// Full trajectory state sampled at time t.
/// Passed directly to PX4Interface for setpoint publishing.
struct TrajectoryState {
    Eigen::Vector3d position     = Eigen::Vector3d::Zero(); ///< ENU m
    Eigen::Vector3d velocity     = Eigen::Vector3d::Zero(); ///< ENU m/s
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); ///< ENU m/s²
    double yaw      = 0.0;   ///< rad, NaN = maintain current
    double yaw_rate = 0.0;   ///< rad/s
    double time     = 0.0;   ///< time within trajectory, s
    bool   valid    = false; ///< false if no active trajectory
};

/// Velocity-only command (for VelocityServo mode, bypasses trajectory layer)
struct VelocityCommand {
    Eigen::Vector3d linear  = Eigen::Vector3d::Zero(); ///< ENU m/s
    double          yaw_rate = 0.0;                    ///< rad/s
    bool            valid    = false;
};

} // namespace fc2::traj
