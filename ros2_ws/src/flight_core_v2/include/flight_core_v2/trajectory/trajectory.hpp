#pragma once
#include "flight_core_v2/trajectory/spline_path.hpp"
#include "flight_core_v2/trajectory/scurve_planner.hpp"
#include "flight_core_v2/trajectory/types.hpp"
#include <memory>

namespace fc2::traj {

/**
 * @brief Combines SplinePath (geometry) with SCurvePlanner (timing).
 *
 * Given a path and a speed profile, produces full TrajectoryState
 * (position, velocity, acceleration) at any time t ∈ [0, duration()].
 *
 * Usage:
 *   Trajectory traj;
 *   traj.build(waypoints, v_max, a_max, j_max, target_yaw);
 *   TrajectoryState s = traj.sample(t);
 */
class Trajectory {
public:
    /**
     * @brief Build the trajectory from A* waypoints.
     * @param waypoints  ENU positions from A* planner (min 2 points)
     * @param v_max      Max speed m/s
     * @param a_max      Max accel m/s²
     * @param j_max      Max jerk  m/s³
     * @param yaw_target Final yaw (rad). NaN = keep current heading.
     */
    void build(const std::vector<Eigen::Vector3d>& waypoints,
               double v_max, double a_max, double j_max,
               double yaw_target = std::numeric_limits<double>::quiet_NaN());

    /// Sample at time t seconds from trajectory start.
    /// Returns invalid state if not built or t > duration.
    TrajectoryState sample(double t) const;

    double duration() const { return profile_.duration(); }
    double length()   const { return path_.length(); }
    bool   isBuilt()  const { return built_; }

    // Accessors for progress reporting
    const SCurvePlanner& profile() const { return profile_; }
    const SplinePath&    path()    const { return path_; }

private:
    SplinePath    path_;
    SCurvePlanner profile_;
    double yaw_target_ = std::numeric_limits<double>::quiet_NaN();
    bool   built_ = false;
};

} // namespace fc2::traj
