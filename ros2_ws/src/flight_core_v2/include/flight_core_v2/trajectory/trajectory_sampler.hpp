#pragma once
#include "flight_core_v2/trajectory/trajectory.hpp"
#include <Eigen/Core>
#include <memory>

namespace fc2::traj {

/**
 * @brief Lookahead "carrot" tracker that drives a Trajectory.
 *
 * Advances an internal time counter at 50 Hz.  If the drone falls too
 * far behind the nominal position the counter slows down (wait-for-drone
 * logic) – this prevents huge setpoint discontinuities when the UAV is
 * disturbed by wind or recovers from an action switch.
 *
 * The published setpoint is sampled at (current_t + LOOKAHEAD_TIME),
 * giving the PX4 position controller a "carrot" to chase rather than a
 * point it has to correct after passing.
 *
 * Lookahead distance auto-scales with velocity:
 *   lookahead_time = clamp(v_current / a_max, 0.1s, 0.6s)
 */
class TrajectorySampler {
public:
    explicit TrajectorySampler(double lookahead_time_s = 0.4);

    /// Load a new trajectory and reset internal time to 0.
    void setTrajectory(std::shared_ptr<Trajectory> traj);

    /**
     * @brief Advance time and return the next setpoint.
     * @param drone_pos  Current drone ENU position (used for lag detection)
     * @param dt         Time step (seconds), typically 0.02 s at 50 Hz
     * @return           TrajectoryState to send to PX4
     */
    TrajectoryState tick(const Eigen::Vector3d& drone_pos, double dt);

    bool finished() const;
    bool   hasTrajectory() const { return traj_ != nullptr; }
    double currentTime()   const { return current_t_; }

    void reset();

private:
    std::shared_ptr<Trajectory> traj_;
    double current_t_     = 0.0;
    double lookahead_time_ = 0.4; ///< seconds ahead to sample

    static constexpr double MAX_LAG_DIST    = 1.5; ///< m — start slowing time
    static constexpr double PAUSE_LAG_DIST  = 3.0; ///< m — pause time advancement
};

} // namespace fc2::traj
