#include "flight_core_v2/trajectory/trajectory_sampler.hpp"
#include <cmath>
#include <algorithm>

namespace fc2::traj {

TrajectorySampler::TrajectorySampler(double lookahead_time_s)
    : lookahead_time_(lookahead_time_s) {}

void TrajectorySampler::setTrajectory(std::shared_ptr<Trajectory> traj)
{
    traj_ = traj;
    current_t_ = 0.0;
}

void TrajectorySampler::reset()
{
    traj_ = nullptr;
    current_t_ = 0.0;
}

bool TrajectorySampler::finished() const
{
    if (!traj_) return true;
    return current_t_ >= traj_->duration();
}

TrajectoryState TrajectorySampler::tick(const Eigen::Vector3d& drone_pos, double dt)
{
    if (!traj_) return {};

    // ── Advance time, slowing down if drone lags ──────────────────────
    TrajectoryState nominal = traj_->sample(current_t_);
    double dist_to_nominal  = (drone_pos - nominal.position).norm();

    double time_scale = 1.0;
    if (dist_to_nominal > PAUSE_LAG_DIST) {
        time_scale = 0.0; // fully pause: wait for drone to catch up
    } else if (dist_to_nominal > MAX_LAG_DIST) {
        // Linear reduction from 1.0→0.0 as dist goes MAX_LAG_DIST→PAUSE_LAG_DIST
        time_scale = 1.0 - (dist_to_nominal - MAX_LAG_DIST) /
                           (PAUSE_LAG_DIST - MAX_LAG_DIST);
    }

    current_t_ += dt * time_scale;
    current_t_  = std::min(current_t_, traj_->duration());

    // ── Sample at lookahead ───────────────────────────────────────────
    double sample_t = std::min(current_t_ + lookahead_time_, traj_->duration());
    return traj_->sample(sample_t);
}

} // namespace fc2::traj
