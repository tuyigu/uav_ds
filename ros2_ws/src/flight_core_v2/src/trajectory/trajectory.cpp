#include "flight_core_v2/trajectory/trajectory.hpp"
#include <stdexcept>
#include <cmath>

namespace fc2::traj {

void Trajectory::build(const std::vector<Eigen::Vector3d>& waypoints,
                       double v_max, double a_max, double j_max,
                       double yaw_target)
{
    built_ = false;
    path_.setWaypoints(waypoints);
    profile_.plan(path_.length(), v_max, a_max, j_max);
    yaw_target_ = yaw_target;
    built_ = true;
}

TrajectoryState Trajectory::sample(double t) const
{
    TrajectoryState st;
    if (!built_) return st;

    t = std::clamp(t, 0.0, profile_.duration());

    double s_pos = profile_.position(t);     // arc-length position
    double v_mag = profile_.velocity(t);     // speed scalar
    double a_mag = profile_.acceleration(t); // accel scalar

    st.position     = path_.position(s_pos);
    Eigen::Vector3d tan = path_.tangent(s_pos);
    st.velocity     = tan * v_mag;
    st.acceleration = tan * a_mag; // tangential accel (safe approximation)
    st.yaw          = yaw_target_;
    st.yaw_rate     = 0.0;
    st.time         = t;
    st.valid        = true;
    return st;
}

} // namespace fc2::traj
