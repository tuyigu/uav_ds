#include "flight_core_v2/trajectory/spline_path.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace fc2::traj {

// ─────────────────────────────────────────────────────────────────────────────
// Catmull-Rom helpers
// Segment i uses control points: pts_[i-1], pts_[i], pts_[i+1], pts_[i+2]
// (with phantom endpoints at -1 and N)
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d SplinePath::crPos(int seg, double t) const
{
    // p_{seg-1}, p_{seg}, p_{seg+1}, p_{seg+2}
    // With phantom caps: segment spans waypoints_[seg] → waypoints_[seg+1]
    const int N = static_cast<int>(waypoints_.size());
    auto P = [&](int i) -> const Eigen::Vector3d& {
        if (i < 0)   return p_before_;
        if (i >= N)  return p_after_;
        return waypoints_[i];
    };

    const Eigen::Vector3d& p0 = P(seg - 1);
    const Eigen::Vector3d& p1 = P(seg);
    const Eigen::Vector3d& p2 = P(seg + 1);
    const Eigen::Vector3d& p3 = P(seg + 2);

    double t2 = t * t;
    double t3 = t2 * t;

    return 0.5 * (
        2.0 * p1 +
        (-p0 + p2)          * t +
        (2.0*p0 - 5.0*p1 + 4.0*p2 - p3) * t2 +
        (-p0 + 3.0*p1 - 3.0*p2 + p3)    * t3
    );
}

Eigen::Vector3d SplinePath::crTangent(int seg, double t) const
{
    const int N = static_cast<int>(waypoints_.size());
    auto P = [&](int i) -> const Eigen::Vector3d& {
        if (i < 0)   return p_before_;
        if (i >= N)  return p_after_;
        return waypoints_[i];
    };

    const Eigen::Vector3d& p0 = P(seg - 1);
    const Eigen::Vector3d& p1 = P(seg);
    const Eigen::Vector3d& p2 = P(seg + 1);
    const Eigen::Vector3d& p3 = P(seg + 2);

    double t2 = t * t;

    // d/dt of crPos
    return 0.5 * (
        (-p0 + p2) +
        2.0 * (2.0*p0 - 5.0*p1 + 4.0*p2 - p3)  * t +
        3.0 * (-p0 + 3.0*p1 - 3.0*p2 + p3)       * t2
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// setWaypoints — build phantom endpoints + arc-length LUT
// ─────────────────────────────────────────────────────────────────────────────

void SplinePath::setWaypoints(const std::vector<Eigen::Vector3d>& pts)
{
    if (pts.size() < 2)
        throw std::invalid_argument("SplinePath requires at least 2 waypoints");

    waypoints_ = pts;

    // Phantom endpoints: "reflection" of the second/second-to-last points
    p_before_ = 2.0 * pts.front() - pts[1];
    p_after_  = 2.0 * pts.back()  - pts[pts.size() - 2];

    buildLut();
}

void SplinePath::buildLut()
{
    lut_.clear();
    total_length_ = 0.0;

    const int num_segs = static_cast<int>(waypoints_.size()) - 1;

    for (int seg = 0; seg < num_segs; ++seg) {
        Eigen::Vector3d prev = crPos(seg, 0.0);

        for (int k = 1; k <= LUT_SAMPLES_PER_SEG; ++k) {
            double t   = static_cast<double>(k) / LUT_SAMPLES_PER_SEG;
            Eigen::Vector3d cur = crPos(seg, t);
            total_length_ += (cur - prev).norm();
            lut_.push_back({total_length_, seg, t});
            prev = cur;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// lookupS — binary search into LUT
// ─────────────────────────────────────────────────────────────────────────────

void SplinePath::lookupS(double s, int& seg, double& t) const
{
    s = std::clamp(s, 0.0, total_length_);

    if (lut_.empty()) { seg = 0; t = 0.0; return; }

    // Binary search for the first entry with entry.s >= s
    auto it = std::lower_bound(lut_.begin(), lut_.end(), s,
        [](const LutEntry& e, double val){ return e.s < val; });

    if (it == lut_.end()) {
        const auto& last = lut_.back();
        seg = last.seg; t = last.t; return;
    }
    if (it == lut_.begin()) {
        seg = it->seg; t = it->t; return;
    }

    const auto& hi = *it;
    const auto& lo = *(it - 1);

    // Linear interpolation within the LUT interval
    double alpha = (hi.s - lo.s > 1e-9) ? (s - lo.s) / (hi.s - lo.s) : 0.0;

    // Interpolate local t within the same segment (or use hi directly)
    if (hi.seg == lo.seg) {
        seg = hi.seg;
        t   = lo.t + alpha * (hi.t - lo.t);
    } else {
        // Segment boundary — snap to hi
        seg = hi.seg; t = hi.t;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public query API
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d SplinePath::position(double s) const
{
    int seg; double t;
    lookupS(s, seg, t);
    return crPos(seg, t);
}

Eigen::Vector3d SplinePath::tangent(double s) const
{
    int seg; double t;
    lookupS(s, seg, t);
    Eigen::Vector3d tan = crTangent(seg, t);
    double n = tan.norm();
    if (n < 1e-9) return Eigen::Vector3d::UnitX(); // degenerate → forward
    return tan / n;
}

} // namespace fc2::traj
