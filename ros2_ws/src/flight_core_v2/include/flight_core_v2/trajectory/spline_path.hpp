#pragma once
#include <Eigen/Core>
#include <vector>
#include <stdexcept>

namespace fc2::traj {

/**
 * @brief Catmull-Rom spline with chord-length arc-length parameterization.
 *
 * Accepts a list of ENU waypoints (minimum 2) produced by A*.
 * Provides continuous position and unit-tangent queries by arc-length s ∈ [0, length()].
 *
 * Properties:
 *  - C1 continuous (smooth tangent at joints)
 *  - Passes through every supplied waypoint
 *  - Arc-length queries via pre-built LUT (built once on setWaypoints)
 */
class SplinePath {
public:
    /// Replace the current path with a new set of waypoints.
    /// Rebuilds the arc-length LUT immediately.
    /// @throws std::invalid_argument if |pts| < 2.
    void setWaypoints(const std::vector<Eigen::Vector3d>& pts);

    /// Query 3D position at arc-length s.
    Eigen::Vector3d position(double s) const;

    /// Query unit tangent vector at arc-length s.
    Eigen::Vector3d tangent(double s) const;

    /// Total arc length of the path (m).
    double length() const { return total_length_; }

    bool empty() const { return waypoints_.empty(); }

private:
    // ── Catmull-Rom evaluation ───────────────────────────────────
    // seg ∈ [0, N-1], t ∈ [0,1]
    Eigen::Vector3d crPos(int seg, double t) const;
    Eigen::Vector3d crTangent(int seg, double t) const;

    // ── Arc-length LUT ───────────────────────────────────────────
    // Maps arc-length s → (segment_index, local_t)
    struct LutEntry { double s; int seg; double t; };
    std::vector<LutEntry> lut_;   ///< sorted ascending by s
    void buildLut();
    void lookupS(double s, int& seg, double& t) const;

    std::vector<Eigen::Vector3d> waypoints_; ///< original A* waypoints
    // Phantom endpoints for Catmull-Rom (p[-1] and p[N])
    Eigen::Vector3d p_before_, p_after_;

    double total_length_ = 0.0;
    static constexpr int LUT_SAMPLES_PER_SEG = 100;
};

} // namespace fc2::traj
