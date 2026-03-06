#include "flight_core_v2/trajectory/scurve_planner.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace fc2::traj {

// ─────────────────────────────────────────────────────────────────────────────
// plan() — compute time-optimal jerk-limited velocity profile
//
// Strategy (v_start = v_end = 0):
//   Tj = a_max / j_max          (jerk phase duration at full a_max)
//   v_Tj = 0.5 * j_max * Tj^2  (velocity reached after one jerk phase)
//   d_accel(v_peak, Ta) = integral of accel profile from 0→v_peak
//
//   Case 1 (full profile): can reach v_max, Ta ≥ 0
//   Case 2 (no cruise):    v_peak < v_max but a_max reached, Ta = 0
//   Case 3 (triangle):     v_peak so small that a_max not reached
// ─────────────────────────────────────────────────────────────────────────────

static double d_accel_full(double v_peak, double a_max, double j_max)
{
    // Distance covered during a full acceleration from 0 to v_peak
    // (phases 1+2+3), with a_max reached.
    // t_j = a_max / j_max,  t_a = (v_peak - j_max*t_j^2) / a_max
    double t_j = a_max / j_max;
    double t_a = (v_peak - j_max * t_j * t_j) / a_max;
    if (t_a < 0.0) t_a = 0.0;
    return a_max * t_j * t_j + 1.5 * a_max * t_j * t_a + 0.5 * a_max * t_a * t_a;
}



void SCurvePlanner::plan(double distance, double v_max, double a_max, double j_max)
{
    planned_ = false;
    j_max_ = j_max;

    // Guard trivial case
    if (distance < 1e-6) {
        T_total_ = 0.0; v_peak_ = 0.0; a_peak_ = 0.0;
        for (auto& p : phase_start_) p = {0.0, 0.0, 0.0};
        t_[0] = t_[1] = t_[2] = t_[3] = t_[4] = t_[5] = t_[6] = t_[7] = 0.0;
        planned_ = true;
        return;
    }

    double Tj  = a_max / j_max;             // jerk phase duration (at full a)
    // vel at end of Phase 1 = 0.5 * j_max * Tj * Tj (unused directly, kept for reference)

    // ── Determine peak velocity and whether a_max is reached ─────────────
    double Ta  = 0.0; // constant accel phase duration
    double Tv  = 0.0; // cruise phase duration

    // Check if full v_max profile fits in distance
    double d_half = d_accel_full(v_max, a_max, j_max); // accel distance (= decel distance)

    if (2.0 * d_half <= distance) {
        // Case 1: full S-curve, can reach v_max
        v_peak_ = v_max;
        a_peak_ = a_max;
        Ta = (v_max - j_max * Tj * Tj) / a_max;
        Tv = (distance - 2.0 * d_half) / v_max;
    } else {
        // Can't reach v_max.  Find v_peak such that 2 * d_accel(v_peak) = distance
        // First check if a_max can be reached
        // d_accel with a_max, Ta=0 means v_peak = j_max * Tj^2
        double v_peak_no_ta = j_max * Tj * Tj; // max speed reachable without Ta phase
        double d_no_ta = d_accel_full(v_peak_no_ta, a_max, j_max);

        if (2.0 * d_no_ta <= distance) {
            // Case 2: a_max reached but no cruise
            // Solve: v_peak^2/a_max + Tj*v_peak - distance/2 = 0  (quadratic in v_peak)
            // Rearranged: v_peak^2 + a_max*Tj*v_peak - a_max*distance/2 = 0
            double disc = a_max * a_max * Tj * Tj + 2.0 * a_max * distance;
            v_peak_ = (-a_max * Tj + std::sqrt(disc)) * 0.5;
            v_peak_ = std::min(v_peak_, v_max);
            a_peak_ = a_max;
            Ta = (v_peak_ - j_max * Tj * Tj) / a_max;
            if (Ta < 0.0) { Ta = 0.0; }
            Tv = 0.0;
        } else {
            // Case 3: triangle profile — a_max NOT reached
            // 2 * j * (sqrt(v/j))^3 = distance  → v = (distance * sqrt(j) / 2)^(2/3)
            double base = distance * std::sqrt(j_max) / 2.0;
            v_peak_ = std::pow(base, 2.0 / 3.0);
            v_peak_ = std::min(v_peak_, v_max);
            Tj  = std::sqrt(v_peak_ / j_max);  // re-compute Tj for this case
            a_peak_ = j_max * Tj;              // actual peak acceleration
            Ta = 0.0; Tv = 0.0;
        }
    }

    // ── Build phase boundary times ────────────────────────────────────────
    // Total time = 2*(2*Tj + Ta) + Tv
    T_total_ = 4.0 * Tj + 2.0 * Ta + Tv;
    t_[0] = 0.0;
    t_[1] = Tj;
    t_[2] = Tj + Ta;
    t_[3] = 2.0 * Tj + Ta;
    t_[4] = 2.0 * Tj + Ta + Tv;  // end of cruise
    t_[5] = 3.0 * Tj + Ta + Tv;
    t_[6] = 3.0 * Tj + 2.0 * Ta + Tv;
    t_[7] = T_total_;

    // ── Compute phase start states by forward integration ─────────────────
    // All phases have constant jerk ±j or 0.
    // Jerk sequence: +j, 0, -j, 0, -j, 0, +j
    double jerks[7] = { j_max, 0.0, -j_max, 0.0, -j_max, 0.0, j_max };

    phase_start_[0] = {0.0, 0.0, 0.0};
    for (int ph = 0; ph < 6; ++ph) {
        double dt = t_[ph+1] - t_[ph];
        auto& ps = phase_start_[ph];
        double j  = jerks[ph];
        phase_start_[ph+1].s = ps.s + ps.v * dt + 0.5 * ps.a * dt*dt + (1.0/6.0) * j * dt*dt*dt;
        phase_start_[ph+1].v = ps.v + ps.a * dt + 0.5 * j * dt*dt;
        phase_start_[ph+1].a = ps.a + j * dt;
    }

    planned_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// findPhase — which phase is t in?
// ─────────────────────────────────────────────────────────────────────────────

int SCurvePlanner::findPhase(double t, double& dt) const
{
    t = std::clamp(t, 0.0, T_total_);
    for (int ph = 0; ph < 7; ++ph) {
        if (t <= t_[ph + 1] + 1e-12) {
            dt = t - t_[ph];
            return ph;
        }
    }
    dt = t_[7] - t_[6];
    return 6;
}

// Jerk values per phase
static const double JERKS[7] = {
    1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 1.0
};

// ─────────────────────────────────────────────────────────────────────────────
// Sample functions
// ─────────────────────────────────────────────────────────────────────────────

double SCurvePlanner::position(double t) const
{
    if (!planned_ || T_total_ < 1e-9) return 0.0;
    if (t >= T_total_) return phase_start_[6].s +
        phase_start_[6].v * (T_total_ - t_[6]) +
        0.5 * phase_start_[6].a * (T_total_ - t_[6]) * (T_total_ - t_[6]) +
        (1.0/6.0) * (JERKS[6] * j_max_) * std::pow(T_total_ - t_[6], 3.0);

    double dt;
    int ph = findPhase(t, dt);
    double j = JERKS[ph] * j_max_;
    auto& ps = phase_start_[ph];
    return ps.s + ps.v * dt + 0.5 * ps.a * dt*dt + (1.0/6.0) * j * dt*dt*dt;
}

double SCurvePlanner::velocity(double t) const
{
    if (!planned_ || T_total_ < 1e-9) return 0.0;
    if (t >= T_total_) return 0.0;
    double dt;
    int ph = findPhase(t, dt);
    double j = JERKS[ph] * j_max_;
    auto& ps = phase_start_[ph];
    return ps.v + ps.a * dt + 0.5 * j * dt*dt;
}

double SCurvePlanner::acceleration(double t) const
{
    if (!planned_ || T_total_ < 1e-9) return 0.0;
    if (t >= T_total_) return 0.0;
    double dt;
    int ph = findPhase(t, dt);
    double j = JERKS[ph] * j_max_;
    auto& ps = phase_start_[ph];
    return ps.a + j * dt;
}

} // namespace fc2::traj
