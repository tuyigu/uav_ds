#pragma once
#include <cmath>
#include <array>

namespace fc2::traj {

/**
 * @brief Jerk-limited (S-curve) 1-D velocity profile planner.
 *
 * Generates a time-optimal, jerk-limited velocity profile for a given
 * scalar distance from rest (v_start=0) to rest (v_end=0).
 *
 * Internal 7-phase structure:
 *   Phase 1  jerk+    accel rises  0 → a_max   (duration Tj)
 *   Phase 2  accel+   const accel  a_max        (duration Ta, may be 0)
 *   Phase 3  jerk-    accel drops  a_max → 0    (duration Tj)
 *   Phase 4  cruise   const vel    v_peak       (duration Tv, may be 0)
 *   Phase 5  jerk-    accel rises  0 → -a_max   (duration Tj)
 *   Phase 6  accel-   const decel  -a_max       (duration Ta, may be 0)
 *   Phase 7  jerk+    accel drops  -a_max → 0   (duration Tj)
 *
 * If the distance is too short to reach v_max or a_max, the profile
 * degenerates gracefully (no cruise phase / triangle accel profile).
 *
 * Usage:
 *   SCurvePlanner sc;
 *   sc.plan(distance, v_max, a_max, j_max);
 *   double pos = sc.position(t);
 *   double vel = sc.velocity(t);
 */
class SCurvePlanner {
public:
    /// Plan the profile.  Call before any sample query.
    void plan(double distance, double v_max, double a_max, double j_max);

    double position(double t)     const;
    double velocity(double t)     const;
    double acceleration(double t) const;

    double duration()  const { return T_total_; }
    double peakVel()   const { return v_peak_; }
    bool   isPlanned() const { return planned_; }

private:
    // Phase boundary times (cumulative)
    double t_[8] = {};      // t_[0]=0, t_[7]=T_total
    double T_total_  = 0.0;
    double v_peak_   = 0.0;
    double a_peak_   = 0.0; // actual peak accel (may be < a_max)
    double j_max_    = 0.0;
    bool   planned_  = false;

    // Integrate one phase analytically
    struct PhaseState { double s, v, a; }; // pos offset, vel, accel at phase start
    std::array<PhaseState, 7> phase_start_ = {};

    double samplePhase(int ph, double dt_into_phase) const;
    double velPhase   (int ph, double dt_into_phase) const;
    double accPhase   (int ph, double dt_into_phase) const;
    int    findPhase  (double t, double& dt) const;
};

} // namespace fc2::traj
