#ifndef HEADING1D_H
#define HEADING1D_H

#include <cmath>

namespace ctrl {

/**
 * Heading1D: trapezoidal angular profile for heading (theta)
 * - Construct with start/end angle and limits (omega_max, alpha_max)
 * - compute minimal feasible profile (triangular or trapezoidal)
 * - can then be stretched externally by SetTotalTime if needed to sync durations
 */
class Heading1D {
public:
  Heading1D() = default;
  Heading1D(double theta_start_rad, double theta_end_rad, double t_start_s = 0.0,
            double omega_max = 2.0, double alpha_max = 4.0);

  void Init(double theta_start_rad, double theta_end_rad, double t_start_s = 0.0,
            double omega_max = 2.0, double alpha_max = 4.0);

  // If you want to override total duration (stretching), call this BEFORE queries:
  void SetTotalTime(double total_time_s);

  double PositionAtT(double t) const;  // theta (rad)
  double VelocityAtT(double t) const;  // omega (rad/s)
  double GetTStart() const { return t_start_; }
  double GetTFinish() const { return t_start_ + total_time_; }
  void Print() const;

private:
  double theta0_{0.0};
  double theta1_{0.0};
  double t_start_{0.0};
  double total_time_{0.0};

  // internal trapezoid params (when minimal)
  double omega_max_{0.0};
  double alpha_max_{0.0};
  double t_acc_{0.0};
  double t_cruise_{0.0};
  double sign_{1.0};

  void computeMinimalProfile();
};

} // namespace ctrl

#endif // HEADING1D_H
