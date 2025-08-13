#ifndef SPLINE2D_H
#define SPLINE2D_H

#include <vector>
#include <Eigen/Dense>
#include <tinysplinecxx.h>

namespace ctrl {

/**
 * Spline2D:
 * - builds cubic TinySpline from 2D waypoints (preprocessing corner repeats)
 * - samples it into N samples, computes curvature, arc-length
 * - computes curvature-based max velocity and runs forward/backward pass for longitudinal accel
 * - time-parameterizes and supports VelocityAtTime queries (world frame)
 */
class Spline2D {
 public:
  Spline2D() = default;

  // main initializer:
  // waypoints_xy: list of (x,y)
  // v_max: max straight-line speed (m/s)
  // a_max: max longitudinal accel (m/s^2)
  // a_lat_max: max lateral accel used for curvature cap (m/s^2)
  // t_start_s: global start time for trajectory
  // samples: number of discretization samples for profiling
  void Init(const std::vector<Eigen::Vector2d>& waypoints_xy, double v_max, double a_max,
            double a_lat_max, double t_start_s = 0.0, int samples = 300, bool log_data = false);

  Eigen::Vector2d PositionAtT(double t) const;
  Eigen::Vector2d VelocityAtT(double t) const;

  double GetTStart() const { return t_start_; }
  double GetTFinish() const { return t_finish_; }
  double TotalArcLength() const;
  double GetTotalTime() const;
  void logSplineData(const std::string& filename) const;

  void Print() const;

 private:
  struct Sample {
    double u;  // spline parameter 0..1
    Eigen::Vector2d pos;
    Eigen::Vector2d d1;  // derivative wrt u
    Eigen::Vector2d d2;  // second derivative wrt u
    double arc;          // cumulative arc length
    double kappa;        // curvature
    double v;            // assigned speed
    double t;            // global time
  };

  // helpers
  std::vector<Eigen::Vector2d> preprocessCorners(const std::vector<Eigen::Vector2d>& pts);
  int repeatsForTurnAngle(double turn_deg) const;

  tinyspline::BSpline spline_xy_;
  tinyspline::BSpline spline_d1_;
  tinyspline::BSpline spline_d2_;

  std::vector<Sample> samples_;
  int samples_n_{300};

  // limits & times
  double v_max_{0.8};
  double a_max_{0.5};
  double a_lat_max_{0.8};
  double t_start_{0.0};
  double t_finish_{0.0};

  // core steps
  void buildSpline(const std::vector<Eigen::Vector2d>& pts);
  void sampleSpline();
  void computeCurvatureAndArc();
  void computeVelocityProfile();
  double findUforTime(double t) const;
};

}  // namespace ctrl

#endif  // SPLINE2D_H
