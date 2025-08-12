#ifndef SPLINE_TRAJECTORY_3D_H
#define SPLINE_TRAJECTORY_3D_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <tinysplinecxx.h>
#include <Eigen/Dense>

#include "Trajectory3D.h"

namespace ctrl {

/**
 * SplineTrajectory3D
 *
 * - Fits a cubic B-spline (2D: x,y) from input waypoints (Eigen::Vector2d)
 * - Samples the spline into N samples, computes arc-length(s) and curvature
 * - Computes curvature-based speed caps and runs forward/backward pass to enforce longitudinal accel limits
 * - Produces time table t[i] and velocity profile v[i]
 * - Heading (theta) is handled with simple linear interpolation over the same total duration
 *
 * NOTE: This intentionally keeps the heading simple for now (linear). Replace later with trapezoid if needed.
 */
class SplineTrajectory3D : public Trajectory3D {
public:
    // waypoints: vector of (x,y,theta). theta used only for start/end heading.
    // v_max: max linear speed (m/s)
    // a_max: max longitudinal accel (m/s^2)
    // a_lat_max: max lateral accel (m/s^2) used to compute curvature speed cap
    // update_hz: the controller update rate (default 20)
    SplineTrajectory3D(const std::vector<Eigen::Vector3d>& path_fWorld,
                       double v_max,
                       double a_max,
                       double a_lat_max,
                       double t_start_s = 0.0,
                       int update_hz = 20,
                       int samples = 300);

    // Trajectory3D interface
    Eigen::Vector3d VelocityAtT(double t) override; // returns (vx_world, vy_world, omega)
    void Print() override;
    Eigen::Vector3d TotalDistance() override;

    void SetTFinish(double t_finish_s) override { t_finish_ = t_finish_s; }
    double GetTStart() override { return t_start_; }
    double GetTFinish() override { return t_finish_; }
    Eigen::Vector3d GetPoseStart() override { return pose_start_; }
    Eigen::Vector3d GetPoseEnd() override { return pose_end_; }

private:
    // helper types
    struct Sample {
        double u;               // spline parameter in [0,1]
        Eigen::Vector2d pos;    // x,y
        Eigen::Vector2d d1;     // first derivative wrt u
        Eigen::Vector2d d2;     // second derivative wrt u
        double arc;             // cumulative arc-length up to this sample
        double kappa;           // curvature
        double v;               // assigned velocity at this sample
        double t;               // time stamp for this sample (global time)
    };

    // spline + derivative
    tinyspline::BSpline spline_xy_;
    tinyspline::BSpline spline_d1_;
    tinyspline::BSpline spline_d2_;

    // samples
    std::vector<Sample> samples_;
    int samples_count_;

    // input / limits
    double v_max_;
    double a_max_;
    double a_lat_max_;
    double t_start_;
    double t_finish_;

    // heading (theta) simple linear over same duration
    double theta_start_;
    double theta_end_;
    Eigen::Vector3d pose_start_;
    Eigen::Vector3d pose_end_;

    // utility
    void buildSpline(const std::vector<Eigen::Vector2d>& xy_waypoints);
    void sampleSpline();
    void computeCurvatureAndArc();
    void computeVelocityProfile();
    double findUTfromTime(double t) const;
    double safe_div(double a, double b) const { return (std::abs(b) < 1e-12) ? 0.0 : (a / b); }
};

} // namespace ctrl

#endif // SPLINE_TRAJECTORY_3D_H
