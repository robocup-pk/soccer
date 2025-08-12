#include "SplineTrajectory3D.h"

namespace ctrl {

using tsreal = tinyspline::real;

SplineTrajectory3D::SplineTrajectory3D(const std::vector<Eigen::Vector3d>& path_fWorld,
                                       double v_max,
                                       double a_max,
                                       double a_lat_max,
                                       double t_start_s,
                                       int update_hz,
                                       int samples)
    : v_max_(v_max),
      a_max_(a_max),
      a_lat_max_(a_lat_max),
      t_start_(t_start_s),
      t_finish_(t_start_s),
      samples_count_(std::max(50, samples)) // min 50
{
    if (path_fWorld.size() < 2) {
        throw std::runtime_error("SplineTrajectory3D: need at least 2 waypoints");
    }

    // Extract xy waypoints and theta endpoints
    std::vector<Eigen::Vector2d> xy_waypoints;
    xy_waypoints.reserve(path_fWorld.size());
    for (const auto &p : path_fWorld) xy_waypoints.emplace_back(p.x(), p.y());
    theta_start_ = path_fWorld.front().z();
    theta_end_ = path_fWorld.back().z();
    pose_start_ = Eigen::Vector3d(xy_waypoints.front().x(), xy_waypoints.front().y(), theta_start_);
    pose_end_ = Eigen::Vector3d(xy_waypoints.back().x(), xy_waypoints.back().y(), theta_end_);

    // Build TinySpline spline and its derivatives
    buildSpline(xy_waypoints);

    // Sample the spline and compute curvature/arc-length
    sampleSpline();
    computeCurvatureAndArc();

    // Compute velocity profile (curvature caps + forward/backward accel limits)
    computeVelocityProfile();

    // compute t_finish_ from last sample's time
    t_finish_ = samples_.back().t;
}

void SplineTrajectory3D::buildSpline(const std::vector<Eigen::Vector2d>& xy_waypoints) {
    // Create cubic 2D spline
    int ctrl_count = static_cast<int>(xy_waypoints.size());
    spline_xy_ = tinyspline::BSpline(ctrl_count, 2, 3);

    // flatten control points into vector<tsreal>
    std::vector<tsreal> ctrlp;
    ctrlp.reserve(ctrl_count * 2);
    for (const auto &pt : xy_waypoints) {
        ctrlp.push_back(static_cast<tsreal>(pt.x()));
        ctrlp.push_back(static_cast<tsreal>(pt.y()));
    }
    spline_xy_.setControlPoints(ctrlp);

    // derivatives
    spline_d1_ = spline_xy_.derive();
    spline_d2_ = spline_d1_.derive();
}

void SplineTrajectory3D::sampleSpline() {
    samples_.clear();
    samples_.resize(samples_count_);

    for (int i = 0; i < samples_count_; ++i) {
        double u = static_cast<double>(i) / (samples_count_ - 1);
        // eval pos
        auto net = spline_xy_.eval(u);
        auto res = net.result(); // vector<tsreal>
        double x = static_cast<double>(res[0]);
        double y = static_cast<double>(res[1]);

        // eval derivatives wrt u
        auto d1 = spline_d1_.eval(u).result();
        auto d2 = spline_d2_.eval(u).result();
        Eigen::Vector2d du1(static_cast<double>(d1[0]), static_cast<double>(d1[1]));
        Eigen::Vector2d du2(static_cast<double>(d2[0]), static_cast<double>(d2[1]));

        samples_[i].u = u;
        samples_[i].pos = Eigen::Vector2d(x, y);
        samples_[i].d1 = du1;
        samples_[i].d2 = du2;
        samples_[i].arc = 0.0;
        samples_[i].kappa = 0.0;
        samples_[i].v = v_max_;
        samples_[i].t = t_start_;
    }
}

void SplineTrajectory3D::computeCurvatureAndArc() {
    // compute approximate arc length and curvature
    double total = 0.0;
    for (int i = 0; i < samples_count_; ++i) {
        // curvature (2D formula)
        const auto &s = samples_[i];
        double denom = std::pow(s.d1.norm(), 3);
        if (denom > 1e-9) {
            double num = s.d1.x() * s.d2.y() - s.d1.y() * s.d2.x();
            samples_[i].kappa = std::abs(num) / denom;
        } else {
            samples_[i].kappa = 0.0;
        }

        if (i > 0) {
            // approximate ds by |d1| * du
            double du = samples_[i].u - samples_[i - 1].u;
            // average |d1| between samples
            double speed_u = 0.5 * (samples_[i].d1.norm() + samples_[i - 1].d1.norm());
            double ds = speed_u * du;
            if (!(std::isfinite(ds) && ds >= 0.0)) ds = (samples_[i].pos - samples_[i - 1].pos).norm();
            total += ds;
            samples_[i].arc = total;
        } else {
            samples_[i].arc = 0.0;
        }
    }
    // note: samples_.back().arc is total arc length
}

void SplineTrajectory3D::computeVelocityProfile() {
    const int N = samples_count_;
    if (N < 2) return;

    // 1) curvature-based limits
    for (int i = 0; i < N; ++i) {
        double v_curv = v_max_;
        if (samples_[i].kappa > 1e-9) {
            double cand = std::sqrt(std::max(0.0, a_lat_max_ / samples_[i].kappa));
            v_curv = std::min(v_curv, cand);
        }
        samples_[i].v = std::min(v_max_, v_curv);
    }

    // 2) forward pass (acceleration)
    samples_[0].v = 0.0; // start from rest (you can change this to project current velocity)
    for (int i = 1; i < N; ++i) {
        double ds = samples_[i].arc - samples_[i - 1].arc;
        if (ds < 0) ds = (samples_[i].pos - samples_[i - 1].pos).norm();
        double v_prev = samples_[i - 1].v;
        double v_allowed = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * a_max_ * ds));
        samples_[i].v = std::min(samples_[i].v, v_allowed);
    }

    // 3) backward pass (deceleration)
    samples_[N - 1].v = 0.0; // end at rest
    for (int i = N - 2; i >= 0; --i) {
        double ds = samples_[i + 1].arc - samples_[i].arc;
        if (ds < 0) ds = (samples_[i + 1].pos - samples_[i].pos).norm();
        double v_next = samples_[i + 1].v;
        double v_allowed = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * a_max_ * ds));
        samples_[i].v = std::min(samples_[i].v, v_allowed);
    }

    // 4) compute time stamps along path
    samples_[0].t = t_start_;
    for (int i = 1; i < N; ++i) {
        double ds = samples_[i].arc - samples_[i - 1].arc;
        double v_avg = 0.5 * (samples_[i].v + samples_[i - 1].v);
        double dt = (v_avg > 1e-9) ? (ds / v_avg) : 0.0;
        samples_[i].t = samples_[i - 1].t + dt;
    }
    // ensure last time is set
    t_finish_ = samples_.back().t;
}

double SplineTrajectory3D::findUTfromTime(double t) const {
    // map global t to normalized u; find enclosing samples_[i].t
    if (t <= samples_.front().t) return samples_.front().u;
    if (t >= samples_.back().t) return samples_.back().u;

    // binary search over time
    int lo = 0;
    int hi = samples_count_ - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (samples_[mid].t <= t) lo = mid; else hi = mid;
    }
    double t0 = samples_[lo].t;
    double t1 = samples_[hi].t;
    double u0 = samples_[lo].u;
    double u1 = samples_[hi].u;
    double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
    return u0 + alpha * (u1 - u0);
}

Eigen::Vector3d SplineTrajectory3D::VelocityAtT(double t) {
    // returns world-frame velocity (vx, vy, omega)
    if (t <= t_start_) {
        // at start: zero velocity
        return Eigen::Vector3d(0, 0, 0);
    }
    if (t >= t_finish_) {
        return Eigen::Vector3d(0, 0, 0);
    }

    // find u for this time
    double u = findUTfromTime(t);

    // evaluate derivative wrt u (dC/du), then convert to dC/dt = dC/du * du/dt
    auto d1 = spline_d1_.eval(u).result(); // vector<tsreal>
    Eigen::Vector2d d1v(static_cast<double>(d1[0]), static_cast<double>(d1[1]));

    // find local v(s) by interpolation between neighbor samples (by time)
    // locate which sample index corresponds to t
    int idx = 0;
    int N = samples_count_;
    // quick linear search (N ~ 200-500 is cheap)
    while (idx < N - 1 && samples_[idx + 1].t < t) ++idx;
    int idx1 = std::min(idx + 1, N - 1);
    double t0 = samples_[idx].t, t1 = samples_[idx1].t;
    double v0 = samples_[idx].v, v1 = samples_[idx1].v;
    double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
    double v_local = v0 + alpha * (v1 - v0);

    // compute du/dt: speed along curve = |dC/du| * du/dt = v_local -> du/dt = v_local / |dC/du|
    double speed_du = d1v.norm();
    double dudt = (speed_du > 1e-9) ? (v_local / speed_du) : 0.0;

    Eigen::Vector2d vel_xy = d1v * dudt; // vx, vy in world frame

    // heading (simple linear interpolation)
    double theta;
    double omega = 0.0;
    double duration = t_finish_ - t_start_;
    if (duration > 1e-9) {
        double frac = (t - t_start_) / duration;
        theta = theta_start_ + frac * (theta_end_ - theta_start_);
        omega = (theta_end_ - theta_start_) / duration;
    } else {
        theta = theta_end_;
        omega = 0.0;
    }

    return Eigen::Vector3d(vel_xy.x(), vel_xy.y(), omega);
}

void SplineTrajectory3D::Print() {
    std::cout << "[SplineTrajectory3D] duration: " << (t_finish_ - t_start_)
              << " s, samples: " << samples_count_ << ", v_max: " << v_max_
              << ", a_max: " << a_max_ << ", a_lat_max: " << a_lat_max_ << "\n";
}

Eigen::Vector3d SplineTrajectory3D::TotalDistance() {
    Eigen::Vector2d start = samples_.front().pos;
    Eigen::Vector2d end = samples_.back().pos;
    return Eigen::Vector3d((end - start).norm(), 0, 0);
}

} // namespace ctrl
