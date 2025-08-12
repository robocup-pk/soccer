#include "Spline2D.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace ctrl {

using tsreal = tinyspline::real;

void Spline2D::Init(const std::vector<Eigen::Vector2d>& waypoints_xy,
                    double v_max, double a_max, double a_lat_max,
                    double t_start_s, int samples) {
  if (waypoints_xy.size() < 2) throw std::runtime_error("Spline2D: need >=2 pts");

  v_max_ = v_max;
  a_max_ = a_max;
  a_lat_max_ = a_lat_max;
  t_start_ = t_start_s;
  samples_n_ = std::max(50, samples);

  auto pts = preprocessCorners(waypoints_xy);
  buildSpline(pts);
  sampleSpline();
  computeCurvatureAndArc();
  computeVelocityProfile();
  t_finish_ = samples_.back().t;
}

std::vector<Eigen::Vector2d> Spline2D::preprocessCorners(const std::vector<Eigen::Vector2d>& pts) {
  // repeat points at sharp corners. Compute turn angle between (p_{i-1}->p_i) and (p_i->p_{i+1})
  // turn angle = acos( clamp( dot(u,v) ) ). We will map turn angle (deg) -> repeats.
  if (pts.size() < 3) return pts;
  std::vector<Eigen::Vector2d> out;
  out.reserve(pts.size() * 3);

  for (size_t i = 0; i < pts.size(); ++i) {
    const Eigen::Vector2d& p_prev = (i == 0) ? pts[0] : pts[i - 1];
    const Eigen::Vector2d& p_curr = pts[i];
    const Eigen::Vector2d& p_next = (i + 1 == pts.size()) ? pts.back() : pts[i + 1];

    Eigen::Vector2d v1 = (p_curr - p_prev);
    Eigen::Vector2d v2 = (p_next - p_curr);
    if (v1.norm() < 1e-9 || v2.norm() < 1e-9) {
      out.push_back(p_curr);
      continue;
    }
    v1.normalize();
    v2.normalize();
    double dot = std::clamp(v1.dot(v2), -1.0, 1.0);
    double turn_rad = std::acos(dot); // this is the angle between directions (turn angle)
    double turn_deg = turn_rad * 180.0 / M_PI;

    int reps = repeatsForTurnAngle(turn_deg);
    for (int r = 0; r < reps; ++r) out.push_back(p_curr);
  }
  return out;
}

int Spline2D::repeatsForTurnAngle(double turn_deg) const {
  // mapping (turn angle in degrees):
  // >= 120° -> 4 repeats
  // >= 90°  -> 3 repeats
  // >= 60°  -> 2 repeats
  // else    -> 1
  if (turn_deg >= 120.0) return 4;
  if (turn_deg >= 90.0) return 3;
  if (turn_deg >= 60.0) return 2;
  return 1;
}

void Spline2D::buildSpline(const std::vector<Eigen::Vector2d>& pts) {
  int ctrl_count = static_cast<int>(pts.size());
  spline_xy_ = tinyspline::BSpline(ctrl_count, 2, 3); // cubic clamped default
  std::vector<tsreal> ctrlp;
  ctrlp.reserve(ctrl_count * 2);
  for (const auto &p : pts) {
    ctrlp.push_back(static_cast<tsreal>(p.x()));
    ctrlp.push_back(static_cast<tsreal>(p.y()));
  }
  spline_xy_.setControlPoints(ctrlp);
  spline_d1_ = spline_xy_.derive();
  spline_d2_ = spline_d1_.derive();
}

void Spline2D::sampleSpline() {
  samples_.clear();
  samples_.resize(samples_n_);
  for (int i = 0; i < samples_n_; ++i) {
    double u = static_cast<double>(i) / (samples_n_ - 1);
    auto net = spline_xy_.eval(u);
    auto res = net.result();
    auto r1 = spline_d1_.eval(u).result();
    auto r2 = spline_d2_.eval(u).result();

    samples_[i].u = u;
    samples_[i].pos = Eigen::Vector2d(static_cast<double>(res[0]), static_cast<double>(res[1]));
    samples_[i].d1 = Eigen::Vector2d(static_cast<double>(r1[0]), static_cast<double>(r1[1]));
    samples_[i].d2 = Eigen::Vector2d(static_cast<double>(r2[0]), static_cast<double>(r2[1]));
    samples_[i].arc = 0.0;
    samples_[i].kappa = 0.0;
    samples_[i].v = v_max_;
    samples_[i].t = t_start_;
  }
}

void Spline2D::computeCurvatureAndArc() {
  double total = 0.0;
  for (int i = 0; i < samples_n_; ++i) {
    auto &s = samples_[i];
    double denom = std::pow(s.d1.norm(), 3);
    if (denom > 1e-9) {
      double num = s.d1.x() * s.d2.y() - s.d1.y() * s.d2.x();
      s.kappa = std::abs(num) / denom;
    } else {
      s.kappa = 0.0;
    }

    if (i > 0) {
      double du = samples_[i].u - samples_[i - 1].u;
      double avg_d1 = 0.5 * (samples_[i].d1.norm() + samples_[i - 1].d1.norm());
      double ds = avg_d1 * du;
      if (!(std::isfinite(ds) && ds >= 0.0)) ds = (samples_[i].pos - samples_[i - 1].pos).norm();
      total += ds;
      samples_[i].arc = total;
    } else {
      samples_[i].arc = 0.0;
    }
  }
}

void Spline2D::computeVelocityProfile() {
  const int N = samples_n_;
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
  samples_[0].v = 0.0; // start at rest (change if you want to start with non-zero)
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

  // 4) compute time stamps
  samples_[0].t = t_start_;
  for (int i = 1; i < N; ++i) {
    double ds = samples_[i].arc - samples_[i - 1].arc;
    double v_avg = 0.5 * (samples_[i].v + samples_[i - 1].v);
    double dt = (v_avg > 1e-9) ? (ds / v_avg) : 0.0;
    samples_[i].t = samples_[i - 1].t + dt;
  }
}

double Spline2D::findUforTime(double t) const {
  if (t <= samples_.front().t) return samples_.front().u;
  if (t >= samples_.back().t) return samples_.back().u;
  int lo = 0;
  int hi = samples_n_ - 1;
  while (hi - lo > 1) {
    int mid = (lo + hi) >> 1;
    if (samples_[mid].t <= t) lo = mid; else hi = mid;
  }
  double t0 = samples_[lo].t;
  double t1 = samples_[hi].t;
  double a = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
  return samples_[lo].u + a * (samples_[hi].u - samples_[lo].u);
}

Eigen::Vector2d Spline2D::PositionAtT(double t) const {
  double u = findUforTime(t);
  auto net = spline_xy_.eval(u);
  auto res = net.result();
  return Eigen::Vector2d(static_cast<double>(res[0]), static_cast<double>(res[1]));
}

Eigen::Vector2d Spline2D::VelocityAtT(double t) const {
  double u = findUforTime(t);
  auto d1 = spline_d1_.eval(u).result();
  Eigen::Vector2d d1v(static_cast<double>(d1[0]), static_cast<double>(d1[1]));

  // find local v via interpolation in sample times
  int idx = 0;
  while (idx < samples_n_ - 1 && samples_[idx + 1].t < t) ++idx;
  int idx1 = std::min(idx + 1, samples_n_ - 1);
  double t0 = samples_[idx].t, t1 = samples_[idx1].t;
  double v0 = samples_[idx].v, v1 = samples_[idx1].v;
  double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
  double v_local = v0 + alpha * (v1 - v0);

  double norm_d1 = d1v.norm();
  double dudt = (norm_d1 > 1e-9) ? (v_local / norm_d1) : 0.0;
  Eigen::Vector2d vel = d1v * dudt;
  return vel;
}


void Spline2D::logSplineData(const std::string& filename) const {
  std::ofstream logFile(filename);
  if (!logFile.is_open()) {
    std::cerr << "Error: Could not open log file " << filename << std::endl;
    return;
  }

  logFile << std::fixed << std::setprecision(8);

  // Write header with spline parameters
  logFile << "# Spline2D Log File\n";
  logFile << "# Parameters:\n";
  logFile << "# v_max: " << v_max_ << " m/s\n";
  logFile << "# a_max: " << a_max_ << " m/s²\n";
  logFile << "# a_lat_max: " << a_lat_max_ << " m/s²\n";
  logFile << "# t_start: " << t_start_ << " s\n";
  logFile << "# t_finish: " << t_finish_ << " s\n";
  logFile << "# samples: " << samples_n_ << "\n";
  logFile << "# total_arc_length: " << TotalArcLength() << " m\n";
  logFile << "# total_time: " << GetTotalTime() << " s\n";
  logFile << "#\n";
  logFile << "# Data Format: index u pos_x pos_y d1_x d1_y d2_x d2_y arc_length curvature velocity time\n";

  // Write sample data
  for (int i = 0; i < samples_n_; ++i) {
    const auto& s = samples_[i];
    logFile << i << " "
            << s.u << " "
            << s.pos.x() << " " << s.pos.y() << " "
            << s.d1.x() << " " << s.d1.y() << " "
            << s.d2.x() << " " << s.d2.y() << " "
            << s.arc << " "
            << s.kappa << " "
            << s.v << " "
            << s.t << "\n";
  }

  logFile.close();
  std::cout << "Spline data logged to: " << filename << std::endl;
}

double Spline2D::TotalArcLength() const {
  if (samples_.empty()) return 0.0;
  return samples_.back().arc;
}

double Spline2D::GetTotalTime() const {
  return t_finish_ - t_start_;
}

void Spline2D::Print() const {
  std::cout << "[Spline2D] samples: " << samples_n_
            << " duration: " << (t_finish_ - t_start_)
            << "s, v_max: " << v_max_ << ", a_max: " << a_max_
            << ", a_lat_max: " << a_lat_max_ << "\n";
}

} // namespace ctrl
