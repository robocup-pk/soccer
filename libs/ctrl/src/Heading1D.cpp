#include "Heading1D.h"
#include <iostream>

namespace ctrl {


void Heading1D::Init(double theta_start_rad, double theta_end_rad, double t_start_s,
                     double omega_max, double alpha_max, double available_time_s) {
  theta0_ = theta_start_rad;
  theta1_ = theta_end_rad;
  t_start_ = t_start_s;
  omega_max_ = std::abs(omega_max);
  alpha_max_ = std::abs(alpha_max);
  sign_ = (theta1_ >= theta0_) ? 1.0 : -1.0;
  computeMinimalProfile();
  // If available time is provided, adjust the profile to fit within it
  if (available_time_s > 0) {
    MakeFeasible(available_time_s);
  }
}

void Heading1D::computeMinimalProfile() {
  double dtheta = std::abs(theta1_ - theta0_);
  if (dtheta < 1e-9) {
    t_acc_ = 0.0;
    t_cruise_ = 0.0;
    total_time_ = 0.0;
    return;
  }

  // time to reach omega_max
  double t_to_max = omega_max_ / alpha_max_;
  double theta_acc = 0.5 * alpha_max_ * t_to_max * t_to_max;

  if (2.0 * theta_acc >= dtheta) {
    // triangular profile
    t_acc_ = std::sqrt(dtheta / alpha_max_);
    t_cruise_ = 0.0;
    total_time_ = 2.0 * t_acc_;
  } else {
    // trapezoid
    t_acc_ = t_to_max;
    t_cruise_ = (dtheta - 2.0 * theta_acc) / omega_max_;
    total_time_ = 2.0 * t_acc_ + t_cruise_;
  }
}

void Heading1D::SetTotalTime(double total_time_s) {
  // Stretch the profile uniformly: convert to simple linear ramp if requested time == 0
  if (total_time_s <= 0) return;
  if (total_time_s >= total_time_) {
    // Keep the same shape but scale times proportionally: simple approach
    double scale = total_time_s / std::max(1e-12, total_time_);
    t_acc_ *= scale;
    t_cruise_ *= scale;
    total_time_ = total_time_s;
    // recompute omega_max_ and alpha_max_ roughly consistent with scaled times
    // (simple and conservative): reduce omega_max_ proportionally
    omega_max_ = omega_max_ / scale;
    alpha_max_ = alpha_max_ / (scale * scale);  // naive but safe-ish
  } else {
    // if new total is smaller than minimal feasible, ignore (do nothing)
  }
}

void Heading1D::MakeFeasible(double available_time_s) {
  if (available_time_s <= 0) return;

  computeMinimalProfile();
  if (total_time_ <= available_time_s) {
    // Already feasible, just stretch to exactly available_time_s
    SetTotalTime(available_time_s);
    return;
  }

  // Too short: reduce omega_max_ first
  double scale = available_time_s / total_time_;
  omega_max_ *= scale;
  computeMinimalProfile();

  if (total_time_ > available_time_s) {
    // Still too long: reduce alpha_max_ proportionally as well
    double scale_alpha = available_time_s / total_time_;
    alpha_max_ *= scale_alpha;
    computeMinimalProfile();
  }

  // If still too long after both reductions, cap at minimal achievable
  if (total_time_ > available_time_s) {
    SetTotalTime(available_time_s);
  }
}

double Heading1D::PositionAtT(double t) const {
  if (t <= t_start_) return theta0_;
  double rel = t - t_start_;
  if (rel >= total_time_) return theta1_;

  double theta = 0.0;
  if (t_cruise_ <= 0.0) {
    // triangular
    if (rel <= t_acc_) {
      theta = 0.5 * alpha_max_ * rel * rel;
    } else {
      double d = rel - t_acc_;
      double theta_acc = 0.5 * alpha_max_ * t_acc_ * t_acc_;
      double omega_peak = alpha_max_ * t_acc_;
      theta = theta_acc + omega_peak * d - 0.5 * alpha_max_ * d * d;
    }
  } else {
    if (rel <= t_acc_) {
      theta = 0.5 * alpha_max_ * rel * rel;
    } else if (rel <= (t_acc_ + t_cruise_)) {
      double theta_acc = 0.5 * alpha_max_ * t_acc_ * t_acc_;
      theta = theta_acc + omega_max_ * (rel - t_acc_);
    } else {
      double theta_acc = 0.5 * alpha_max_ * t_acc_ * t_acc_;
      double theta_cruise = omega_max_ * t_cruise_;
      double d = rel - (t_acc_ + t_cruise_);
      theta = theta_acc + theta_cruise + omega_max_ * d - 0.5 * alpha_max_ * d * d;
    }
  }
  return theta0_ + sign_ * theta;
}

double Heading1D::VelocityAtT(double t) const {
  if (t <= t_start_) return 0.0;
  double rel = t - t_start_;
  if (rel >= total_time_) return 0.0;

  if (t_cruise_ <= 0.0) {
    if (rel <= t_acc_) return sign_ * alpha_max_ * rel;
    double d = rel - t_acc_;
    return sign_ * (alpha_max_ * t_acc_ - alpha_max_ * d);
  } else {
    if (rel <= t_acc_) return sign_ * alpha_max_ * rel;
    if (rel <= (t_acc_ + t_cruise_)) return sign_ * omega_max_;
    double d = rel - (t_acc_ + t_cruise_);
    return sign_ * std::max(0.0, omega_max_ - alpha_max_ * d);
  }
}

void Heading1D::Print() const {
  std::cout << "[Heading1D] theta: " << theta0_ << " -> " << theta1_ << ", t_start: " << t_start_
            << ", total_time: " << total_time_ << "\n";
}

}  // namespace ctrl
