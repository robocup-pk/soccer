#include "PlanarCurveSegment.h"
#include <cmath>

namespace ctrl {

PlanarCurveSegment::PlanarCurveSegment(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                                     const Eigen::Vector2d& m0, const Eigen::Vector2d& m1)
    : p0_(p0), p1_(p1), m0_(m0), m1_(m1) {}

// Hermite basis functions
inline double h00(double t) { return 2 * t * t * t - 3 * t * t + 1; }
inline double h10(double t) { return t * t * t - 2 * t * t + t; }
inline double h01(double t) { return -2 * t * t * t + 3 * t * t; }
inline double h11(double t) { return t * t * t - t * t; }

// Derivatives of Hermite basis functions
inline double h00_d(double t) { return 6 * t * t - 6 * t; }
inline double h10_d(double t) { return 3 * t * t - 4 * t + 1; }
inline double h01_d(double t) { return -6 * t * t + 6 * t; }
inline double h11_d(double t) { return 3 * t * t - 2 * t; }

// Second derivatives of Hermite basis functions
inline double h00_dd(double t) { return 12 * t - 6; }
inline double h10_dd(double t) { return 6 * t - 4; }
inline double h01_dd(double t) { return -12 * t + 6; }
inline double h11_dd(double t) { return 6 * t - 2; }


Eigen::Vector2d PlanarCurveSegment::getPosition(double t) const {
    return h00(t) * p0_ + h10(t) * m0_ + h01(t) * p1_ + h11(t) * m1_;
}

Eigen::Vector2d PlanarCurveSegment::getVelocity(double t) const {
    return h00_d(t) * p0_ + h10_d(t) * m0_ + h01_d(t) * p1_ + h11_d(t) * m1_;
}

Eigen::Vector2d PlanarCurveSegment::getAcceleration(double t) const {
    return h00_dd(t) * p0_ + h10_dd(t) * m0_ + h01_dd(t) * p1_ + h11_dd(t) * m1_;
}

double PlanarCurveSegment::getCurvature(double t) const {
    const Eigen::Vector2d vel = getVelocity(t);
    const Eigen::Vector2d acc = getAcceleration(t);
    const double vel_sq_norm = vel.squaredNorm();

    if (vel_sq_norm < 1e-8) {
        return 0.0;
    }

    return (vel.x() * acc.y() - vel.y() * acc.x()) / std::pow(vel_sq_norm, 1.5);
}

double PlanarCurveSegment::getLength(int intervals) const {
    double length = 0.0;
    Eigen::Vector2d last_pos = getPosition(0);
    for (int i = 1; i <= intervals; ++i) {
        double t = static_cast<double>(i) / intervals;
        Eigen::Vector2d current_pos = getPosition(t);
        length += (current_pos - last_pos).norm();
        last_pos = current_pos;
    }
    return length;
}

} // namespace ctrl
