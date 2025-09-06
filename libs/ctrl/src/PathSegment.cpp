#include "PathSegment.h"
#include <cmath>

namespace ctrl {

Eigen::Vector2d PathSegment::getPosition(double s) const {
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
    double x = x_coeffs[5] * s5 + x_coeffs[4] * s4 + x_coeffs[3] * s3 + x_coeffs[2] * s2 + x_coeffs[1] * s + x_coeffs[0];
    double y = y_coeffs[5] * s5 + y_coeffs[4] * s4 + y_coeffs[3] * s3 + y_coeffs[2] * s2 + y_coeffs[1] * s + y_coeffs[0];
    return {x, y};
}

Eigen::Vector2d PathSegment::getVelocity(double s) const {
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double dx = 5 * x_coeffs[5] * s4 + 4 * x_coeffs[4] * s3 + 3 * x_coeffs[3] * s2 + 2 * x_coeffs[2] * s + x_coeffs[1];
    double dy = 5 * y_coeffs[5] * s4 + 4 * y_coeffs[4] * s3 + 3 * y_coeffs[3] * s2 + 2 * y_coeffs[2] * s + y_coeffs[1];
    return {dx, dy};
}

Eigen::Vector2d PathSegment::getAcceleration(double s) const {
    double s2 = s * s;
    double s3 = s2 * s;
    double ddx = 20 * x_coeffs[5] * s3 + 12 * x_coeffs[4] * s2 + 6 * x_coeffs[3] * s + 2 * x_coeffs[2];
    double ddy = 20 * y_coeffs[5] * s3 + 12 * y_coeffs[4] * s2 + 6 * y_coeffs[3] * s + 2 * y_coeffs[2];
    return {ddx, ddy};
}

double PathSegment::getCurvature(double s) const {
    Eigen::Vector2d vel = getVelocity(s);
    Eigen::Vector2d acc = getAcceleration(s);

    double vel_sq_norm = vel.squaredNorm();
    if (vel_sq_norm < 1e-8) {
        return 0.0; // Curvature is undefined if velocity is zero
    }

    double numerator = vel.x() * acc.y() - vel.y() * acc.x();
    double denominator = std::pow(vel_sq_norm, 1.5);

    return numerator / denominator;
}

} // namespace ctrl