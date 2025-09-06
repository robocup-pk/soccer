#pragma once

#include <Eigen/Dense>

namespace ctrl {

/**
 * @brief Represents a single, continuous segment of a robot's path.
 * * This class stores the coefficients of a quintic Hermite spline for both the
 * x and y coordinates, parameterized by a variable 's' from 0 to 1.
 * It provides methods to evaluate the position, velocity, acceleration, and curvature
 * at any point along the segment.
 */
class PathSegment {
public:
    // Coefficients for the quintic polynomial: p(s) = c5*s^5 + c4*s^4 + c3*s^3 + c2*s^2 + c1*s + c0
    Eigen::Matrix<double, 6, 1> x_coeffs;
    Eigen::Matrix<double, 6, 1> y_coeffs;

    double arc_length; // The total length of this path segment

    PathSegment() : arc_length(0.0) {
        x_coeffs.setZero();
        y_coeffs.setZero();
    }

    /**
     * @brief Evaluates the position on the path at parameter s.
     * @param s The parameter, ranging from 0 (start of segment) to 1 (end of segment).
     * @return The (x, y) position.
     */
    Eigen::Vector2d getPosition(double s) const;

    /**
     * @brief Evaluates the first derivative (tangent vector) on the path at parameter s.
     * The magnitude of this vector is related to the "speed" of the parameterization.
     * @param s The parameter, ranging from 0 to 1.
     * @return The (dx/ds, dy/ds) vector.
     */
    Eigen::Vector2d getVelocity(double s) const;

    /**
     * @brief Evaluates the second derivative (acceleration vector) on the path at parameter s.
     * @param s The parameter, ranging from 0 to 1.
     * @return The (d²x/ds², d²y/ds²) vector.
     */
    Eigen::Vector2d getAcceleration(double s) const;

    /**
     * @brief Calculates the curvature of the path at parameter s.
     * @param s The parameter, ranging from 0 to 1.
     * @return The curvature (kappa).
     */
    double getCurvature(double s) const;
};

} // namespace ctrl