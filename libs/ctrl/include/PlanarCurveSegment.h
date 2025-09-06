#pragma once

#include <Eigen/Dense>

namespace ctrl {

/**
 * @brief Represents a single segment of a smooth path using Cubic Hermite Spline interpolation.
 *
 * This class stores the boundary conditions (start/end positions and tangents)
 * and uses them to calculate the position, velocity, and curvature at any point 't'
 * (from 0 to 1) along the curve.
 */
class PlanarCurveSegment {
public:
    PlanarCurveSegment(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                         const Eigen::Vector2d& m0, const Eigen::Vector2d& m1);

    Eigen::Vector2d getPosition(double t) const;
    Eigen::Vector2d getVelocity(double t) const;
    Eigen::Vector2d getAcceleration(double t) const;
    double getCurvature(double t) const;
    double getLength(int intervals = 20) const;

private:
    Eigen::Vector2d p0_, p1_; // Start and end points
    Eigen::Vector2d m0_, m1_; // Start and end tangents (derivatives)
};

} // namespace ctrl
