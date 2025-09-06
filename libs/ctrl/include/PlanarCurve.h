#pragma once

#include "PlanarCurveSegment.h"
#include <vector>

namespace ctrl {

/**
 * @brief Represents a continuous, smooth path composed of multiple segments.
 *
 * This class takes a series of waypoints and creates a smooth, C1-continuous
 * spline curve that passes through them. It automatically calculates tangents
 * at each waypoint to ensure smooth transitions between segments. It also
 * parameterizes the entire curve by arc length, making it easy for a controller
 * to query points at a specific distance along the path.
 */
class PlanarCurve {
public:
    PlanarCurve() = default;

    /**
     * @brief Constructs a smooth curve from a list of waypoints.
     * @param waypoints A vector of 2D points from a path planner like RRTX.
     */
    PlanarCurve(const std::vector<Eigen::Vector2d>& waypoints);

    // --- Path Properties at a given Arc Length 's' ---
    Eigen::Vector2d getPositionAt(double s) const;
    Eigen::Vector2d getTangentAt(double s) const;
    double getCurvatureAt(double s) const;

    double getTotalLength() const { return total_length_; }
    bool isValid() const { return !segments_.empty(); }

private:
    std::pair<int, double> findSegmentForArcLength(double s) const;
    void build(const std::vector<Eigen::Vector2d>& waypoints);
    void parameterizeByArcLength();

    std::vector<PlanarCurveSegment> segments_;
    std::vector<double> segment_start_arclengths_;
    double total_length_ = 0.0;
};

} // namespace ctrl
