#pragma once

#include "PlanarCurveSegment.h"
#include "PlanarCurveState.h"
#include <vector>
#include <Eigen/Dense>

namespace ctrl {

// Corresponds to PlanarCurve.java

class PlanarCurve {
private:
    std::vector<PlanarCurveSegment> segments_;
    std::vector<double> cumulativeArcLengths_;
    std::vector<double> segment_start_arclengths_;
    double total_length_ = 0.0;

public:
    // Constructors
    PlanarCurve(const std::vector<PlanarCurveSegment>& segments);
    PlanarCurve(const std::vector<Eigen::Vector2d>& waypoints);

    // Segment access
    const std::vector<PlanarCurveSegment>& getSegments() const;
    double getTEnd() const;
    double getTStart() const;

    // State queries (time-based)
    PlanarCurveState getState(double t) const;
    Eigen::Vector2d getPos(double t) const;
    Eigen::Vector2d getVel(double t) const;
    Eigen::Vector2d getAcc(double t) const;
    
    // Arc-length parameterization
    Eigen::Vector2d getPositionAt(double s) const;
    Eigen::Vector2d getTangentAt(double s) const;
    double getCurvatureAt(double s) const;
    
    // Factory methods
    static PlanarCurve fromPoint(const Eigen::Vector2d& point);
    
    // Validation
    bool isValid() const;

private:
    void build(const std::vector<Eigen::Vector2d>& waypoints);
    void parameterizeByArcLength();
    std::pair<int, double> findSegmentForArcLength(double s) const;
};

} // namespace ctrl
