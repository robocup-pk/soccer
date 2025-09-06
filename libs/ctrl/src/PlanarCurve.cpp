#include "PlanarCurve.h"
#include <stdexcept>

namespace ctrl {

PlanarCurve::PlanarCurve(const std::vector<PlanarCurveSegment>& segments) 
    : segments_(segments) {
    parameterizeByArcLength();
}

PlanarCurve::PlanarCurve(const std::vector<Eigen::Vector2d>& waypoints) {
    build(waypoints);
}

void PlanarCurve::build(const std::vector<Eigen::Vector2d>& waypoints) {
    if (waypoints.size() < 2) {
        return;
    }

    segments_.clear();
    segments_.reserve(waypoints.size() - 1);

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Eigen::Vector2d& p0 = waypoints[i];
        const Eigen::Vector2d& p1 = waypoints[i+1];

        Eigen::Vector2d m0, m1;

        // Calculate tangent for the start point (m0)
        if (i == 0) {
            m0 = (p1 - p0);
        } else {
            // Catmull-Rom tangent for smooth transitions
            m0 = (p1 - waypoints[i-1]) * 0.5;
        }

        // Calculate tangent for the end point (m1)
        if (i == waypoints.size() - 2) {
            m1 = (p1 - p0);
        } else {
            m1 = (waypoints[i+2] - p0) * 0.5;
        }
        
        // Create a SECOND_ORDER segment (Hermite spline with tangents as accelerations)
        // Convert tangents to accelerations by scaling
        double segment_time = 1.0; // Normalized time per segment
        Eigen::Vector2d vel0 = m0 / segment_time;
        Eigen::Vector2d vel1 = m1 / segment_time;
        Eigen::Vector2d acc = (vel1 - vel0) / segment_time; // Constant acceleration approximation
        
        double tStart = i * segment_time;
        double tEnd = (i + 1) * segment_time;
        
        segments_.push_back(PlanarCurveSegment::fromSecondOrder(p0, vel0, acc, tStart, tEnd));
    }
    
    parameterizeByArcLength();
}

void PlanarCurve::parameterizeByArcLength() {
    total_length_ = 0.0;
    segment_start_arclengths_.clear();
    segment_start_arclengths_.push_back(0.0);

    for (const auto& segment : segments_) {
        total_length_ += segment.getLength();
        segment_start_arclengths_.push_back(total_length_);
    }
}

std::pair<int, double> PlanarCurve::findSegmentForArcLength(double s) const {
    s = std::clamp(s, 0.0, total_length_);
    
    auto it = std::upper_bound(segment_start_arclengths_.begin(), segment_start_arclengths_.end(), s);
    int segment_idx = static_cast<int>(std::distance(segment_start_arclengths_.begin(), it) - 1);
    segment_idx = std::max(0, std::min(segment_idx, static_cast<int>(segments_.size() - 1)));
    
    double length_into_segment = s - segment_start_arclengths_[segment_idx];
    double segment_length = segment_start_arclengths_[segment_idx + 1] - segment_start_arclengths_[segment_idx];

    double t = (segment_length > 1e-6) ? (length_into_segment / segment_length) : 0.0;
    return {segment_idx, t};
}

Eigen::Vector2d PlanarCurve::getPositionAt(double s) const {
    if (!isValid()) return Eigen::Vector2d::Zero();
    auto [segment_idx, t] = findSegmentForArcLength(s);
    return segments_[segment_idx].getPosition(t);
}

Eigen::Vector2d PlanarCurve::getTangentAt(double s) const {
    if (!isValid()) return Eigen::Vector2d::Zero();
    auto [segment_idx, t] = findSegmentForArcLength(s);
    return segments_[segment_idx].getVelocity(t).normalized();
}

double PlanarCurve::getCurvatureAt(double s) const {
    if (!isValid()) return 0.0;
    auto [segment_idx, t] = findSegmentForArcLength(s);
    return segments_[segment_idx].getCurvature(t);
}

bool PlanarCurve::isValid() const {
    return !segments_.empty();
}

const std::vector<PlanarCurveSegment>& PlanarCurve::getSegments() const {
    return segments_;
}

double PlanarCurve::getTEnd() const {
    if (segments_.empty()) return 0.0;
    return segments_.back().getEndTime();
}

double PlanarCurve::getTStart() const {
    if (segments_.empty()) return 0.0;
    return segments_.front().getStartTime();
}

PlanarCurveState PlanarCurve::getState(double t) const {
    // Find the segment that contains time t
    for (const auto& segment : segments_) {
        if (t >= segment.getStartTime() && t <= segment.getEndTime()) {
            double local_t = t - segment.getStartTime();
            Eigen::Vector2d pos = segment.getPosition(local_t);
            Eigen::Vector2d vel = segment.getVelocity(local_t);
            Eigen::Vector2d acc = segment.getAcc(); // Constant acceleration for segments
            return PlanarCurveState(pos, vel, acc);
        }
    }
    
    // If not found, return zero state
    return PlanarCurveState(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
}

Eigen::Vector2d PlanarCurve::getPos(double t) const {
    return getState(t).pos;
}

Eigen::Vector2d PlanarCurve::getVel(double t) const {
    return getState(t).vel;
}

Eigen::Vector2d PlanarCurve::getAcc(double t) const {
    return getState(t).acc;
}

PlanarCurve PlanarCurve::fromPoint(const Eigen::Vector2d& point) {
    std::vector<PlanarCurveSegment> segments;
    segments.push_back(PlanarCurveSegment::fromPoint(point, 0.0, 1.0));
    return PlanarCurve(segments);
}

} // namespace ctrl
