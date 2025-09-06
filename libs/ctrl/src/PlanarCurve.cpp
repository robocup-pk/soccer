#include "PlanarCurve.h"
#include <stdexcept>

namespace ctrl {

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
        
        segments_.emplace_back(p0, p1, m0, m1);
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

} // namespace ctrl
