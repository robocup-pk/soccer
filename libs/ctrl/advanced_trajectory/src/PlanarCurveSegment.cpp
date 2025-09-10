#include "PlanarCurveSegment.h"
#include <cmath>

namespace ctrl {

// Corresponds to PlanarCurveSegment.java

PlanarCurveSegment::PlanarCurveSegment(EPlanarCurveSegmentType type, const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& acc, double startTime, double endTime)
    : type(type), pos(pos), vel(vel), acc(acc), startTime(startTime), endTime(endTime) {}

PlanarCurveSegment PlanarCurveSegment::fromPoint(const Eigen::Vector2d& pos, double tStart, double tEnd) {
    return PlanarCurveSegment(EPlanarCurveSegmentType::POINT, pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), tStart, tEnd);
}

PlanarCurveSegment PlanarCurveSegment::fromFirstOrder(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, double tStart, double tEnd) {
    return PlanarCurveSegment(EPlanarCurveSegmentType::FIRST_ORDER, pos, vel, Eigen::Vector2d::Zero(), tStart, tEnd);
}

PlanarCurveSegment PlanarCurveSegment::fromSecondOrder(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& acc, double tStart, double tEnd) {
    return PlanarCurveSegment(EPlanarCurveSegmentType::SECOND_ORDER, pos, vel, acc, tStart, tEnd);
}

void PlanarCurveSegment::setEndTime(double tEnd) {
    endTime = tEnd;
}

Eigen::Vector2d PlanarCurveSegment::getPosition(double t) const {
    switch (type) {
        case EPlanarCurveSegmentType::FIRST_ORDER:
            return pos + vel * t;
        case EPlanarCurveSegmentType::SECOND_ORDER:
            return pos + vel * t + acc * (0.5 * t * t);
        case EPlanarCurveSegmentType::POINT:
        default:
            return pos;
    }
}

Eigen::Vector2d PlanarCurveSegment::getVelocity(double t) const {
    if (type == EPlanarCurveSegmentType::SECOND_ORDER) {
        return vel + acc * t;
    }
    return vel;
}

double PlanarCurveSegment::getLength() const {
    // For simple segments, approximate length using straight line distance
    // This is a simplified implementation - could be enhanced with numerical integration
    switch (type) {
        case EPlanarCurveSegmentType::POINT:
            return 0.0;
        case EPlanarCurveSegmentType::FIRST_ORDER: {
            // Linear motion: length = velocity * time
            double duration = getDuration();
            return vel.norm() * duration;
        }
        case EPlanarCurveSegmentType::SECOND_ORDER: {
            // Quadratic motion: approximate with numerical integration
            double duration = getDuration();
            const int num_samples = 10;
            double length = 0.0;
            for (int i = 0; i < num_samples; ++i) {
                double t1 = (i * duration) / num_samples;
                double t2 = ((i + 1) * duration) / num_samples;
                Eigen::Vector2d p1 = getPosition(t1);
                Eigen::Vector2d p2 = getPosition(t2);
                length += (p2 - p1).norm();
            }
            return length;
        }
        default:
            return 0.0;
    }
}

double PlanarCurveSegment::getCurvature(double t) const {
    switch (type) {
        case EPlanarCurveSegmentType::POINT:
        case EPlanarCurveSegmentType::FIRST_ORDER:
            return 0.0; // No curvature for point or straight line
        case EPlanarCurveSegmentType::SECOND_ORDER: {
            // For quadratic motion: curvature = |v x a| / |v|^3
            Eigen::Vector2d velocity = getVelocity(t);
            double vel_magnitude = velocity.norm();
            if (vel_magnitude < 1e-9) return 0.0;
            
            // 2D cross product: v x a = v.x * a.y - v.y * a.x
            double cross_product = velocity.x() * acc.y() - velocity.y() * acc.x();
            return std::abs(cross_product) / (vel_magnitude * vel_magnitude * vel_magnitude);
        }
        default:
            return 0.0;
    }
}

std::pair<PlanarCurveSegment, PlanarCurveSegment> PlanarCurveSegment::split(double tSplit) const {
    if (tSplit >= endTime) {
        Eigen::Vector2d endPos = getPosition(getDuration());
        return {*this, PlanarCurveSegment::fromPoint(endPos, endTime, tSplit)};
    }

    double t = tSplit - startTime;
    PlanarCurveSegment first(type, pos, vel, acc, startTime, tSplit);
    
    switch (type) {
        case EPlanarCurveSegmentType::FIRST_ORDER: {
            Eigen::Vector2d posNow = pos + (vel * t);
            return {first, PlanarCurveSegment::fromFirstOrder(posNow, vel, tSplit, endTime)};
        }
        case EPlanarCurveSegmentType::SECOND_ORDER: {
            Eigen::Vector2d posNow = pos + (vel * t) + (acc * (0.5 * t * t));
            Eigen::Vector2d velNow = vel + (acc * t);
            return {first, PlanarCurveSegment::fromSecondOrder(posNow, velNow, acc, tSplit, endTime)};
        }
        default: // POINT
            return {first, PlanarCurveSegment::fromPoint(pos, tSplit, endTime)};
    }
}


double PlanarCurveSegment::getEndTime() const { return endTime; }
double PlanarCurveSegment::getStartTime() const { return startTime; }
double PlanarCurveSegment::getDuration() const { return endTime - startTime; }
EPlanarCurveSegmentType PlanarCurveSegment::getType() const { return type; }
const Eigen::Vector2d& PlanarCurveSegment::getPos() const { return pos; }
const Eigen::Vector2d& PlanarCurveSegment::getVel() const { return vel; }
const Eigen::Vector2d& PlanarCurveSegment::getAcc() const { return acc; }

} // namespace ctrl
