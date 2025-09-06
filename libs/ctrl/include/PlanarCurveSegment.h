#pragma once

#include "EPlanarCurveSegmentType.h"
#include <Eigen/Dense>
#include <utility>

namespace ctrl {

// Corresponds to PlanarCurveSegment.java

/**
 * @brief A planar curve segment that restricts a curve to a specific time frame.
 */
class PlanarCurveSegment {
private:
    EPlanarCurveSegmentType type;
    Eigen::Vector2d pos; // [mm]
    Eigen::Vector2d vel; // [mm/s]
    Eigen::Vector2d acc; // [mm/s^2]
    double startTime;    // [s]
    double endTime;      // [s]

    PlanarCurveSegment(EPlanarCurveSegmentType type, const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& acc, double startTime, double endTime);

public:
    static PlanarCurveSegment fromPoint(const Eigen::Vector2d& pos, double tStart, double tEnd);
    static PlanarCurveSegment fromFirstOrder(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, double tStart, double tEnd);
    static PlanarCurveSegment fromSecondOrder(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& acc, double tStart, double tEnd);

    void setEndTime(double tEnd);
    Eigen::Vector2d getPosition(double t) const;
    Eigen::Vector2d getVelocity(double t) const;
    double getLength() const;
    double getCurvature(double t) const;
    
    std::pair<PlanarCurveSegment, PlanarCurveSegment> split(double tSplit) const;
    
    // Getters
    double getEndTime() const;
    double getStartTime() const;
    double getDuration() const;
    EPlanarCurveSegmentType getType() const;
    const Eigen::Vector2d& getPos() const;
    const Eigen::Vector2d& getVel() const;
    const Eigen::Vector2d& getAcc() const;
};

} // namespace ctrl
