#include "PlanarCurveFactory.h"
#include "EPlanarCurveSegmentType.h"
#include <algorithm>
#include <stdexcept>

namespace ctrl {

PlanarCurve PlanarCurveFactory::fromBangBangTrajectory2D(const BangBangTrajectory2D& trajectory, int numSegments) {
    std::vector<double> timeSections = trajectory.getTimeSections();
    
    if (numSegments > 0) {
        // Create evenly spaced time sections if specific count requested
        timeSections.clear();
        double totalTime = trajectory.getTotalTime();
        for (int i = 0; i <= numSegments; ++i) {
            timeSections.push_back((i * totalTime) / numSegments);
        }
    }
    
    auto segments = createSegmentsFromTimeSections(trajectory, timeSections);
    return PlanarCurve(segments);
}

PlanarCurve PlanarCurveFactory::fromTrajectoryXyw(const TrajectoryXyw& trajectory, int numSegments) {
    std::vector<double> timeSections = trajectory.getTimeSections();
    
    if (numSegments > 0) {
        // Create evenly spaced time sections if specific count requested
        timeSections.clear();
        double totalTime = trajectory.getTotalTime();
        for (int i = 0; i <= numSegments; ++i) {
            timeSections.push_back((i * totalTime) / numSegments);
        }
    }
    
    auto segments = createSegmentsFromTimeSections(trajectory, timeSections);
    return PlanarCurve(segments);
}

template<typename TrajectoryType>
std::vector<PlanarCurveSegment> PlanarCurveFactory::createSegmentsFromTimeSections(
    const TrajectoryType& trajectory, 
    const std::vector<double>& timeSections) {
    
    std::vector<PlanarCurveSegment> segments;
    
    if (timeSections.size() < 2) {
        throw std::invalid_argument("Need at least 2 time sections to create segments");
    }
    
    for (size_t i = 0; i < timeSections.size() - 1; ++i) {
        double tStart = timeSections[i];
        double tEnd = timeSections[i + 1];
        
        segments.push_back(createSegmentFromTimeInterval(trajectory, tStart, tEnd));
    }
    
    return segments;
}

template<typename TrajectoryType>
PlanarCurveSegment PlanarCurveFactory::createSegmentFromTimeInterval(
    const TrajectoryType& trajectory,
    double tStart, 
    double tEnd) {
    
    // Get boundary conditions at start and end of interval
    Eigen::Vector2d p0 = extractPosition2D(trajectory, tStart);
    Eigen::Vector2d v0 = extractVelocity2D(trajectory, tStart);
    
    // For BangBang trajectories, use SECOND_ORDER segments (position + velocity + acceleration)
    // This matches the Advanced approach where segments can represent constant acceleration phases
    Eigen::Vector2d acc = Eigen::Vector2d::Zero();
    
    // Extract acceleration from trajectory if available
    // We know BangBang trajectories have constant acceleration
    if constexpr (std::is_same_v<TrajectoryType, TrajectoryXyw>) {
        // For TrajectoryXyw, getAcceleration returns Vector3d
        Eigen::Vector3d acc3d = trajectory.getAcceleration(tStart);
        acc = acc3d.segment<2>(0);  // Extract first 2 components (x, y)
    } else {
        // For BangBangTrajectory2D, getAcceleration returns Vector2d
        acc = trajectory.getAcceleration(tStart);
    }
    
    // Create SECOND_ORDER segment (matches Advanced's approach for BangBang trajectories)
    return PlanarCurveSegment::fromSecondOrder(p0, v0, acc, tStart, tEnd);
}

template<typename TrajectoryType>
int PlanarCurveFactory::determineOptimalSegmentCount(const TrajectoryType& trajectory) {
    double totalTime = trajectory.getTotalTime();
    std::vector<double> timeSections = trajectory.getTimeSections();
    
    // Use natural time sections from trajectory as base
    int naturalSegments = std::max(1, static_cast<int>(timeSections.size()) - 1);
    
    // For longer trajectories, increase segment count for smoother representation
    int timeBasedSegments = static_cast<int>(std::ceil(totalTime * 2.0)); // ~2 segments per second
    
    return std::max(naturalSegments, std::min(timeBasedSegments, 10)); // Cap at 10 segments
}

// Template specializations for extracting 2D data from different trajectory types

template<>
Eigen::Vector2d PlanarCurveFactory::extractPosition2D<BangBangTrajectory2D>(
    const BangBangTrajectory2D& trajectory, double t) {
    return trajectory.getPosition(t);
}

template<>
Eigen::Vector2d PlanarCurveFactory::extractVelocity2D<BangBangTrajectory2D>(
    const BangBangTrajectory2D& trajectory, double t) {
    return trajectory.getVelocity(t);
}

template<>
Eigen::Vector2d PlanarCurveFactory::extractPosition2D<TrajectoryXyw>(
    const TrajectoryXyw& trajectory, double t) {
    Eigen::Vector3d pos3d = trajectory.getPosition(t);
    return pos3d.head<2>();
}

template<>
Eigen::Vector2d PlanarCurveFactory::extractVelocity2D<TrajectoryXyw>(
    const TrajectoryXyw& trajectory, double t) {
    Eigen::Vector3d vel3d = trajectory.getVelocity(t);
    return vel3d.head<2>();
}

// Explicit template instantiations to ensure linking
template std::vector<PlanarCurveSegment> PlanarCurveFactory::createSegmentsFromTimeSections<BangBangTrajectory2D>(
    const BangBangTrajectory2D&, const std::vector<double>&);

template std::vector<PlanarCurveSegment> PlanarCurveFactory::createSegmentsFromTimeSections<TrajectoryXyw>(
    const TrajectoryXyw&, const std::vector<double>&);

template PlanarCurveSegment PlanarCurveFactory::createSegmentFromTimeInterval<BangBangTrajectory2D>(
    const BangBangTrajectory2D&, double, double);

template PlanarCurveSegment PlanarCurveFactory::createSegmentFromTimeInterval<TrajectoryXyw>(
    const TrajectoryXyw&, double, double);

template int PlanarCurveFactory::determineOptimalSegmentCount<BangBangTrajectory2D>(
    const BangBangTrajectory2D&);

template int PlanarCurveFactory::determineOptimalSegmentCount<TrajectoryXyw>(
    const TrajectoryXyw&);

} // namespace ctrl