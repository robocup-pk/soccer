#pragma once

#include "PlanarCurveSegment.h"
#include "PlanarCurve.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1D.h"
#include "TrajectoryXyw.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief Factory for converting between BangBang trajectories and PlanarCurve segments
 * 
 * Direct C++ port of PlanarCurveFactory.java from Team Mannheim.
 * This class provides methods to convert bang-bang trajectories into planar curve segments
 * and vice versa, enabling integration between the two trajectory representations.
 */
class PlanarCurveFactory {
public:
    /**
     * @brief Convert a 2D BangBang trajectory into PlanarCurve segments
     * @param trajectory 2D BangBang trajectory to convert
     * @param numSegments Number of segments to create (default: auto-detect from time sections)
     * @return PlanarCurve composed of segments
     */
    static PlanarCurve fromBangBangTrajectory2D(const BangBangTrajectory2D& trajectory, 
                                                int numSegments = -1);
    
    /**
     * @brief Convert a 3D BangBang trajectory (XYW) into PlanarCurve segments
     * @param trajectory 3D BangBang trajectory to convert
     * @param numSegments Number of segments to create (default: auto-detect from time sections)
     * @return PlanarCurve composed of segments (XY only, orientation handled separately)
     */
    static PlanarCurve fromTrajectoryXyw(const TrajectoryXyw& trajectory, 
                                         int numSegments = -1);
    
    /**
     * @brief Create PlanarCurve segments from trajectory time sections
     * @param trajectory The trajectory to segment
     * @param timeSections Vector of time points to create segments between
     * @return Vector of PlanarCurveSegment objects
     */
    template<typename TrajectoryType>
    static std::vector<PlanarCurveSegment> createSegmentsFromTimeSections(
        const TrajectoryType& trajectory, 
        const std::vector<double>& timeSections);
    
    /**
     * @brief Convert a single time interval of a trajectory into a PlanarCurveSegment
     * @param trajectory The source trajectory
     * @param tStart Start time of the segment
     * @param tEnd End time of the segment
     * @return PlanarCurveSegment representing this time interval
     */
    template<typename TrajectoryType>
    static PlanarCurveSegment createSegmentFromTimeInterval(
        const TrajectoryType& trajectory,
        double tStart, 
        double tEnd);
    
    /**
     * @brief Determine optimal number of segments based on trajectory complexity
     * @param trajectory The trajectory to analyze
     * @return Recommended number of segments
     */
    template<typename TrajectoryType>
    static int determineOptimalSegmentCount(const TrajectoryType& trajectory);

private:
    /**
     * @brief Helper to extract 2D position from various trajectory types
     */
    template<typename TrajectoryType>
    static Eigen::Vector2d extractPosition2D(const TrajectoryType& trajectory, double t);
    
    /**
     * @brief Helper to extract 2D velocity from various trajectory types  
     */
    template<typename TrajectoryType>
    static Eigen::Vector2d extractVelocity2D(const TrajectoryType& trajectory, double t);
};

} // namespace ctrl