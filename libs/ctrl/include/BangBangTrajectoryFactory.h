#pragma once

#include "BangBangTrajectory1D.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include "BangBangTrajectory2DAsync.h"
#include "BBTrajectoryPart.h"
#include "PlanarCurve.h"
#include "DestinationForTimedPositionCalc.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>

namespace ctrl {

/**
 * @brief Factory for creating Bang Bang trajectories.
 * 
 * Direct C++ port of BangBangTrajectoryFactory.java from TIGERs Mannheim
 */
class BangBangTrajectoryFactory {
public:
    static const double MAX_VEL_TOLERANCE;
    static const float SYNC_ACCURACY;
    static const std::function<float(float)> ALPHA_FN_ASYNC;

    /**
     * @brief Create asynchronous 2D trajectory
     * @param s0 Start position
     * @param s1 Target position
     * @param v0 Initial velocity
     * @param vmax Maximum velocity
     * @param acc Maximum acceleration
     * @param primaryDirection Primary direction vector
     * @return Asynchronous 2D trajectory
     */
    BangBangTrajectory2DAsync async(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vmax,
        double acc,
        const Eigen::Vector2d& primaryDirection
    );

    /**
     * @brief Create synchronized 2D trajectory
     * @param s0 Start position
     * @param s1 Target position  
     * @param v0 Initial velocity
     * @param vmax Maximum velocity
     * @param acc Maximum acceleration
     * @return Synchronized 2D trajectory
     */
    BangBangTrajectory2D sync(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vmax,
        double acc
    );

    /**
     * @brief Create single dimension trajectory
     * @param initialPos Initial position
     * @param finalPos Final position
     * @param initialVel Initial velocity
     * @param maxVel Maximum velocity
     * @param maxAcc Maximum acceleration
     * @return 1D trajectory
     */
    std::unique_ptr<ITrajectory<double>> single(
        double initialPos,
        double finalPos,
        double initialVel,
        double maxVel,
        double maxAcc
    );

    /**
     * @brief Create single dimension trajectory (returns concrete type)
     * @param initialPos Initial position
     * @param finalPos Final position
     * @param initialVel Initial velocity
     * @param maxVel Maximum velocity
     * @param maxAcc Maximum acceleration
     * @return 1D bang-bang trajectory
     */
    BangBangTrajectory1D singleDim(
        double initialPos,
        double finalPos,
        double initialVel,
        double maxVel,
        double maxAcc
    );

    /**
     * @brief Create orientation trajectory
     * @param initialPos Initial orientation
     * @param finalPos Final orientation
     * @param initialVel Initial angular velocity
     * @param maxVel Maximum angular velocity
     * @param maxAcc Maximum angular acceleration
     * @return Orientation trajectory
     */
    BangBangTrajectory1DOrient orientation(
        double initialPos,
        double finalPos,
        double initialVel,
        double maxVel,
        double maxAcc
    );
    
    // --- PlanarCurve Integration Methods (Sumatra approach) ---
    
    /**
     * @brief Create PlanarCurve from 2D BangBang trajectory
     * @param s0 Start position
     * @param s1 Target position  
     * @param v0 Initial velocity
     * @param vmax Maximum velocity
     * @param acc Maximum acceleration
     * @param numSegments Number of curve segments (default: auto-detect)
     * @return PlanarCurve representation
     */
    PlanarCurve toPlanarCurve(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vmax,
        double acc,
        int numSegments = -1
    );
    
    /**
     * @brief Create BBTrajectoryPart segments directly (advanced usage)
     * @param initialPos Initial position
     * @param finalPos Final position
     * @param initialVel Initial velocity
     * @param maxVel Maximum velocity
     * @param maxAcc Maximum acceleration
     * @return Vector of trajectory parts/segments
     */
    std::vector<BBTrajectoryPart> createTrajectoryParts(
        double initialPos,
        double finalPos,
        double initialVel,
        double maxVel,
        double maxAcc
    );
    
    // --- Timed Interception Methods (DestinationForTimedPositionCalc integration) ---
    
    /**
     * @brief Create trajectory with timed interception (overshooting)
     * @param s0 Start position
     * @param s1 Target position  
     * @param v0 Initial velocity
     * @param vmax Maximum velocity
     * @param acc Maximum acceleration
     * @param targetTime Target time to reach position
     * @return Synchronized 2D trajectory that reaches target at specific time
     */
    BangBangTrajectory2D syncTimed(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vmax,
        double acc,
        double targetTime
    );
    
    /**
     * @brief Create asynchronous trajectory with timed interception
     * @param s0 Start position
     * @param s1 Target position  
     * @param v0 Initial velocity
     * @param vmax Maximum velocity
     * @param acc Maximum acceleration
     * @param targetTime Target time to reach position
     * @param primaryDirection Primary direction of movement
     * @return Asynchronous 2D trajectory that reaches target at specific time
     */
    BangBangTrajectory2DAsync asyncTimed(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vmax,
        double acc,
        double targetTime,
        const Eigen::Vector2d& primaryDirection
    );

private:
    /**
     * @brief Adapt velocity to stay within limits
     * @param v0 Initial velocity vector
     * @param vMax Maximum velocity
     * @return Adapted velocity vector
     */
    static Eigen::Vector2d adaptVel(const Eigen::Vector2d& v0, double vMax);

    /**
     * @brief Adapt velocity to stay within limits (scalar version)
     * @param v0 Initial velocity
     * @param vMax Maximum velocity
     * @return Adapted velocity
     */
    static double adaptVel(double v0, double vMax);

private:
    /// Calculator for timed interception trajectories
    DestinationForTimedPositionCalc destination_calc_;
};

} // namespace ctrl