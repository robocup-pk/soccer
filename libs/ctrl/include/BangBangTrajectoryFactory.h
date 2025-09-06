#pragma once

#include "BangBangTrajectory1D.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include "BangBangTrajectory2DAsync.h"
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
};

} // namespace ctrl