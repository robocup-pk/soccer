#pragma once

#include "ITrajectory.h"
#include "BangBangTrajectory1D.h" 
#include <Eigen/Dense>
#include <memory>
#include <functional>

namespace ctrl {

/**
 * @brief Bang Bang Trajectory for two dimensions.
 * 
 * Direct C++ port of BangBangTrajectory2D.java from Team Mannheim
 */
class BangBangTrajectory2D : public ITrajectory<Eigen::Vector2d> {
public:
    BangBangTrajectory2D() = default;

    // --- ITrajectory Interface ---
    Eigen::Vector2d getPositionMM(double t) const override;
    Eigen::Vector2d getPosition(double t) const override;
    Eigen::Vector2d getVelocity(double t) const override;
    Eigen::Vector2d getAcceleration(double t) const override;
    double getTotalTime() const override;
    std::unique_ptr<ITrajectory<Eigen::Vector2d>> mirrored() const override;
    PosVelAcc<Eigen::Vector2d> getValuesAtTime(double tt) const override;
    std::vector<double> getTimeSections() const override;
    double getMaxSpeed() const override;

    /**
     * @brief Generate the trajectory based on the input parameters
     * @param s0 Initial position
     * @param s1 Target position
     * @param v0 Initial velocity
     * @param vmax Max velocity
     * @param acc Acceleration
     * @param accuracy Synchronization accuracy
     * @param alphaFn Alpha function for synchronization
     * @return Reference to this object for chaining
     */
    BangBangTrajectory2D& generate(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        float vmax,
        float acc,
        float accuracy,
        const std::function<float(float)>& alphaFn
    );

    // Public for friend access (like Advanced package-private)
    BangBangTrajectory1D x;
    BangBangTrajectory1D y;
};

} // namespace ctrl