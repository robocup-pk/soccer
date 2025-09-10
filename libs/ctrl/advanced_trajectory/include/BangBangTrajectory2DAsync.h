#pragma once

#include "ITrajectory.h"
#include "BangBangTrajectory2D.h"
#include "Utils.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

/**
 * @brief Asynchronous Bang Bang Trajectory for two dimensions.
 * 
 * X and Y are not synchronized in this version. The trajectory tries
 * to get on the line defined by target position and primary direction first.
 * 
 * Direct C++ port of BangBangTrajectory2DAsync.java from Team Mannheim
 */
class BangBangTrajectory2DAsync : public ITrajectory<Eigen::Vector2d> {
public:
    BangBangTrajectory2DAsync(const BangBangTrajectory2D& child, 
                              const Eigen::Vector2d& initialPos,
                              double rotation)
        : child_(child), initialPos_(initialPos), rotation_(rotation) {}
    
    BangBangTrajectory2DAsync() 
        : child_(), initialPos_(Eigen::Vector2d::Zero()), rotation_(0.0) {}

    // --- ITrajectory Interface ---
    Eigen::Vector2d getPositionMM(double t) const override;
    Eigen::Vector2d getPosition(double t) const override;
    Eigen::Vector2d getVelocity(double t) const override;
    Eigen::Vector2d getAcceleration(double t) const override;
    double getTotalTime() const override;
    double getTotalTimeToPrimaryDirection() const override;
    std::unique_ptr<ITrajectory<Eigen::Vector2d>> mirrored() const override;
    PosVelAcc<Eigen::Vector2d> getValuesAtTime(double tt) const override;
    std::vector<double> getTimeSections() const override;
    double getMaxSpeed() const override;

    // Getters (as needed Lombok @Getter)
    const BangBangTrajectory2D& getChild() const { return child_; }
    const Eigen::Vector2d& getInitialPos() const { return initialPos_; }
    double getRotation() const { return rotation_; }

private:
    BangBangTrajectory2D child_;
    Eigen::Vector2d initialPos_;
    double rotation_;
};

} // namespace ctrl