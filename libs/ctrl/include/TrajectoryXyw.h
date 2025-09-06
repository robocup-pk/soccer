#pragma once

#include "ITrajectory.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include "PosVelAcc.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

/**
 * @brief 3D trajectory combining 2D position and 1D orientation trajectories.
 * 
 * Direct C++ port of TrajectoryXyw.java from TIGERs Mannheim.
 * This class coordinates XY translation and W (orientation) rotation,
 * allowing them to have different durations and timing.
 */
class TrajectoryXyw : public ITrajectory<Eigen::Vector3d> {
public:
    TrajectoryXyw() = default;
    
    /**
     * @brief Constructor with separate XY and orientation trajectories
     * @param xy_traj 2D position trajectory
     * @param w_traj 1D orientation trajectory
     */
    TrajectoryXyw(const BangBangTrajectory2D& xy_traj, const BangBangTrajectory1DOrient& w_traj);
    
    // --- ITrajectory Interface ---
    Eigen::Vector3d getPositionMM(double t) const override;
    Eigen::Vector3d getPosition(double t) const override;
    Eigen::Vector3d getVelocity(double t) const override;
    Eigen::Vector3d getAcceleration(double t) const override;
    double getTotalTime() const override;
    std::unique_ptr<ITrajectory<Eigen::Vector3d>> mirrored() const override;
    PosVelAcc<Eigen::Vector3d> getValuesAtTime(double t) const override;
    std::vector<double> getTimeSections() const override;
    double getMaxSpeed() const override;
    double getTotalTimeToPrimaryDirection() const override;
    
    /**
     * @brief Get the XY trajectory component
     */
    const BangBangTrajectory2D& getXyTrajectory() const { return xy_trajectory_; }
    
    /**
     * @brief Get the orientation trajectory component
     */
    const BangBangTrajectory1DOrient& getOrientationTrajectory() const { return w_trajectory_; }
    
    /**
     * @brief Check if trajectories are valid
     */
    bool isValid() const;

private:
    BangBangTrajectory2D xy_trajectory_;      ///< XY position trajectory
    BangBangTrajectory1DOrient w_trajectory_; ///< Orientation trajectory
};

} // namespace ctrl