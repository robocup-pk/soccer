#pragma once

#include "ITrajectory.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include "PosVelAcc.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

// Forward declaration
class TrajPath;

/**
 * @brief 3D trajectory combining 2D position and 1D orientation trajectories.
 * 
 * Direct C++ port of TrajectoryXyw.java from Team Mannheim.
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
    
    /**
     * @brief Constructor with TrajPath (XY) and orientation trajectory (EXACT copy of Advanced's constructor)
     * @param trajPath TrajPath for XY position (from PathFinder with obstacle avoidance)
     * @param w_traj 1D orientation trajectory
     */
    TrajectoryXyw(const TrajPath& trajPath, const BangBangTrajectory1DOrient& w_traj);
    
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
    BangBangTrajectory2D xy_trajectory_;      ///< XY position trajectory (when using BangBang)
    BangBangTrajectory1DOrient w_trajectory_; ///< Orientation trajectory
    std::shared_ptr<TrajPath> xy_trajpath_;   ///< XY position trajectory (when using TrajPath from PathFinder)
    bool using_trajpath_;                     ///< Flag indicating which XY trajectory type is active
};

} // namespace ctrl