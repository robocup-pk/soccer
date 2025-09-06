#include "TrajectoryXyw.h"
#include <algorithm>
#include <stdexcept>

namespace ctrl {

TrajectoryXyw::TrajectoryXyw(const BangBangTrajectory2D& xy_traj, const BangBangTrajectory1DOrient& w_traj)
    : xy_trajectory_(xy_traj), w_trajectory_(w_traj) {
}

Eigen::Vector3d TrajectoryXyw::getPositionMM(double t) const {
    Eigen::Vector2d xy_pos = xy_trajectory_.getPositionMM(t);
    double w_pos = w_trajectory_.getPositionMM(t);
    return Eigen::Vector3d(xy_pos.x(), xy_pos.y(), w_pos);
}

Eigen::Vector3d TrajectoryXyw::getPosition(double t) const {
    Eigen::Vector2d xy_pos = xy_trajectory_.getPosition(t);
    double w_pos = w_trajectory_.getPosition(t);
    return Eigen::Vector3d(xy_pos.x(), xy_pos.y(), w_pos);
}

Eigen::Vector3d TrajectoryXyw::getVelocity(double t) const {
    Eigen::Vector2d xy_vel = xy_trajectory_.getVelocity(t);
    double w_vel = w_trajectory_.getVelocity(t);
    return Eigen::Vector3d(xy_vel.x(), xy_vel.y(), w_vel);
}

Eigen::Vector3d TrajectoryXyw::getAcceleration(double t) const {
    Eigen::Vector2d xy_acc = xy_trajectory_.getAcceleration(t);
    double w_acc = w_trajectory_.getAcceleration(t);
    return Eigen::Vector3d(xy_acc.x(), xy_acc.y(), w_acc);
}

double TrajectoryXyw::getTotalTime() const {
    // Total time is the maximum of XY and orientation trajectory times
    return std::max(xy_trajectory_.getTotalTime(), w_trajectory_.getTotalTime());
}

std::unique_ptr<ITrajectory<Eigen::Vector3d>> TrajectoryXyw::mirrored() const {
    // Mirror both XY and orientation trajectories
    auto xy_mirrored = xy_trajectory_.mirrored();
    auto w_mirrored = w_trajectory_.mirrored();
    
    // Cast to concrete types for constructor
    const BangBangTrajectory2D* xy_concrete = dynamic_cast<const BangBangTrajectory2D*>(xy_mirrored.get());
    const BangBangTrajectory1DOrient* w_concrete = dynamic_cast<const BangBangTrajectory1DOrient*>(w_mirrored.get());
    
    if (!xy_concrete || !w_concrete) {
        throw std::runtime_error("Failed to cast mirrored trajectories to concrete types");
    }
    
    return std::make_unique<TrajectoryXyw>(*xy_concrete, *w_concrete);
}

PosVelAcc<Eigen::Vector3d> TrajectoryXyw::getValuesAtTime(double t) const {
    PosVelAcc<Eigen::Vector2d> xy_values = xy_trajectory_.getValuesAtTime(t);
    PosVelAcc<double> w_values = w_trajectory_.getValuesAtTime(t);
    
    PosVelAcc<Eigen::Vector3d> result;
    result.pos = Eigen::Vector3d(xy_values.pos.x(), xy_values.pos.y(), w_values.pos);
    result.vel = Eigen::Vector3d(xy_values.vel.x(), xy_values.vel.y(), w_values.vel);
    result.acc = Eigen::Vector3d(xy_values.acc.x(), xy_values.acc.y(), w_values.acc);
    
    return result;
}

std::vector<double> TrajectoryXyw::getTimeSections() const {
    // Combine time sections from both trajectories
    std::vector<double> xy_sections = xy_trajectory_.getTimeSections();
    std::vector<double> w_sections = w_trajectory_.getTimeSections();
    
    // Merge and sort unique time points
    std::vector<double> combined_sections;
    combined_sections.reserve(xy_sections.size() + w_sections.size());
    
    combined_sections.insert(combined_sections.end(), xy_sections.begin(), xy_sections.end());
    combined_sections.insert(combined_sections.end(), w_sections.begin(), w_sections.end());
    
    std::sort(combined_sections.begin(), combined_sections.end());
    
    // Remove duplicates
    auto last = std::unique(combined_sections.begin(), combined_sections.end());
    combined_sections.erase(last, combined_sections.end());
    
    return combined_sections;
}

double TrajectoryXyw::getMaxSpeed() const {
    // Maximum speed considering both XY and angular components
    double xy_max_speed = xy_trajectory_.getMaxSpeed();
    double w_max_speed = w_trajectory_.getMaxSpeed();
    
    // Return XY max speed as primary (angular speed has different units)
    return xy_max_speed;
}

double TrajectoryXyw::getTotalTimeToPrimaryDirection() const {
    // Primary direction is typically XY movement
    return xy_trajectory_.getTotalTimeToPrimaryDirection();
}

bool TrajectoryXyw::isValid() const {
    return (xy_trajectory_.getTotalTime() > 0.0 || w_trajectory_.getTotalTime() > 0.0);
}

} // namespace ctrl