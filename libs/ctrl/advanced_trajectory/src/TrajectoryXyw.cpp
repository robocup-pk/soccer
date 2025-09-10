#include "TrajectoryXyw.h"
#include "TrajPath.h"
#include <algorithm>
#include <stdexcept>

namespace ctrl {

TrajectoryXyw::TrajectoryXyw(const BangBangTrajectory2D& xy_traj, const BangBangTrajectory1DOrient& w_traj)
    : xy_trajectory_(xy_traj), w_trajectory_(w_traj), xy_trajpath_(nullptr), using_trajpath_(false) {
}

TrajectoryXyw::TrajectoryXyw(const TrajPath& trajPath, const BangBangTrajectory1DOrient& w_traj)
    : w_trajectory_(w_traj), xy_trajpath_(std::make_shared<TrajPath>(trajPath)), using_trajpath_(true) {
}

Eigen::Vector3d TrajectoryXyw::getPositionMM(double t) const {
    Eigen::Vector2d xy_pos;
    if (using_trajpath_) {
        Eigen::Vector3d pos3d = xy_trajpath_->getPositionMM(t);
        xy_pos = pos3d.head<2>();
    } else {
        xy_pos = xy_trajectory_.getPositionMM(t);
    }
    double w_pos = w_trajectory_.getPositionMM(t);
    return Eigen::Vector3d(xy_pos.x(), xy_pos.y(), w_pos);
}

Eigen::Vector3d TrajectoryXyw::getPosition(double t) const {
    Eigen::Vector2d xy_pos;
    if (using_trajpath_) {
        Eigen::Vector3d pos3d = xy_trajpath_->getPosition(t);
        xy_pos = pos3d.head<2>();
    } else {
        xy_pos = xy_trajectory_.getPosition(t);
    }
    double w_pos = w_trajectory_.getPosition(t);
    return Eigen::Vector3d(xy_pos.x(), xy_pos.y(), w_pos);
}

Eigen::Vector3d TrajectoryXyw::getVelocity(double t) const {
    Eigen::Vector2d xy_vel;
    if (using_trajpath_) {
        Eigen::Vector3d vel3d = xy_trajpath_->getVelocity(t);
        xy_vel = vel3d.head<2>();
    } else {
        xy_vel = xy_trajectory_.getVelocity(t);
    }
    double w_vel = w_trajectory_.getVelocity(t);
    return Eigen::Vector3d(xy_vel.x(), xy_vel.y(), w_vel);
}

Eigen::Vector3d TrajectoryXyw::getAcceleration(double t) const {
    Eigen::Vector2d xy_acc;
    if (using_trajpath_) {
        Eigen::Vector3d acc3d = xy_trajpath_->getAcceleration(t);
        xy_acc = acc3d.head<2>();
    } else {
        xy_acc = xy_trajectory_.getAcceleration(t);
    }
    double w_acc = w_trajectory_.getAcceleration(t);
    return Eigen::Vector3d(xy_acc.x(), xy_acc.y(), w_acc);
}

double TrajectoryXyw::getTotalTime() const {
    // Total time is the maximum of XY and orientation trajectory times
    double xy_time = using_trajpath_ ? xy_trajpath_->getTotalTime() : xy_trajectory_.getTotalTime();
    return std::max(xy_time, w_trajectory_.getTotalTime());
}

std::unique_ptr<ITrajectory<Eigen::Vector3d>> TrajectoryXyw::mirrored() const {
    // Mirror orientation trajectory
    auto w_mirrored = w_trajectory_.mirrored();
    const BangBangTrajectory1DOrient* w_concrete = dynamic_cast<const BangBangTrajectory1DOrient*>(w_mirrored.get());
    
    if (!w_concrete) {
        throw std::runtime_error("Failed to cast mirrored orientation trajectory to concrete type");
    }
    
    if (using_trajpath_) {
        // For TrajPath, we don't have a mirror method, so we'll throw for now
        // In a full implementation, TrajPath would need a mirror method
        throw std::runtime_error("Mirroring not yet implemented for TrajPath-based TrajectoryXyw");
    } else {
        // Mirror XY trajectory
        auto xy_mirrored = xy_trajectory_.mirrored();
        const BangBangTrajectory2D* xy_concrete = dynamic_cast<const BangBangTrajectory2D*>(xy_mirrored.get());
        
        if (!xy_concrete) {
            throw std::runtime_error("Failed to cast mirrored XY trajectory to concrete type");
        }
        
        return std::make_unique<TrajectoryXyw>(*xy_concrete, *w_concrete);
    }
}

PosVelAcc<Eigen::Vector3d> TrajectoryXyw::getValuesAtTime(double t) const {
    PosVelAcc<double> w_values = w_trajectory_.getValuesAtTime(t);
    
    PosVelAcc<Eigen::Vector3d> result;
    
    if (using_trajpath_) {
        // For TrajPath, we need to call individual methods since it doesn't have getValuesAtTime
        Eigen::Vector3d pos3d = xy_trajpath_->getPosition(t);
        Eigen::Vector3d vel3d = xy_trajpath_->getVelocity(t);
        Eigen::Vector3d acc3d = xy_trajpath_->getAcceleration(t);
        
        result.pos = Eigen::Vector3d(pos3d.x(), pos3d.y(), w_values.pos);
        result.vel = Eigen::Vector3d(vel3d.x(), vel3d.y(), w_values.vel);
        result.acc = Eigen::Vector3d(acc3d.x(), acc3d.y(), w_values.acc);
    } else {
        PosVelAcc<Eigen::Vector2d> xy_values = xy_trajectory_.getValuesAtTime(t);
        
        result.pos = Eigen::Vector3d(xy_values.pos.x(), xy_values.pos.y(), w_values.pos);
        result.vel = Eigen::Vector3d(xy_values.vel.x(), xy_values.vel.y(), w_values.vel);
        result.acc = Eigen::Vector3d(xy_values.acc.x(), xy_values.acc.y(), w_values.acc);
    }
    
    return result;
}

std::vector<double> TrajectoryXyw::getTimeSections() const {
    // Combine time sections from both trajectories
    std::vector<double> xy_sections;
    if (using_trajpath_) {
        // For TrajPath, we don't have getTimeSections, so create basic sections
        double total_time = xy_trajpath_->getTotalTime();
        if (total_time > 0) {
            xy_sections = {0.0, total_time};
        }
    } else {
        xy_sections = xy_trajectory_.getTimeSections();
    }
    
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
    double xy_max_speed;
    if (using_trajpath_) {
        // For TrajPath, we need to sample velocities to find max speed
        double max_speed = 0.0;
        double total_time = xy_trajpath_->getTotalTime();
        int num_samples = std::max(10, (int)(total_time * 20)); // Sample every 50ms or at least 10 points
        for (int i = 0; i <= num_samples; ++i) {
            double t = (i * total_time) / num_samples;
            Eigen::Vector3d vel = xy_trajpath_->getVelocity(t);
            double speed = vel.head<2>().norm();
            max_speed = std::max(max_speed, speed);
        }
        xy_max_speed = max_speed;
    } else {
        xy_max_speed = xy_trajectory_.getMaxSpeed();
    }
    
    double w_max_speed = w_trajectory_.getMaxSpeed();
    
    // Return XY max speed as primary (angular speed has different units)
    return xy_max_speed;
}

double TrajectoryXyw::getTotalTimeToPrimaryDirection() const {
    // Primary direction is typically XY movement
    if (using_trajpath_) {
        // For TrajPath, return total time as primary direction time
        return xy_trajpath_->getTotalTime();
    } else {
        return xy_trajectory_.getTotalTimeToPrimaryDirection();
    }
}

bool TrajectoryXyw::isValid() const {
    double xy_time = using_trajpath_ ? xy_trajpath_->getTotalTime() : xy_trajectory_.getTotalTime();
    return (xy_time > 0.0 || w_trajectory_.getTotalTime() > 0.0);
}

} // namespace ctrl