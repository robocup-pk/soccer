#include "TrajPath.h"
#include "TrajectoryGenerator.h"
#include "Utils.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

TrajPath TrajPath::with(const MoveConstraints& mc, const Eigen::Vector2d& curPos, 
                       const Eigen::Vector2d& curVel, const Eigen::Vector2d& dest) {
    
    // EXACT copy of Sumatra's with() method using TrajectoryGenerator
    std::cout << "[TrajPath] Using TrajectoryGenerator for trajectory creation" << std::endl;
    
    // Generate position trajectory using TrajectoryGenerator (EXACT Sumatra approach)
    BangBangTrajectory2D trajXY = TrajectoryGenerator::generatePositionTrajectory(mc, curPos, curVel, dest);
    
    // Generate orientation trajectory using TrajectoryGenerator
    BangBangTrajectory1DOrient trajW = TrajectoryGenerator::generateRotationTrajectory(0.0, 0.0, 0.0, mc);
    
    // Combine into TrajectoryXyw
    TrajectoryXyw trajectory(trajXY, trajW);
    
    return TrajPath(trajectory, trajectory.getTotalTime(), nullptr);
}

TrajPath TrajPath::with(const Eigen::Vector2d& curPos, const Eigen::Vector2d& curVel, 
                       double curTheta, double curOmega,
                       const Eigen::Vector2d& dest, double destTheta,
                       double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc) {
    
    // EXACT copy of Sumatra's with() method using TrajectoryGenerator
    std::cout << "[TrajPath] Using TrajectoryGenerator for full state trajectory" << std::endl;
    
    // Create MoveConstraints for TrajectoryGenerator
    MoveConstraints mc;
    mc.setVelMax(maxVel).setAccMax(maxAcc).setVelMaxW(maxOmega).setAccMaxW(maxOmegaAcc);
    
    // Generate position trajectory using TrajectoryGenerator
    BangBangTrajectory2D trajXY = TrajectoryGenerator::generatePositionTrajectory(mc, curPos, curVel, dest);
    
    // Generate orientation trajectory using TrajectoryGenerator  
    BangBangTrajectory1DOrient trajW = TrajectoryGenerator::generateRotationTrajectory(curTheta, curOmega, destTheta, mc);
    
    // Combine into TrajectoryXyw
    TrajectoryXyw trajectory(trajXY, trajW);
    
    return TrajPath(trajectory, trajectory.getTotalTime(), nullptr);
}

TrajPath TrajPath::append(double connectionTime, const Eigen::Vector2d& dest, double destTheta,
                         double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc) {
    
    // EXACT copy of Sumatra's append() method (lines 85-88)
    
    // Get current position and velocity at connection time (this is the KEY for smooth motion!)
    Eigen::Vector3d curPos3d = this->getPosition(connectionTime);
    Eigen::Vector3d curVel3d = this->getVelocity(connectionTime);
    
    Eigen::Vector2d curPos = curPos3d.head<2>();
    Eigen::Vector2d curVel = curVel3d.head<2>();
    double curTheta = curPos3d.z();
    double curOmega = curVel3d.z();
    
    std::cout << "[TrajPath] Appending segment at t=" << connectionTime 
              << "s with state: pos=(" << curPos.x() << "," << curPos.y() << "," << curTheta 
              << ") vel=(" << curVel.x() << "," << curVel.y() << "," << curOmega << ")" << std::endl;
    
    // Create child path starting from current state (NOT from rest!)
    TrajPath childPath = TrajPath::with(curPos, curVel, curTheta, curOmega,
                                       dest, destTheta, maxVel, maxAcc, maxOmega, maxOmegaAcc);
    
    // Connect this path with the child path
    return this->connect(childPath, connectionTime);
}

TrajPath TrajPath::connect(const TrajPath& path, double tConnect) {
    // EXACT copy of Sumatra's connect() method (lines 149-156)
    
    if (child_ != nullptr && tConnect > tEnd_) {
        auto newChild = std::make_shared<TrajPath>(child_->connect(path, tConnect - tEnd_));
        return TrajPath(trajectory_, tEnd_, newChild);
    }
    
    auto childPtr = std::make_shared<TrajPath>(path);
    return TrajPath(trajectory_, tConnect, childPtr);
}

Eigen::Vector2d TrajPath::getNextDestination(double t) const {
    // This is used by Sumatra's executePath method
    return getPosition(t).head<2>();
}

Eigen::Vector3d TrajPath::getPosition(double t) const {
    // EXACT copy of Sumatra's getPosition() method (lines 186-197)
    
    if (t <= tEnd_) {
        return trajectory_.getPosition(t);
    }
    
    if (child_ != nullptr) {
        return child_->getPosition(t - tEnd_);
    }
    
    return trajectory_.getPosition(tEnd_);
}

Eigen::Vector3d TrajPath::getVelocity(double t) const {
    // EXACT copy of Sumatra's getVelocity() method (lines 201-212)
    
    if (t <= tEnd_) {
        return trajectory_.getVelocity(t);
    }
    
    if (child_ != nullptr) {
        return child_->getVelocity(t - tEnd_);
    }
    
    return trajectory_.getVelocity(tEnd_);
}

Eigen::Vector3d TrajPath::getAcceleration(double t) const {
    // EXACT copy of Sumatra's getAcceleration() method (lines 216-227)
    
    if (t <= tEnd_) {
        return trajectory_.getAcceleration(t);
    }
    
    if (child_ != nullptr) {
        return child_->getAcceleration(t - tEnd_);
    }
    
    return trajectory_.getAcceleration(tEnd_);
}

Eigen::Vector3d TrajPath::getFinalDestination() const {
    // EXACT copy of Sumatra's getFinalDestination() method (lines 160-167)
    
    if (child_ != nullptr) {
        return child_->getFinalDestination();
    }
    
    return trajectory_.getPosition(tEnd_);
}

double TrajPath::getTotalTime() const {
    // EXACT copy of Sumatra's getTotalTime() method (lines 231-238)
    
    if (child_ != nullptr) {
        return tEnd_ + child_->getTotalTime();
    }
    
    return tEnd_;
}

double TrajPath::getMaxSpeed() const {
    // Sample the trajectory to find maximum velocity
    double maxSpeed = 0.0;
    double totalTime = getTotalTime();
    
    if (totalTime <= 0) {
        return 0.0;
    }
    
    int numSamples = std::max(10, (int)(totalTime * 20)); // Sample every 50ms or at least 10 points
    for (int i = 0; i <= numSamples; ++i) {
        double t = (i * totalTime) / numSamples;
        Eigen::Vector3d vel = getVelocity(t);
        double speed = vel.head<2>().norm(); // Only XY velocity
        maxSpeed = std::max(maxSpeed, speed);
    }
    
    return maxSpeed;
}

} // namespace ctrl