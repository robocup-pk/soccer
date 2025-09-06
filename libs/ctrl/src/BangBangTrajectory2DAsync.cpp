#include "BangBangTrajectory2DAsync.h"
#include <cmath>

namespace ctrl {

Eigen::Vector2d BangBangTrajectory2DAsync::getPositionMM(double t) const {
    Eigen::Vector2d childPos = child_.getPositionMM(t);
    Eigen::Vector2d rotated = util::RotateAboutZ(childPos, rotation_);
    return rotated + initialPos_ * 1000.0; // Convert to mm
}

Eigen::Vector2d BangBangTrajectory2DAsync::getPosition(double t) const {
    Eigen::Vector2d childPos = child_.getPosition(t);
    Eigen::Vector2d rotated = util::RotateAboutZ(childPos, rotation_);
    return rotated + initialPos_;
}

Eigen::Vector2d BangBangTrajectory2DAsync::getVelocity(double t) const {
    Eigen::Vector2d childVel = child_.getVelocity(t);
    return util::RotateAboutZ(childVel, rotation_);
}

Eigen::Vector2d BangBangTrajectory2DAsync::getAcceleration(double t) const {
    Eigen::Vector2d childAcc = child_.getAcceleration(t);
    return util::RotateAboutZ(childAcc, rotation_);
}

double BangBangTrajectory2DAsync::getTotalTime() const {
    return std::max(child_.x.getTotalTime(), child_.y.getTotalTime());
}

double BangBangTrajectory2DAsync::getTotalTimeToPrimaryDirection() const {
    return child_.y.getTotalTime();
}

std::unique_ptr<ITrajectory<Eigen::Vector2d>> BangBangTrajectory2DAsync::mirrored() const {
    return std::make_unique<BangBangTrajectory2DAsync>(
        child_,
        initialPos_ * -1.0,
        util::WrapAngle(rotation_ + M_PI)
    );
}

PosVelAcc<Eigen::Vector2d> BangBangTrajectory2DAsync::getValuesAtTime(double tt) const {
    PosVelAcc<Eigen::Vector2d> valuesAtTime = child_.getValuesAtTime(tt);
    
    Eigen::Vector2d rotatedPos = util::RotateAboutZ(valuesAtTime.getPos(), rotation_);
    Eigen::Vector2d rotatedVel = util::RotateAboutZ(valuesAtTime.getVel(), rotation_);
    Eigen::Vector2d rotatedAcc = util::RotateAboutZ(valuesAtTime.getAcc(), rotation_);
    
    return PosVelAcc<Eigen::Vector2d>(
        rotatedPos + initialPos_,
        rotatedVel,
        rotatedAcc
    );
}

std::vector<double> BangBangTrajectory2DAsync::getTimeSections() const {
    return child_.getTimeSections();
}

double BangBangTrajectory2DAsync::getMaxSpeed() const {
    return child_.getMaxSpeed();
}

} // namespace ctrl