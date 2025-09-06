#include "BangBangTrajectoryFactory.h"
#include "Utils.h"
#include <stdexcept>
#include <cmath>

namespace ctrl {

const double BangBangTrajectoryFactory::MAX_VEL_TOLERANCE = 0.2;
const float BangBangTrajectoryFactory::SYNC_ACCURACY = 1e-3f;
const std::function<float(float)> BangBangTrajectoryFactory::ALPHA_FN_ASYNC = 
    [](float alpha) { 
        return alpha + (((float)M_PI_2 - alpha) * 0.5f); 
    };

BangBangTrajectory2DAsync BangBangTrajectoryFactory::async(
    const Eigen::Vector2d& s0,
    const Eigen::Vector2d& s1,
    const Eigen::Vector2d& v0,
    double vmax,
    double acc,
    const Eigen::Vector2d& primaryDirection) {
    
    if (primaryDirection.isZero(1e-9)) {
        throw std::invalid_argument("zero primary direction vector");
    }

    const auto rotation = std::atan2(primaryDirection.y(), primaryDirection.x());
    const auto startToTarget = util::RotateAboutZ((s1 - s0).eval(), -rotation);
    const auto v0Rotated = util::RotateAboutZ(v0, -rotation);

    BangBangTrajectory2D child;
    child.generate(
        Eigen::Vector2d::Zero(),
        startToTarget,
        v0Rotated,
        (float)vmax,
        (float)acc,
        SYNC_ACCURACY,
        ALPHA_FN_ASYNC
    );
    
    return BangBangTrajectory2DAsync(child, s0, rotation);
}

BangBangTrajectory2D BangBangTrajectoryFactory::sync(
    const Eigen::Vector2d& s0,
    const Eigen::Vector2d& s1,
    const Eigen::Vector2d& v0,
    double vmax,
    double acc) {
    
    BangBangTrajectory2D traj;
    traj.generate(
        s0,
        s1,
        adaptVel(v0, vmax),
        (float)vmax,
        (float)acc,
        SYNC_ACCURACY,
        [](float f) { return f; } // Identity function
    );
    return traj;
}

std::unique_ptr<ITrajectory<double>> BangBangTrajectoryFactory::single(
    double initialPos,
    double finalPos,
    double initialVel,
    double maxVel,
    double maxAcc) {
    
    return std::make_unique<BangBangTrajectory1D>(
        singleDim(initialPos, finalPos, initialVel, maxVel, maxAcc));
}

BangBangTrajectory1D BangBangTrajectoryFactory::singleDim(
    double initialPos,
    double finalPos,
    double initialVel,
    double maxVel,
    double maxAcc) {
    
    BangBangTrajectory1D traj;
    traj.generate(
        (float)initialPos,
        (float)finalPos,
        (float)adaptVel(initialVel, maxVel),
        (float)maxVel,
        (float)maxAcc
    );
    return traj;
}

BangBangTrajectory1DOrient BangBangTrajectoryFactory::orientation(
    double initialPos,
    double finalPos,
    double initialVel,
    double maxVel,
    double maxAcc) {
    
    auto adaptedFinalPos = initialPos + util::WrapAngle(finalPos - util::WrapAngle(initialPos));
    return BangBangTrajectory1DOrient(
        singleDim(initialPos, adaptedFinalPos, adaptVel(initialVel, maxVel), maxVel, maxAcc));
}

Eigen::Vector2d BangBangTrajectoryFactory::adaptVel(const Eigen::Vector2d& v0, double vMax) {
    auto curVelAbs = v0.norm();
    if (curVelAbs > vMax && curVelAbs < vMax + MAX_VEL_TOLERANCE) {
        return v0.normalized() * vMax;
    }
    return v0;
}

double BangBangTrajectoryFactory::adaptVel(double v0, double vMax) {
    auto curVelAbs = std::abs(v0);
    if (curVelAbs > vMax && curVelAbs < vMax + MAX_VEL_TOLERANCE) {
        return std::copysign(vMax, v0);
    }
    return v0;
}

} // namespace ctrl
