#include "BangBangTrajectory1DOrient.h"

namespace ctrl {

double BangBangTrajectory1DOrient::getPositionMM(double t) const {
    return getPosition(t);
}

double BangBangTrajectory1DOrient::getPosition(double t) const {
    return util::WrapAngle(child_.getPosition(t));
}

double BangBangTrajectory1DOrient::getVelocity(double t) const {
    return child_.getVelocity(t);
}

double BangBangTrajectory1DOrient::getAcceleration(double t) const {
    return child_.getAcceleration(t);
}

double BangBangTrajectory1DOrient::getTotalTime() const {
    return child_.getTotalTime();
}

std::unique_ptr<ITrajectory<double>> BangBangTrajectory1DOrient::mirrored() const {
    BangBangTrajectory1D mirrored;
    mirrored.numParts = child_.numParts;
    for (int i = 0; i < child_.numParts; i++) {
        mirrored.parts[i].tEnd = child_.parts[i].tEnd;
        mirrored.parts[i].acc = child_.parts[i].acc;
        mirrored.parts[i].v0 = child_.parts[i].v0;
        mirrored.parts[i].s0 = (float)util::MirrorAngle(child_.parts[i].s0);
    }
    return std::make_unique<BangBangTrajectory1DOrient>(mirrored);
}

PosVelAcc<double> BangBangTrajectory1DOrient::getValuesAtTime(double tt) const {
    PosVelAcc<double> valuesAtTime = child_.getValuesAtTime(tt);
    return PosVelAcc<double>(
        util::WrapAngle(valuesAtTime.getPos()),
        valuesAtTime.getVel(),
        valuesAtTime.getAcc()
    );
}

std::vector<double> BangBangTrajectory1DOrient::getTimeSections() const {
    return child_.getTimeSections();
}

double BangBangTrajectory1DOrient::getMaxSpeed() const {
    auto sections = getTimeSections();
    double maxSpeed = 0.0;
    for (auto time : sections) {
        maxSpeed = std::max(maxSpeed, std::abs(getVelocity(time)));
    }
    return maxSpeed;
}

} // namespace ctrl