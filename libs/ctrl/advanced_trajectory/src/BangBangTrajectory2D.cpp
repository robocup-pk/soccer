#include "BangBangTrajectory2D.h"
#include <algorithm>
#include <cmath>

namespace ctrl {

Eigen::Vector2d BangBangTrajectory2D::getPositionMM(double t) const {
    return Eigen::Vector2d(x.getPositionMM(t), y.getPositionMM(t));
}

Eigen::Vector2d BangBangTrajectory2D::getPosition(double t) const {
    return Eigen::Vector2d(x.getPosition(t), y.getPosition(t));
}

Eigen::Vector2d BangBangTrajectory2D::getVelocity(double t) const {
    return Eigen::Vector2d(x.getVelocity(t), y.getVelocity(t));
}

Eigen::Vector2d BangBangTrajectory2D::getAcceleration(double t) const {
    return Eigen::Vector2d(x.getAcceleration(t), y.getAcceleration(t));
}

double BangBangTrajectory2D::getTotalTime() const {
    return std::max(x.getTotalTime(), y.getTotalTime());
}

std::unique_ptr<ITrajectory<Eigen::Vector2d>> BangBangTrajectory2D::mirrored() const {
    auto mirrored = std::make_unique<BangBangTrajectory2D>();
    mirrored->x.numParts = x.numParts;
    mirrored->y.numParts = y.numParts;
    for (int i = 0; i < BangBangTrajectory1D::MAX_PARTS; i++) {
        mirrored->x.parts[i].tEnd = x.parts[i].tEnd;
        mirrored->x.parts[i].acc = -x.parts[i].acc;
        mirrored->x.parts[i].v0 = -x.parts[i].v0;
        mirrored->x.parts[i].s0 = -x.parts[i].s0;
        mirrored->y.parts[i].tEnd = y.parts[i].tEnd;
        mirrored->y.parts[i].acc = -y.parts[i].acc;
        mirrored->y.parts[i].v0 = -y.parts[i].v0;
        mirrored->y.parts[i].s0 = -y.parts[i].s0;
    }
    return std::move(mirrored);
}

PosVelAcc<Eigen::Vector2d> BangBangTrajectory2D::getValuesAtTime(double tt) const {
    PosVelAcc<double> xValues = x.getValuesAtTime(tt);
    PosVelAcc<double> yValues = y.getValuesAtTime(tt);
    return PosVelAcc<Eigen::Vector2d>(
        Eigen::Vector2d(xValues.getPos(), yValues.getPos()),
        Eigen::Vector2d(xValues.getVel(), yValues.getVel()),
        Eigen::Vector2d(xValues.getAcc(), yValues.getAcc())
    );
}

std::vector<double> BangBangTrajectory2D::getTimeSections() const {
    std::vector<double> timeSections;
    timeSections.reserve(BangBangTrajectory1D::MAX_PARTS * 2);
    auto xSections = x.getTimeSections();
    auto ySections = y.getTimeSections();
    timeSections.insert(timeSections.end(), xSections.begin(), xSections.end());
    timeSections.insert(timeSections.end(), ySections.begin(), ySections.end());
    return timeSections;
}

double BangBangTrajectory2D::getMaxSpeed() const {
    auto sections = getTimeSections();
    double maxSpeed = 0.0;
    for (auto time : sections) {
        Eigen::Vector2d vel = getVelocity(time);
        maxSpeed = std::max(maxSpeed, vel.norm());
    }
    return maxSpeed;
}

BangBangTrajectory2D& BangBangTrajectory2D::generate(
    const Eigen::Vector2d& s0,
    const Eigen::Vector2d& s1,
    const Eigen::Vector2d& v0,
    float vmax,
    float acc,
    float accuracy,
    const std::function<float(float)>& alphaFn) {
    
    const auto s0x = (float)s0.x();
    const auto s0y = (float)s0.y();
    const auto s1x = (float)s1.x();
    const auto s1y = (float)s1.y();
    const auto v0x = (float)v0.x();
    const auto v0y = (float)v0.y();

    float inc = (float)M_PI / 8.0f;
    float alpha = (float)M_PI / 4.0f;

    // binary search, some iterations (fixed)
    while (inc > 1e-7f) {
        const float sA = std::sin(alphaFn(alpha));
        const float cA = std::cos(alphaFn(alpha));

        x.generate(s0x, s1x, v0x, vmax * cA, acc * cA);
        y.generate(s0y, s1y, v0y, vmax * sA, acc * sA);

        double diff = std::abs(x.getTotalTime() - y.getTotalTime());
        if (diff < accuracy) {
            break;
        }
        if (x.getTotalTime() > y.getTotalTime()) {
            alpha -= inc;
        } else {
            alpha += inc;
        }

        inc *= 0.5f;
    }
    return *this;
}

} // namespace ctrl