#pragma once

#include "ITrajectory.h"
#include "BangBangTrajectory1D.h"
#include "Utils.h"
#include <memory>

namespace ctrl {

/**
 * @brief Bang Bang Trajectory for one dimension for orientation.
 * 
 * Direct C++ port of BangBangTrajectory1DOrient.java from Team Mannheim
 */
class BangBangTrajectory1DOrient : public ITrajectory<double> {
public:
    BangBangTrajectory1DOrient(const BangBangTrajectory1D& child) : child_(child) {}
    BangBangTrajectory1DOrient() = default;

    // --- ITrajectory Interface ---
    double getPositionMM(double t) const override;
    double getPosition(double t) const override;
    double getVelocity(double t) const override;
    double getAcceleration(double t) const override;
    double getTotalTime() const override;
    std::unique_ptr<ITrajectory<double>> mirrored() const override;
    PosVelAcc<double> getValuesAtTime(double tt) const override;
    std::vector<double> getTimeSections() const override;
    double getMaxSpeed() const override;

private:
    BangBangTrajectory1D child_;
};

} // namespace ctrl