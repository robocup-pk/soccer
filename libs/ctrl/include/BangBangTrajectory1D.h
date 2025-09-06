#pragma once

#include "ITrajectory.h"
#include "BBTrajectoryPart.h"
#include <memory>
#include <vector>
#include <cmath>

namespace ctrl {

/**
 * @brief Bang Bang Trajectory for one dimension.
 * 
 * Direct C++ port of BangBangTrajectory1D.java from TIGERs Mannheim
 */
class BangBangTrajectory1D : public ITrajectory<double> {
public:
    static const int MAX_PARTS = 3;
    
    BangBangTrajectory1D();
    
    // --- ITrajectory Interface ---
    double getPosition(double tt) const override;
    double getPositionMM(double t) const override;
    double getVelocity(double tt) const override;
    double getAcceleration(double tt) const override;
    double getTotalTime() const override;
    std::unique_ptr<ITrajectory<double>> mirrored() const override;
    PosVelAcc<double> getValuesAtTime(double tt) const override;
    std::vector<double> getTimeSections() const override;
    double getMaxSpeed() const override;
    
    /**
     * @brief Generate trajectory from parameters
     * @param initialPos Initial position [m]
     * @param finalPos Final position [m] 
     * @param initialVel Initial velocity [m/s]
     * @param maxVel Maximum velocity [m/s]
     * @param maxAcc Maximum acceleration [m/s²]
     * @return Reference to this object for chaining
     */
    BangBangTrajectory1D& generate(float initialPos, float finalPos, float initialVel, 
                                   float maxVel, float maxAcc);

    // Public for friend access (like Sumatra package-private)
    BBTrajectoryPart parts[MAX_PARTS];
    int numParts{0};

private:
    int findPartIdx(double t) const;
    BBTrajectoryPart findPart(double t) const;
    
    // Trajectory calculation methods (direct port from Sumatra)
    float velChangeToZero(float s0, float v0, float aMax) const;
    float velTriToZero(float s0, float v0, float v1, float aMax) const;
    void calcTri(float s0, float v0, float s2, float a);
    void calcTrapz(float s0, float v0, float v1, float s3, float aMax);
};

} // namespace ctrl