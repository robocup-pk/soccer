#pragma once

#include <Eigen/Dense>
#include <functional>
#include <optional>

// Corresponds to DestinationForTimedPositionCalc.java

/**
 * @brief Generates virtual destinations such that a position will be reached in a specific time.
 * This is also known as "Overshooting".
 */
class DestinationForTimedPositionCalc {
private:
    /**
     * @brief A simple struct to hold a 1D position and the time to reach it.
     */
    struct TimedPos1D {
        float pos;
        float time;
    };

    /**
     * @brief Internal 2D destination calculation with a binary search for synchronization.
     */
    Eigen::Vector2d destinationForBangBang2D(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        float vMax,
        float aMax,
        float targetTime,
        const std::function<float(float)>& alphaFn
    );

    /**
     * @brief Core 1D calculation to find the virtual destination and time.
     */
    TimedPos1D getTimedPos1D(float s, float v0, float vMax, float aMax, float tt);
    
    /**
     * @brief Calculates the slowest possible time to directly reach the target position.
     */
    float calcSlowestDirectTime(float s, float v0, float aMax);

    /**
     * @brief Calculates the virtual destination for a direct hit.
     */
    TimedPos1D calcFastestDirect(float s, float v0, float v1Max, float aMax, float tt);

    /**
     * @brief Helper for calcFastestDirect, handling trapezoidal profiles.
     */
    std::optional<TimedPos1D> calcFastestDirectTrapezoidal(float s, float v0, float v1Max, float aMax, float aDec, float tt);
    
    /**
     * @brief Helper for calcFastestDirect, handling triangular profiles.
     */
    TimedPos1D calcFastestDirectTriangular(float s, float v0, float v1Max, float aMax, float aDec, float tt);
    

public:
    /**
     * @brief Calculates a virtual destination for a synchronized 2D bang-bang trajectory.
     * @param s0 [m] Start position.
     * @param s1 [m] Target position.
     * @param v0 [m/s] Initial velocity.
     * @param vMax [m/s] Maximum velocity.
     * @param aMax [m/s²] Maximum acceleration.
     * @param targetTime [s] Target time.
     * @return [m] The calculated virtual destination.
     */
    Eigen::Vector2d destinationForBangBang2dSync(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vMax,
        double aMax,
        double targetTime
    );
    
    /**
     * @brief Calculates a virtual destination for an asynchronous 2D bang-bang trajectory.
     * @param s0 [m] Start position.
     * @param s1 [m] Target position.
     * @param v0 [m/s] Initial velocity.
     * @param vMax [m/s] Maximum velocity.
     * @param aMax [m/s²] Maximum acceleration.
     * @param targetTime [s] Target time.
     * @param primaryDirection The primary direction of movement.
     * @return [m] The calculated virtual destination.
     */
    Eigen::Vector2d destinationForBangBang2dAsync(
        const Eigen::Vector2d& s0,
        const Eigen::Vector2d& s1,
        const Eigen::Vector2d& v0,
        double vMax,
        double aMax,
        double targetTime,
        const Eigen::Vector2d& primaryDirection
    );
};
