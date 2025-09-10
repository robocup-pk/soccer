#pragma once

#include <Eigen/Dense>

// Corresponds to PlanarCurveState.java

/**
 * @brief Represents the state of a planar curve at a specific time.
 */
class PlanarCurveState {
public:
    /** Position in [mm] */
    Eigen::Vector2d pos;
    /** Velocity in [mm/s] */
    Eigen::Vector2d vel;
    /** Acceleration in [mm/s^2] */
    Eigen::Vector2d acc;

    /**
     * @brief Construct a new Planar Curve State object
     * @param pos Position
     * @param vel Velocity
     * @param acc Acceleration
     */
    PlanarCurveState(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& acc)
        : pos(pos), vel(vel), acc(acc) {}
};
