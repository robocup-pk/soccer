#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "PosVelAcc.h"

namespace ctrl {

/**
 * @brief Generic trajectory with position, velocity and acceleration.
 * 
 * Direct C++ port of ITrajectory.java from TIGERs Mannheim
 * 
 * @tparam T The type of the state vector (e.g., double for 1D, Eigen::Vector2d for 2D).
 */
template <typename T>
class ITrajectory {
public:
    virtual ~ITrajectory() = default;

    /**
     * @brief Get position at time t [mm]
     * @param t time [s]
     * @return position [mm]
     */
    virtual T getPositionMM(double t) const = 0;

    /**
     * @brief Get position at time t [m]
     * @param t time [s]
     * @return position [m]
     */
    virtual T getPosition(double t) const = 0;

    /**
     * @brief Get velocity at a certain time
     * @param t time [s]
     * @return velocity [m/s]
     */
    virtual T getVelocity(double t) const = 0;

    /**
     * @brief Get acceleration at a certain time
     * @param t time [s] 
     * @return acceleration [m/s²]
     */
    virtual T getAcceleration(double t) const = 0;

    /**
     * @brief Get total runtime
     * @return total time for trajectory [s]
     */
    virtual double getTotalTime() const = 0;

    /**
     * @brief Get the next destination, if this trajectory is divided into multiple sub-paths
     * @param t time [s]
     * @return the next destination
     */
    virtual T getNextDestination(double t) const {
        return getPositionMM(getTotalTime());
    }

    /**
     * @brief Get the final position in this trajectory
     * @return the final position
     */
    virtual T getFinalDestination() const {
        return getPositionMM(getTotalTime());
    }

    /**
     * @brief Get the full state at a certain time
     * @param t time [s]
     * @return full state (position, velocity, acceleration)
     */
    virtual PosVelAcc<T> getValuesAtTime(double t) const {
        throw std::runtime_error("Not implemented");
    }

    /**
     * @brief Get a list of sections based on tEnd
     * @return the list of sections
     */
    virtual std::vector<double> getTimeSections() const {
        throw std::runtime_error("Not implemented");
    }

    /**
     * @brief Get total time to primary direction
     * @return total time to primary direction [s]
     */
    virtual double getTotalTimeToPrimaryDirection() const {
        return getTotalTime();
    }

    /**
     * @brief Get maximum speed in trajectory
     * @return maximum speed [m/s]
     */
    virtual double getMaxSpeed() const {
        throw std::runtime_error("Not implemented");
    }

    /**
     * @brief Create a mirrored version of this trajectory
     * @return mirrored trajectory
     */
    virtual std::unique_ptr<ITrajectory<T>> mirrored() const = 0;
};

} // namespace ctrl