#pragma once

#include "PosVelAcc.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's ITrajectory.java interface
 * Generic trajectory with position, velocity and acceleration.
 */
template<typename T>
class ITrajectory {
public:
    virtual ~ITrajectory() = default;
    
    /**
     * Get position at time t.
     * @param t time
     * @return position [mm] (Advanced uses mm)
     */
    virtual T getPositionMM(double t) const = 0;
    
    /**
     * Get position at time t.
     * @param t time  
     * @return position [m] (we use meters internally)
     */
    virtual T getPosition(double t) const = 0;
    
    /**
     * Get velocity at a certain time.
     * @param t time
     * @return velocity [m/s]
     */
    virtual T getVelocity(double t) const = 0;
    
    /**
     * Get acceleration at a certain time.
     * @param t time
     * @return acceleration [m/s²]
     */
    virtual T getAcceleration(double t) const = 0;
    
    /**
     * Get total runtime.
     * @return total time for trajectory
     */
    virtual double getTotalTime() const = 0;
    
    /**
     * @param t time [s]
     * @return the next destination, if this trajectory is divided into multiple subtract-pathes
     */
    virtual T getNextDestination(double t) const {
        return getPositionMM(getTotalTime());
    }
    
    /**
     * @return the final position in this trajectory
     */
    virtual T getFinalDestination() const {
        return getPositionMM(getTotalTime());  
    }
    
    /**
     * Get the full state at a certain time.
     * @param tt time
     * @return full state
     */
    virtual PosVelAcc<T> getValuesAtTime(double tt) const {
        return PosVelAcc<T>(getPosition(tt), getVelocity(tt), getAcceleration(tt));
    }
    
    /**
     * Get a list of sections based on `tEnd`.
     * @return the list of sections
     */
    virtual std::vector<double> getTimeSections() const {
        // Default implementation - can be overridden
        return {getTotalTime()};
    }
    
    virtual double getTotalTimeToPrimaryDirection() const {
        return getTotalTime();
    }
    
    virtual double getMaxSpeed() const {
        // Default implementation - should be overridden
        return 0.0;
    }
    
    /**
     * Mirror this trajectory (EXACT copy of Advanced's IMirrorable interface)
     * @return mirrored trajectory
     */
    virtual std::unique_ptr<ITrajectory<T>> mirrored() const {
        // Default implementation - can be overridden
        return nullptr;
    }
};

} // namespace ctrl