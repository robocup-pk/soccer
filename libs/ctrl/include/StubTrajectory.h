#pragma once

#include "ITrajectory.h"
#include "PosVelAcc.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

/**
 * @brief Static trajectory for non-moving targets.
 * 
 * Direct C++ port of StubTrajectory.java from TIGERs Mannheim.
 * This trajectory represents a stationary robot or target that doesn't move.
 * All positions return the same static value, all velocities and accelerations are zero.
 */
template <typename T>
class StubTrajectory : public ITrajectory<T> {
public:
    /**
     * @brief Constructor with static position
     * @param static_position The constant position value
     */
    explicit StubTrajectory(const T& static_position) 
        : static_position_(static_position) {}
    
    // --- ITrajectory Interface ---
    T getPositionMM(double t) const override {
        (void)t; // Suppress unused parameter warning
        if constexpr (std::is_same_v<T, double>) {
            return static_position_ * 1000.0; // Convert m to mm
        } else {
            return static_position_ * 1000.0; // Vector scaling
        }
    }
    
    T getPosition(double t) const override {
        (void)t; // Suppress unused parameter warning
        return static_position_;
    }
    
    T getVelocity(double t) const override {
        (void)t; // Suppress unused parameter warning
        if constexpr (std::is_same_v<T, double>) {
            return 0.0;
        } else {
            return T::Zero();
        }
    }
    
    T getAcceleration(double t) const override {
        (void)t; // Suppress unused parameter warning
        if constexpr (std::is_same_v<T, double>) {
            return 0.0;
        } else {
            return T::Zero();
        }
    }
    
    double getTotalTime() const override {
        return 0.0; // Static trajectory has no duration
    }
    
    std::unique_ptr<ITrajectory<T>> mirrored() const override {
        if constexpr (std::is_same_v<T, double>) {
            return std::make_unique<StubTrajectory<T>>(-static_position_);
        } else {
            T mirrored_pos = static_position_;
            mirrored_pos.x() *= -1; // Mirror X coordinate
            return std::make_unique<StubTrajectory<T>>(mirrored_pos);
        }
    }
    
    PosVelAcc<T> getValuesAtTime(double t) const override {
        (void)t; // Suppress unused parameter warning
        PosVelAcc<T> result;
        result.pos = static_position_;
        
        if constexpr (std::is_same_v<T, double>) {
            result.vel = 0.0;
            result.acc = 0.0;
        } else {
            result.vel = T::Zero();
            result.acc = T::Zero();
        }
        
        return result;
    }
    
    std::vector<double> getTimeSections() const override {
        return {0.0}; // Only one time section at t=0
    }
    
    double getMaxSpeed() const override {
        return 0.0; // No movement, no speed
    }
    
    T getNextDestination(double t) const override {
        (void)t; // Suppress unused parameter warning
        return getPositionMM(0.0);
    }
    
    T getFinalDestination() const override {
        return getPositionMM(0.0);
    }
    
    double getTotalTimeToPrimaryDirection() const override {
        return 0.0; // No movement, no time
    }

private:
    T static_position_; ///< The constant position value
};

// Common typedefs for convenience
using StubTrajectory1D = StubTrajectory<double>;
using StubTrajectory2D = StubTrajectory<Eigen::Vector2d>;
using StubTrajectory3D = StubTrajectory<Eigen::Vector3d>;

} // namespace ctrl