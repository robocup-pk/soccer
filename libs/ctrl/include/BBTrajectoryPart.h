#pragma once

namespace ctrl {

/**
 * @brief Part of a bang-bang trajectory with kinematic calculations
 * 
 * Direct C++ port of BBTrajectoryPart.java from TIGERs Mannheim
 * This class represents a single segment of a bang-bang trajectory with constant acceleration.
 */
class BBTrajectoryPart {
public:
    float tEnd{0.0f};  ///< End time of this trajectory part [s]
    float acc{0.0f};   ///< Acceleration during this part [m/s²]
    float v0{0.0f};    ///< Initial velocity of this part [m/s]
    float s0{0.0f};    ///< Initial position of this part [m]

    /**
     * @brief Default constructor
     */
    BBTrajectoryPart() = default;
    
    /**
     * @brief Constructor with parameters
     * @param tEnd End time of this part [s]
     * @param acc Acceleration during this part [m/s²]  
     * @param v0 Initial velocity of this part [m/s]
     * @param s0 Initial position of this part [m]
     */
    BBTrajectoryPart(float tEnd, float acc, float v0, float s0)
        : tEnd(tEnd), acc(acc), v0(v0), s0(s0) {}
    
    /**
     * @brief Get position at relative time t within this segment
     * @param t Time relative to start of this segment [s]
     * @return Position [m]
     */
    double getPosition(double t) const {
        float tf = static_cast<float>(t);
        return static_cast<double>(s0 + (v0 * tf) + (0.5f * acc * tf * tf));
    }
    
    /**
     * @brief Get velocity at relative time t within this segment
     * @param t Time relative to start of this segment [s] 
     * @return Velocity [m/s]
     */
    double getVelocity(double t) const {
        float tf = static_cast<float>(t);
        return static_cast<double>(v0 + (acc * tf));
    }
    
    /**
     * @brief Get acceleration (constant for this segment)
     * @param t Time (unused, acceleration is constant)
     * @return Acceleration [m/s²]
     */
    double getAcceleration(double t) const {
        (void)t; // Suppress unused parameter warning
        return static_cast<double>(acc);
    }
    
    /**
     * @brief Get duration of this segment
     * @return Duration [s]
     */
    double getDuration() const {
        return static_cast<double>(tEnd);
    }
    
    /**
     * @brief Check if this segment is active at the given time
     * @param t Absolute trajectory time [s]
     * @param tStart Start time of this segment [s] 
     * @return True if segment is active at time t
     */
    bool isActive(double t, double tStart) const {
        return t >= tStart && t <= (tStart + static_cast<double>(tEnd));
    }
};

} // namespace ctrl