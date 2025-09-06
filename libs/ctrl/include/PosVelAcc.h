#pragma once

namespace ctrl {

/**
 * @brief State class containing position, velocity and acceleration.
 * 
 * Direct C++ port of PosVelAcc.java from TIGERs Mannheim
 * 
 * @tparam T The type (e.g., double for 1D, Eigen::Vector2d for 2D)
 */
template<typename T>
class PosVelAcc {
public:
    /// Default constructor
    PosVelAcc() = default;
    
    /**
     * @brief Constructor
     * @param pos Position [m]
     * @param vel Velocity [m/s]  
     * @param acc Acceleration [m/s²]
     */
    PosVelAcc(const T& pos, const T& vel, const T& acc) 
        : pos_(pos), vel_(vel), acc_(acc) {}
    
    /// Get position [m]
    const T& getPos() const { return pos_; }
    
    /// Get velocity [m/s]
    const T& getVel() const { return vel_; }
    
    /// Get acceleration [m/s²]
    const T& getAcc() const { return acc_; }
    
    /// Set position [m]
    void setPos(const T& pos) { pos_ = pos; }
    
    /// Set velocity [m/s]
    void setVel(const T& vel) { vel_ = vel; }
    
    /// Set acceleration [m/s²]
    void setAcc(const T& acc) { acc_ = acc; }
    
    // Public members for direct access (matching original usage pattern)
    T pos{};  ///< Position [m]
    T vel{};  ///< Velocity [m/s]
    T acc{};  ///< Acceleration [m/s²]

private:
    T pos_;  ///< Position [m]
    T vel_;  ///< Velocity [m/s]
    T acc_;  ///< Acceleration [m/s²]
};

} // namespace ctrl