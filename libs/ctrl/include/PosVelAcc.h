#pragma once

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's PosVelAcc.java
 * State class containing position, velocity and acceleration.
 */
template<typename T>
struct PosVelAcc {
    T pos;  // [m]
    T vel;  // [m/s] 
    T acc;  // [m/s^2]
    
    PosVelAcc() = default;
    PosVelAcc(const T& p, const T& v, const T& a) : pos(p), vel(v), acc(a) {}
    
    // Getter methods (like Advanced)
    const T& getPos() const { return pos; }
    const T& getVel() const { return vel; }
    const T& getAcc() const { return acc; }
    
    T& getPos() { return pos; }
    T& getVel() { return vel; }
    T& getAcc() { return acc; }
};

} // namespace ctrl