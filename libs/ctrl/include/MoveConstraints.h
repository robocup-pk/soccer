#pragma once

#include <Eigen/Dense>

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's MoveConstraints.java
 * Movement constraints for robot motion planning
 */
class MoveConstraints {
private:
    double velMax_{1.5};
    double velMaxFast_{3.5}; 
    double accMax_{3.0};
    double accMaxDerived_{3.0};
    double brkMax_{6.0};
    double velMaxW_{10.0};
    double accMaxW_{30.0};
    double jerkMax_{30.0};
    double jerkMaxW_{300.0};
    
    Eigen::Vector2d primaryDirection_{0.0, 0.0};
    bool fastMove_{false};

public:
    MoveConstraints() = default;
    
    // Copy constructor
    MoveConstraints(const MoveConstraints& other) = default;
    MoveConstraints& operator=(const MoveConstraints& other) = default;
    
    // Getters (EXACT copy of Advanced interface)
    double getVelMax() const { return velMax_; }
    double getVelMaxFast() const { return velMaxFast_; }
    double getAccMax() const { return accMax_; }
    double getAccMaxDerived() const { return accMaxDerived_; }
    double getBrkMax() const { return brkMax_; }
    double getVelMaxW() const { return velMaxW_; }
    double getAccMaxW() const { return accMaxW_; }
    double getJerkMax() const { return jerkMax_; }
    double getJerkMaxW() const { return jerkMaxW_; }
    const Eigen::Vector2d& getPrimaryDirection() const { return primaryDirection_; }
    bool isFastMove() const { return fastMove_; }
    
    // Setters (EXACT copy of Advanced interface)
    MoveConstraints& setVelMax(double vel) { velMax_ = vel; return *this; }
    MoveConstraints& setVelMaxFast(double vel) { velMaxFast_ = vel; return *this; }
    MoveConstraints& setAccMax(double acc) { accMax_ = acc; accMaxDerived_ = acc; return *this; }
    MoveConstraints& setBrkMax(double brk) { brkMax_ = brk; return *this; }
    MoveConstraints& setVelMaxW(double velW) { velMaxW_ = velW; return *this; }
    MoveConstraints& setAccMaxW(double accW) { accMaxW_ = accW; return *this; }
    MoveConstraints& setJerkMax(double jerk) { jerkMax_ = jerk; return *this; }
    MoveConstraints& setJerkMaxW(double jerkW) { jerkMaxW_ = jerkW; return *this; }
    MoveConstraints& setPrimaryDirection(const Eigen::Vector2d& dir) { primaryDirection_ = dir; return *this; }
    MoveConstraints& setFastMove(bool fast) { fastMove_ = fast; return *this; }
    
    // Utility methods (EXACT copy of Advanced)
    MoveConstraints limit(const MoveConstraints& limits) const {
        MoveConstraints result = *this;
        result.velMax_ = std::min(velMax_, limits.velMax_);
        result.velMaxFast_ = std::min(velMaxFast_, limits.velMaxFast_);
        result.accMax_ = std::min(accMax_, limits.accMax_);
        result.accMaxDerived_ = std::min(accMaxDerived_, limits.accMaxDerived_);
        result.brkMax_ = std::min(brkMax_, limits.brkMax_);
        result.velMaxW_ = std::min(velMaxW_, limits.velMaxW_);
        result.accMaxW_ = std::min(accMaxW_, limits.accMaxW_);
        result.jerkMax_ = std::min(jerkMax_, limits.jerkMax_);
        result.jerkMaxW_ = std::min(jerkMaxW_, limits.jerkMaxW_);
        return result;
    }
};

} // namespace ctrl