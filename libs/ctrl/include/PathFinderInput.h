#pragma once

#include "MoveConstraints.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace ctrl {

// Forward declarations
class IObstacle;

/**
 * @brief EXACT copy of Sumatra's PathFinderInput.java
 * Input data for path finding operations
 */
class PathFinderInput {
public:
    // Builder pattern like Sumatra (forward declaration)
    class PathFinderInputBuilder;
    
private:
    Eigen::Vector2d pos_;
    Eigen::Vector2d vel_;
    Eigen::Vector2d dest_;
    MoveConstraints moveConstraints_;
    std::vector<std::shared_ptr<IObstacle>> obstacles_;
    long timestamp_;
    
    // Private constructor for builder
    PathFinderInput() = default;
    friend class PathFinderInputBuilder;

public:
    
    // Static factory methods (EXACT copy of Sumatra)
    static PathFinderInputBuilder fromBot(const Eigen::Vector3d& botState, const Eigen::Vector3d& botVel);
    
    // Getters (EXACT copy of Sumatra interface)
    const Eigen::Vector2d& getPos() const { return pos_; }
    const Eigen::Vector2d& getVel() const { return vel_; }
    const Eigen::Vector2d& getDest() const { return dest_; }
    const MoveConstraints& getMoveConstraints() const { return moveConstraints_; }
    const std::vector<std::shared_ptr<IObstacle>>& getObstacles() const { return obstacles_; }
    long getTimestamp() const { return timestamp_; }
};

// Builder implementation (must be after PathFinderInput is fully defined)
class PathFinderInput::PathFinderInputBuilder {
private:
    PathFinderInput input_;
    
public:
    PathFinderInputBuilder() = default;
    
    PathFinderInputBuilder& pos(const Eigen::Vector2d& pos) {
        input_.pos_ = pos;
        return *this;
    }
    
    PathFinderInputBuilder& vel(const Eigen::Vector2d& vel) {
        input_.vel_ = vel;
        return *this;
    }
    
    PathFinderInputBuilder& dest(const Eigen::Vector2d& dest) {
        input_.dest_ = dest;
        return *this;
    }
    
    PathFinderInputBuilder& moveConstraints(const MoveConstraints& mc) {
        input_.moveConstraints_ = mc;
        return *this;
    }
    
    PathFinderInputBuilder& obstacles(const std::vector<std::shared_ptr<IObstacle>>& obs) {
        input_.obstacles_ = obs;
        return *this;
    }
    
    PathFinderInputBuilder& timestamp(long ts) {
        input_.timestamp_ = ts;
        return *this;
    }
    
    PathFinderInput build() {
        return input_;
    }
};

// Inline implementation of static factory method
inline PathFinderInput::PathFinderInputBuilder PathFinderInput::fromBot(const Eigen::Vector3d& botState, const Eigen::Vector3d& botVel) {
    PathFinderInputBuilder builder;
    return builder.pos(botState.head<2>()).vel(botVel.head<2>());
}

} // namespace ctrl