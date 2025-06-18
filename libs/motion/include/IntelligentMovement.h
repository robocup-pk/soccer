#ifndef INTELLIGENT_MOVEMENT_H
#define INTELLIGENT_MOVEMENT_H

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "GameObject.h"
#include "path_trajectory.h"

namespace vis {

class IntelligentMovement {
public:
    IntelligentMovement();
    ~IntelligentMovement();
    
    // Movement update - now uses trajectory algorithm
    void UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt);
    
    // Mode control
    void SetAutoMode(bool enabled) { auto_mode_enabled_ = enabled; }
    bool IsAutoModeEnabled() const { return auto_mode_enabled_; }
    
    // Configuration
    void SetBallName(const std::string& ball_name) { ball_name_ = ball_name; }
    void SetPlayerName(const std::string& player_name) { player_name_ = player_name; }
    void SetMaxSpeed(double speed) { max_speed_ = speed; }
    void SetMaxAcceleration(double accel) { max_acceleration_ = accel; }
    
private:
    std::unique_ptr<algos::PathTrajectory> path_finder_;
    bool auto_mode_enabled_;
    std::string ball_name_;
    std::string player_name_;
    double max_speed_;
    double max_acceleration_;
    
    // Trajectory tracking
    std::vector<algos::TrajectoryPoint> current_trajectory_;
    size_t current_trajectory_index_;
    float trajectory_update_timer_;
    static constexpr float TRAJECTORY_UPDATE_INTERVAL = 0.3f; // Update every 300ms for better obstacle tracking
    
    // Utility methods
    algos::Vector2D ConvertToVector2D(const glm::vec2& vec);
    glm::vec2 ConvertToGLMVec2(const algos::Vector2D& vec);
    std::vector<algos::Obstacle> GetObstacles(const std::map<std::string, GameObject>& game_objects);
    
    // Movement execution
    void UpdateTrajectory(const std::map<std::string, GameObject>& game_objects);
    void FollowTrajectory(GameObject& player, float dt);
};

}

#endif // INTELLIGENT_MOVEMENT_H