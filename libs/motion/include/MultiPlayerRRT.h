#ifndef MULTIPLAYER_RRT_H
#define MULTIPLAYER_RRT_H

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "GameObject.h"
#include "path_trajectory.h"

namespace vis {

/**
 * RRTPlayerController - Individual player RRT-based path planning
 * Manages collision-free path planning and movement for a single player
 */
class RRTPlayerController {
public:
    RRTPlayerController(const std::string& player_name);
    ~RRTPlayerController();

    // Configuration
    void SetMaxSpeed(double speed) { max_speed_ = speed; }
    void SetMaxAcceleration(double accel) { max_acceleration_ = accel; }
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    
    // Getters
    const std::string& GetPlayerName() const { return player_name_; }
    bool IsEnabled() const { return enabled_; }
    
    // Update methods
    void UpdatePlayer(std::map<std::string, GameObject>& game_objects, 
                     const std::vector<std::string>& opponent_names,
                     const std::string& ball_name, float dt);

private:
    // Core components
    std::unique_ptr<algos::PathTrajectory> path_finder_;
    
    // Configuration
    std::string player_name_;
    double max_speed_;
    double max_acceleration_;
    bool enabled_;
    
    // Trajectory state
    std::vector<algos::TrajectoryPoint> current_trajectory_;
    int current_trajectory_index_;
    float trajectory_update_timer_;
    
    // Update intervals
    static constexpr float TRAJECTORY_UPDATE_INTERVAL = 0.3f; // 300ms
    
    // Helper methods
    void UpdateTrajectory(const std::map<std::string, GameObject>& game_objects,
                         const std::vector<std::string>& opponent_names,
                         const std::string& ball_name);
    void FollowTrajectory(GameObject& player, float dt);
    
    // Conversion utilities
    algos::Vector2D ConvertToVector2D(const glm::vec2& vec) const;
    glm::vec2 ConvertToGLMVec2(const algos::Vector2D& vec) const;
    
    // Game state analysis
    std::vector<algos::Obstacle> GetObstacles(const std::map<std::string, GameObject>& game_objects,
                                             const std::vector<std::string>& opponent_names) const;
    bool ShouldPursue(const GameObject& player, const GameObject& ball,
                     const std::vector<std::string>& opponent_names,
                     const std::map<std::string, GameObject>& game_objects) const;
};

/**
 * MultiPlayerRRT - Multi-Player RRT-based Path Planning System
 * 
 * Controls multiple players (robot0, robot2, robot3) for RRT-based navigation:
 * - Each player uses independent RRT path planning to reach the ball
 * - Considers other players as obstacles for collision-free navigation
 * - Coordinate team-based movement with role assignment
 */
class MultiPlayerRRT {
public:
    MultiPlayerRRT();
    ~MultiPlayerRRT();

    // Configuration methods
    void AddControlledPlayer(const std::string& player_name);
    void RemoveControlledPlayer(const std::string& player_name);
    void SetOpponents(const std::vector<std::string>& opponent_names);
    void SetBallName(const std::string& name) { ball_name_ = name; }
    void SetMaxSpeed(double speed);
    void SetMaxAcceleration(double accel);
    void SetAutoMode(bool enabled) { auto_mode_enabled_ = enabled; }
    
    // Player-specific configuration
    void SetPlayerEnabled(const std::string& player_name, bool enabled);
    void SetPlayerMaxSpeed(const std::string& player_name, double speed);
    
    // Main update method
    void UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt);
    
    // Toggle methods
    void ToggleAutoMode() { auto_mode_enabled_ = !auto_mode_enabled_; }
    void TogglePlayer(const std::string& player_name);
    bool IsAutoModeEnabled() const { return auto_mode_enabled_; }
    
    // Status methods
    std::vector<std::string> GetControlledPlayers() const;
    std::vector<std::string> GetEnabledPlayers() const;
    void PrintStatus() const;

private:
    // Player controllers
    std::map<std::string, std::unique_ptr<RRTPlayerController>> player_controllers_;
    
    // Configuration
    std::vector<std::string> opponent_names_;
    std::string ball_name_;
    bool auto_mode_enabled_;
    
    // Team coordination
    void CoordinateTeamMovement(std::map<std::string, GameObject>& game_objects);
    std::string SelectBestPlayerForBall(const std::map<std::string, GameObject>& game_objects) const;
    void AssignPlayerRoles(const std::map<std::string, GameObject>& game_objects);
    float CalculatePathLength(const GameObject& player, const GameObject& ball,
                             const std::vector<std::string>& opponent_names,
                             const std::map<std::string, GameObject>& game_objects) const;
};

} // namespace vis

#endif // MULTIPLAYER_RRT_H