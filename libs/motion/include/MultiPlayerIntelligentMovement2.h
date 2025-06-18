#ifndef MULTIPLAYER_INTELLIGENT_MOVEMENT2_H
#define MULTIPLAYER_INTELLIGENT_MOVEMENT2_H

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "GameObject.h"
#include "ball_intercept.h"

namespace vis {

/**
 * PlayerController - Individual player control system
 * Manages ball interception and movement for a single player
 */
class PlayerController {
public:
    PlayerController(const std::string& player_name, 
                    const algos::BallInterceptor::InterceptParams& params);
    ~PlayerController();

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
    std::unique_ptr<algos::BallInterceptAction> ball_intercept_action_;
    std::unique_ptr<algos::BallInterceptor> ball_interceptor_;
    
    // Configuration
    std::string player_name_;
    double max_speed_;
    double max_acceleration_;
    bool enabled_;
    
    // Action state
    algos::BallInterceptAction::Action current_action_;
    float action_timer_;
    bool has_active_action_;
    float intercept_update_timer_;
    
    // Update intervals
    static constexpr float INTERCEPT_UPDATE_INTERVAL = 0.2f; // 200ms
    
    // Helper methods
    void UpdateInterception(const std::map<std::string, GameObject>& game_objects,
                           const std::vector<std::string>& opponent_names,
                           const std::string& ball_name);
    void ExecuteCurrentAction(GameObject& player, float dt);
    
    // Conversion utilities
    algos::SimplePlayer ConvertToSimplePlayer(const GameObject& obj) const;
    algos::SimpleBall ConvertToSimpleBall(const GameObject& obj) const;
    glm::vec2 ConvertToGLMVec2(const algos::Vector2D& vec) const;
    algos::Vector2D ConvertToVector2D(const glm::vec2& vec) const;
    
    // Game state analysis
    std::vector<algos::SimplePlayer> GetOpponents(const std::map<std::string, GameObject>& game_objects,
                                                 const std::vector<std::string>& opponent_names) const;
    bool ShouldPursue(const algos::SimplePlayer& player, const algos::SimpleBall& ball, 
                     const std::vector<algos::SimplePlayer>& opponents) const;
    
    // Action execution helpers
    void ApplyDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
    void ApplyTurnAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
    void ApplyTurnAndDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
};

/**
 * MultiPlayerIntelligentMovement2 - Multi-Player Ball Interception System
 * 
 * Controls multiple players (robot0, robot2, robot3) for strategic ball interception:
 * - Each player uses independent ball interception strategy
 * - Considers other players as opponents or teammates
 * - Coordinate team-based ball pursuit
 */
class MultiPlayerIntelligentMovement2 {
public:
    MultiPlayerIntelligentMovement2();
    ~MultiPlayerIntelligentMovement2();

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
    std::map<std::string, std::unique_ptr<PlayerController>> player_controllers_;
    
    // Configuration
    std::vector<std::string> opponent_names_;
    std::string ball_name_;
    bool auto_mode_enabled_;
    
    // Team coordination
    void CoordinateTeamMovement(std::map<std::string, GameObject>& game_objects);
    std::string SelectBestPlayerForBall(const std::map<std::string, GameObject>& game_objects) const;
    void AssignPlayerRoles(const std::map<std::string, GameObject>& game_objects);
};

} // namespace vis

#endif // MULTIPLAYER_INTELLIGENT_MOVEMENT2_H