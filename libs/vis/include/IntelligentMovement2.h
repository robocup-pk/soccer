#ifndef INTELLIGENT_MOVEMENT2_H
#define INTELLIGENT_MOVEMENT2_H

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "GameObject.h"
#include "ball_intercept.h"

namespace vis {

/**
 * IntelligentMovement2 - Ball Interception Based Movement System
 * 
 * Uses algos::BallInterceptor for strategic ball interception with:
 * - Robot0 as source player (controlled)
 * - Robot1 as opponent 
 * - Ball as target
 * - Considers 100x100 pixel robot sizes
 */
class IntelligentMovement2 {
public:
    IntelligentMovement2();
    ~IntelligentMovement2();

    // Configuration methods
    void SetPlayerName(const std::string& name) { player_name_ = name; }
    void SetOpponentName(const std::string& name) { opponent_name_ = name; }
    void SetBallName(const std::string& name) { ball_name_ = name; }
    void SetMaxSpeed(double speed) { max_speed_ = speed; }
    void SetMaxAcceleration(double accel) { max_acceleration_ = accel; }
    void SetAutoMode(bool enabled) { auto_mode_enabled_ = enabled; }
    
    // Main update method
    void UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt);
    
    // Toggle auto mode
    void ToggleAutoMode() { auto_mode_enabled_ = !auto_mode_enabled_; }
    bool IsAutoModeEnabled() const { return auto_mode_enabled_; }

private:
    // Core components
    std::unique_ptr<algos::BallInterceptAction> ball_intercept_action_;
    std::unique_ptr<algos::BallInterceptor> ball_interceptor_;
    
    // Configuration
    std::string player_name_;
    std::string opponent_name_;
    std::string ball_name_;
    double max_speed_;
    double max_acceleration_;
    bool auto_mode_enabled_;
    
    // Action state
    algos::BallInterceptAction::Action current_action_;
    float action_timer_;
    bool has_active_action_;
    
    // Update intervals
    static constexpr float INTERCEPT_UPDATE_INTERVAL = 0.2f; // 200ms
    float intercept_update_timer_;
    
    // Helper methods
    void UpdateInterception(const std::map<std::string, GameObject>& game_objects);
    void ExecuteCurrentAction(GameObject& player, float dt);
    
    // Conversion utilities
    algos::SimplePlayer ConvertToSimplePlayer(const GameObject& obj) const;
    algos::SimpleBall ConvertToSimpleBall(const GameObject& obj) const;
    glm::vec2 ConvertToGLMVec2(const algos::Vector2D& vec) const;
    algos::Vector2D ConvertToVector2D(const glm::vec2& vec) const;
    
    // Game state analysis
    std::vector<algos::SimplePlayer> GetOpponents(const std::map<std::string, GameObject>& game_objects) const;
    bool ShouldPursue(const algos::SimplePlayer& player, const algos::SimpleBall& ball, 
                     const std::vector<algos::SimplePlayer>& opponents) const;
    
    // Action execution helpers
    void ApplyDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
    void ApplyTurnAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
    void ApplyTurnAndDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt);
};

} // namespace vis

#endif // INTELLIGENT_MOVEMENT2_H