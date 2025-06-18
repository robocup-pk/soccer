#include "MultiPlayerIntelligentMovement2.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace vis {

// ============================================================================
// PlayerController Implementation
// ============================================================================

PlayerController::PlayerController(const std::string& player_name, 
                                 const algos::BallInterceptor::InterceptParams& params)
    : player_name_(player_name),
      max_speed_(90.0),
      max_acceleration_(45.0),
      enabled_(true),
      action_timer_(0.0f),
      has_active_action_(false),
      intercept_update_timer_(0.0f) {
    
    // Create ball interceptor and action with consistent parameters
    ball_interceptor_ = std::make_unique<algos::BallInterceptor>(params);
    ball_intercept_action_ = std::make_unique<algos::BallInterceptAction>(params);
    
    std::cout << "[PlayerController] Created controller for " << player_name_ 
              << " with collision radius: " << params.collision_radius << std::endl;
}

PlayerController::~PlayerController() = default;

void PlayerController::UpdatePlayer(std::map<std::string, GameObject>& game_objects, 
                                   const std::vector<std::string>& opponent_names,
                                   const std::string& ball_name, float dt) {
    if (!enabled_) return;
    
    // Find the player we're controlling
    auto player_it = game_objects.find(player_name_);
    if (player_it == game_objects.end()) {
        std::cerr << "[PlayerController] Player '" << player_name_ << "' not found" << std::endl;
        return;
    }
    
    // Find the ball
    auto ball_it = game_objects.find(ball_name);
    if (ball_it == game_objects.end()) {
        std::cerr << "[PlayerController] Ball '" << ball_name << "' not found" << std::endl;
        return;
    }
    
    GameObject& player = player_it->second;
    
    // Update interception strategy periodically
    intercept_update_timer_ += dt;
    if (intercept_update_timer_ >= INTERCEPT_UPDATE_INTERVAL || !has_active_action_) {
        intercept_update_timer_ = 0.0f;
        UpdateInterception(game_objects, opponent_names, ball_name);
    }
    
    // Execute current action
    if (has_active_action_) {
        ExecuteCurrentAction(player, dt);
    }
}

void PlayerController::UpdateInterception(const std::map<std::string, GameObject>& game_objects,
                                         const std::vector<std::string>& opponent_names,
                                         const std::string& ball_name) {
    auto player_it = game_objects.find(player_name_);
    auto ball_it = game_objects.find(ball_name);
    
    if (player_it == game_objects.end() || ball_it == game_objects.end()) {
        return;
    }
    
    const GameObject& player = player_it->second;
    const GameObject& ball = ball_it->second;
    
    // Convert to algorithm format
    algos::SimplePlayer simple_player = ConvertToSimplePlayer(player);
    algos::SimpleBall simple_ball = ConvertToSimpleBall(ball);
    std::vector<algos::SimplePlayer> opponents = GetOpponents(game_objects, opponent_names);
    
    std::cout << "[" << player_name_ << "] === BALL INTERCEPTION ANALYSIS ===" << std::endl;
    std::cout << "  Player at: (" << simple_player.x << ", " << simple_player.y 
              << ") angle: " << simple_player.angle << std::endl;
    std::cout << "  Ball at: (" << simple_ball.x << ", " << simple_ball.y 
              << ") vel: (" << simple_ball.vx << ", " << simple_ball.vy << ")" << std::endl;
    std::cout << "  Opponents: " << opponents.size() << std::endl;
    
    // Check if we should pursue the ball
    if (ShouldPursue(simple_player, simple_ball, opponents)) {
        std::cout << "  Decision: PURSUE BALL" << std::endl;
        
        // Get the best interception action
        current_action_ = ball_intercept_action_->execute(simple_player, simple_ball, opponents);
        has_active_action_ = true;
        action_timer_ = 0.0f;
        
        // Log the action decision
        std::string action_name;
        switch (current_action_.type) {
            case algos::BallInterceptAction::DASH:
                action_name = "DASH";
                break;
            case algos::BallInterceptAction::TURN:
                action_name = "TURN";
                break;
            case algos::BallInterceptAction::TURN_AND_DASH:
                action_name = "TURN_AND_DASH";
                break;
            case algos::BallInterceptAction::WAIT:
                action_name = "WAIT";
                break;
        }
        
        std::cout << "  Action: " << action_name 
                  << " (power: " << current_action_.power 
                  << ", angle: " << current_action_.angle << ")" << std::endl;
        
        // Convert to pixel coordinates to see actual target
        glm::vec2 pixel_target = ConvertToGLMVec2(current_action_.target_position);
        std::cout << "  Target (pixels): (" << pixel_target.x 
                  << ", " << pixel_target.y << ")" << std::endl;
    } else {
        std::cout << "  Decision: WAIT (not optimal to pursue)" << std::endl;
        current_action_ = algos::BallInterceptAction::Action(algos::BallInterceptAction::WAIT);
        has_active_action_ = true;
    }
}

void PlayerController::ExecuteCurrentAction(GameObject& player, float dt) {
    if (!has_active_action_) return;
    
    action_timer_ += dt;
    
    switch (current_action_.type) {
        case algos::BallInterceptAction::DASH:
            ApplyDashAction(player, current_action_, dt);
            break;
        case algos::BallInterceptAction::TURN:
            ApplyTurnAction(player, current_action_, dt);
            break;
        case algos::BallInterceptAction::TURN_AND_DASH:
            ApplyTurnAndDashAction(player, current_action_, dt);
            break;
        case algos::BallInterceptAction::WAIT:
            player.velocity = glm::vec2(0.0f, 0.0f);
            break;
    }
}

void PlayerController::ApplyDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
    glm::vec2 current_pos = player.GetCenterPosition();
    glm::vec2 target_pos = ConvertToGLMVec2(action.target_position);
    
    glm::vec2 direction = target_pos - current_pos;
    float distance = glm::length(direction);
    
    if (distance > 2.0f) {
        direction = glm::normalize(direction);
        
        // Apply velocity with power scaling
        float effective_speed = static_cast<float>(max_speed_ * action.power / 100.0);
        player.velocity = direction * effective_speed;
        
        std::cout << "[" << player_name_ << "] DASH to (" << target_pos.x << ", " << target_pos.y 
                  << ") speed: " << effective_speed << std::endl;
    } else {
        player.velocity = glm::vec2(0.0f, 0.0f);
        std::cout << "[" << player_name_ << "] Reached target!" << std::endl;
    }
}

void PlayerController::ApplyTurnAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
    glm::vec2 current_pos = player.GetCenterPosition();
    glm::vec2 target_pos = ConvertToGLMVec2(action.target_position);
    
    glm::vec2 direction = target_pos - current_pos;
    float distance = glm::length(direction);
    
    if (distance > 2.0f) {
        direction = glm::normalize(direction);
        
        // Apply movement with power scaling (TURN with movement)
        float effective_speed = static_cast<float>(max_speed_ * action.power / 100.0);
        player.velocity = direction * effective_speed;
        
        std::cout << "[" << player_name_ << "] TURN to (" << target_pos.x << ", " << target_pos.y 
                  << ") angle: " << action.angle << " speed: " << effective_speed << std::endl;
    } else {
        player.velocity = glm::vec2(0.0f, 0.0f);
        std::cout << "[" << player_name_ << "] TURN completed - reached target!" << std::endl;
    }
}

void PlayerController::ApplyTurnAndDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
    // Combine turn and dash - apply both direction and movement
    glm::vec2 direction = ConvertToGLMVec2(action.direction);
    if (glm::length(direction) > 0.01f) {
        direction = glm::normalize(direction);
        float effective_speed = static_cast<float>(max_speed_ * action.power / 100.0);
        player.velocity = direction * effective_speed;
        
        std::cout << "[" << player_name_ << "] TURN_AND_DASH direction: (" 
                  << direction.x << ", " << direction.y << ") speed: " << effective_speed << std::endl;
    }
}

// Conversion utility methods (same as IntelligentMovement2)
algos::SimplePlayer PlayerController::ConvertToSimplePlayer(const GameObject& obj) const {
    glm::vec2 pos = obj.GetCenterPosition();
    
    // Normalize coordinates to algorithm scale (divide by 100 for smaller numbers)
    double norm_x = pos.x / 100.0;
    double norm_y = pos.y / 100.0;
    
    // Calculate angle from velocity direction (simple approximation)
    double angle = 0.0;
    if (glm::length(obj.velocity) > 0.1f) {
        angle = atan2(obj.velocity.y, obj.velocity.x);
    }
    
    return algos::SimplePlayer(norm_x, norm_y, angle);
}

algos::SimpleBall PlayerController::ConvertToSimpleBall(const GameObject& obj) const {
    glm::vec2 pos = obj.GetCenterPosition();
    
    // Normalize coordinates and velocity to algorithm scale
    double norm_x = pos.x / 100.0;
    double norm_y = pos.y / 100.0;
    double norm_vx = obj.velocity.x / 100.0;
    double norm_vy = obj.velocity.y / 100.0;
    
    return algos::SimpleBall(norm_x, norm_y, norm_vx, norm_vy);
}

glm::vec2 PlayerController::ConvertToGLMVec2(const algos::Vector2D& vec) const {
    // Scale back up from normalized coordinates
    return glm::vec2(vec.x * 100.0, vec.y * 100.0);
}

algos::Vector2D PlayerController::ConvertToVector2D(const glm::vec2& vec) const {
    // Scale down to normalized coordinates
    return algos::Vector2D(vec.x / 100.0, vec.y / 100.0);
}

std::vector<algos::SimplePlayer> PlayerController::GetOpponents(const std::map<std::string, GameObject>& game_objects,
                                                               const std::vector<std::string>& opponent_names) const {
    std::vector<algos::SimplePlayer> opponents;
    
    for (const std::string& opponent_name : opponent_names) {
        auto opponent_it = game_objects.find(opponent_name);
        if (opponent_it != game_objects.end()) {
            opponents.push_back(ConvertToSimplePlayer(opponent_it->second));
        }
    }
    
    return opponents;
}

bool PlayerController::ShouldPursue(const algos::SimplePlayer& player, const algos::SimpleBall& ball, 
                                   const std::vector<algos::SimplePlayer>& opponents) const {
    // Calculate minimum steps to intercept for player
    int self_min_step = ball_intercept_action_->calculateMinInterceptSteps(player, ball);
    
    // Calculate minimum steps for opponents
    int opponent_min_step = 1000; // Default high value
    if (!opponents.empty()) {
        for (const auto& opponent : opponents) {
            int steps = ball_intercept_action_->calculateMinInterceptSteps(opponent, ball);
            opponent_min_step = std::min(opponent_min_step, steps);
        }
    }
    
    // Use shouldChase logic from BallInterceptAction
    bool should_chase = ball_intercept_action_->shouldChase(player, ball, self_min_step, 1000, opponent_min_step);
    
    std::cout << "[" << player_name_ << "] Interception analysis:" << std::endl;
    std::cout << "  - Self min steps: " << self_min_step << std::endl;
    std::cout << "  - Opponent min steps: " << opponent_min_step << std::endl;
    std::cout << "  - Should chase: " << (should_chase ? "YES" : "NO") << std::endl;
    
    return should_chase;
}

// ============================================================================
// MultiPlayerIntelligentMovement2 Implementation
// ============================================================================

MultiPlayerIntelligentMovement2::MultiPlayerIntelligentMovement2()
    : ball_name_("ball"),
      auto_mode_enabled_(true) {
    
    // Configure default parameters for all players
    algos::BallInterceptor::InterceptParams params;
    params.max_predict_steps = 10;
    params.collision_radius = 1.2; // 120px for 100x100px robots + safety
    params.max_candidates = 8;
    params.enable_obstacle_avoidance = true;
    params.player_max_speed = 0.9;
    
    // Add default controlled players
    AddControlledPlayer("robot0");
    //AddControlledPlayer("robot2");
    //AddControlledPlayer("robot3");
    
    // Set default opponents
    SetOpponents({"robot1", "robot2", "robot3", "robot4", "robot5", "robot6", "robot7", "robot8", "robot9"});
    
    std::cout << "[MultiPlayerIntelligentMovement2] Initialized multi-player system" << std::endl;
    PrintStatus();
}

MultiPlayerIntelligentMovement2::~MultiPlayerIntelligentMovement2() = default;

void MultiPlayerIntelligentMovement2::AddControlledPlayer(const std::string& player_name) {
    if (player_controllers_.find(player_name) != player_controllers_.end()) {
        std::cout << "[MultiPlayerIntelligentMovement2] Player " << player_name << " already controlled" << std::endl;
        return;
    }
    
    // Configure parameters for this player
    algos::BallInterceptor::InterceptParams params;
    params.max_predict_steps = 10;
    params.collision_radius = 1.2;
    params.max_candidates = 8;
    params.enable_obstacle_avoidance = true;
    params.player_max_speed = 0.9;
    
    player_controllers_[player_name] = std::make_unique<PlayerController>(player_name, params);
    
    std::cout << "[MultiPlayerIntelligentMovement2] Added player: " << player_name << std::endl;
}

void MultiPlayerIntelligentMovement2::RemoveControlledPlayer(const std::string& player_name) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        player_controllers_.erase(it);
        std::cout << "[MultiPlayerIntelligentMovement2] Removed player: " << player_name << std::endl;
    }
}

void MultiPlayerIntelligentMovement2::SetOpponents(const std::vector<std::string>& opponent_names) {
    opponent_names_ = opponent_names;
    std::cout << "[MultiPlayerIntelligentMovement2] Set opponents: ";
    for (const auto& name : opponent_names_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
}

void MultiPlayerIntelligentMovement2::SetMaxSpeed(double speed) {
    for (auto& [name, controller] : player_controllers_) {
        controller->SetMaxSpeed(speed);
    }
}

void MultiPlayerIntelligentMovement2::SetMaxAcceleration(double accel) {
    for (auto& [name, controller] : player_controllers_) {
        controller->SetMaxAcceleration(accel);
    }
}

void MultiPlayerIntelligentMovement2::SetPlayerEnabled(const std::string& player_name, bool enabled) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        it->second->SetEnabled(enabled);
        std::cout << "[MultiPlayerIntelligentMovement2] " << player_name 
                  << (enabled ? " enabled" : " disabled") << std::endl;
    }
}

void MultiPlayerIntelligentMovement2::SetPlayerMaxSpeed(const std::string& player_name, double speed) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        it->second->SetMaxSpeed(speed);
        std::cout << "[MultiPlayerIntelligentMovement2] " << player_name 
                  << " max speed set to " << speed << std::endl;
    }
}

void MultiPlayerIntelligentMovement2::UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!auto_mode_enabled_) return;
    
    // Coordinate team movement first
    CoordinateTeamMovement(game_objects);
    
    // Update each controlled player
    for (auto& [player_name, controller] : player_controllers_) {
        if (controller->IsEnabled()) {
            controller->UpdatePlayer(game_objects, opponent_names_, ball_name_, dt);
        }
    }
}

void MultiPlayerIntelligentMovement2::TogglePlayer(const std::string& player_name) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        bool current_state = it->second->IsEnabled();
        it->second->SetEnabled(!current_state);
        std::cout << "[MultiPlayerIntelligentMovement2] " << player_name 
                  << (current_state ? " disabled" : " enabled") << std::endl;
    }
}

std::vector<std::string> MultiPlayerIntelligentMovement2::GetControlledPlayers() const {
    std::vector<std::string> players;
    for (const auto& [name, controller] : player_controllers_) {
        players.push_back(name);
    }
    return players;
}

std::vector<std::string> MultiPlayerIntelligentMovement2::GetEnabledPlayers() const {
    std::vector<std::string> players;
    for (const auto& [name, controller] : player_controllers_) {
        if (controller->IsEnabled()) {
            players.push_back(name);
        }
    }
    return players;
}

void MultiPlayerIntelligentMovement2::CoordinateTeamMovement(std::map<std::string, GameObject>& game_objects) {
    // Simple coordination: determine which player should pursue the ball most aggressively
    std::string best_player = SelectBestPlayerForBall(game_objects);
    
    // For now, just log the coordination decision
    if (!best_player.empty()) {
        std::cout << "[TEAM COORDINATION] Best player for ball: " << best_player << std::endl;
    }
}

std::string MultiPlayerIntelligentMovement2::SelectBestPlayerForBall(const std::map<std::string, GameObject>& game_objects) const {
    auto ball_it = game_objects.find(ball_name_);
    if (ball_it == game_objects.end()) return "";
    
    const GameObject& ball = ball_it->second;
    glm::vec2 ball_pos = ball.GetCenterPosition();
    
    std::string best_player;
    float best_distance = std::numeric_limits<float>::max();
    
    for (const auto& [player_name, controller] : player_controllers_) {
        if (!controller->IsEnabled()) continue;
        
        auto player_it = game_objects.find(player_name);
        if (player_it != game_objects.end()) {
            glm::vec2 player_pos = player_it->second.GetCenterPosition();
            float distance = glm::length(ball_pos - player_pos);
            
            if (distance < best_distance) {
                best_distance = distance;
                best_player = player_name;
            }
        }
    }
    
    return best_player;
}

void MultiPlayerIntelligentMovement2::AssignPlayerRoles(const std::map<std::string, GameObject>& game_objects) {
    // Future implementation: assign roles like attacker, defender, support
    // For now, all players pursue the ball independently with coordination
}

void MultiPlayerIntelligentMovement2::PrintStatus() const {
    std::cout << "[MultiPlayerIntelligentMovement2] === STATUS ===" << std::endl;
    std::cout << "  Auto mode: " << (auto_mode_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "  Ball: " << ball_name_ << std::endl;
    std::cout << "  Controlled players: ";
    for (const auto& [name, controller] : player_controllers_) {
        std::cout << name << (controller->IsEnabled() ? "(ON)" : "(OFF)") << " ";
    }
    std::cout << std::endl;
    std::cout << "  Opponents: ";
    for (const auto& name : opponent_names_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
}

} // namespace vis