#include "IntelligentMovement2.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace vis {

IntelligentMovement2::IntelligentMovement2() 
    : player_name_("robot0"),
      opponent_name_("robot1"), 
      ball_name_("ball"),
      max_speed_(80.0),
      max_acceleration_(40.0),
      auto_mode_enabled_(true),  // Enable IntelligentMovement2 by default
      action_timer_(0.0f),
      has_active_action_(false),
      intercept_update_timer_(0.0f) {
    
    // Configure ball interceptor for 100x100 pixel robots
    algos::BallInterceptor::InterceptParams intercept_params;
    intercept_params.max_predict_steps = 10; // Shorter prediction for more immediate targets
    intercept_params.collision_radius = 1.2; // Larger radius: 50px robot radius + 70px safety = 120px total / 100px scaling
    intercept_params.max_candidates = 8; // Fewer candidates for faster computation
    intercept_params.enable_obstacle_avoidance = true;
    intercept_params.player_max_speed = 0.8; // Slightly slower normalized speed
    
    ball_interceptor_ = std::make_unique<algos::BallInterceptor>(intercept_params);
    ball_intercept_action_ = std::make_unique<algos::BallInterceptAction>(intercept_params);
    
    std::cout << "[IntelligentMovement2] Initialized with ball interception system" << std::endl;
    std::cout << "  - Source player: " << player_name_ << std::endl;
    std::cout << "  - Opponent: " << opponent_name_ << std::endl;
    std::cout << "  - Ball: " << ball_name_ << std::endl;
    std::cout << "  - Robot collision radius: " << intercept_params.collision_radius << " (normalized units, ~75px actual)" << std::endl;
    std::cout << "  - Max speed: " << max_speed_ << std::endl;
}

IntelligentMovement2::~IntelligentMovement2() = default;

void IntelligentMovement2::UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!auto_mode_enabled_) return;
    
    // Find the player we're controlling
    auto player_it = game_objects.find(player_name_);
    if (player_it == game_objects.end()) {
        std::cerr << "[IntelligentMovement2] Player '" << player_name_ << "' not found" << std::endl;
        return;
    }
    
    // Find the ball
    auto ball_it = game_objects.find(ball_name_);
    if (ball_it == game_objects.end()) {
        std::cerr << "[IntelligentMovement2] Ball '" << ball_name_ << "' not found" << std::endl;
        return;
    }
    
    GameObject& player = player_it->second;
    
    // Update interception strategy periodically
    intercept_update_timer_ += dt;
    if (intercept_update_timer_ >= INTERCEPT_UPDATE_INTERVAL || !has_active_action_) {
        intercept_update_timer_ = 0.0f;
        UpdateInterception(game_objects);
    }
    
    // Execute current action
    if (has_active_action_) {
        ExecuteCurrentAction(player, dt);
    }
}

void IntelligentMovement2::UpdateInterception(const std::map<std::string, GameObject>& game_objects) {
    auto player_it = game_objects.find(player_name_);
    auto ball_it = game_objects.find(ball_name_);
    
    if (player_it == game_objects.end() || ball_it == game_objects.end()) {
        return;
    }
    
    const GameObject& player = player_it->second;
    const GameObject& ball = ball_it->second;
    
    // Convert to algorithm format
    algos::SimplePlayer simple_player = ConvertToSimplePlayer(player);
    algos::SimpleBall simple_ball = ConvertToSimpleBall(ball);
    std::vector<algos::SimplePlayer> opponents = GetOpponents(game_objects);
    
    std::cout << "[" << player_name_ << "] === BALL INTERCEPTION ANALYSIS ===" << std::endl;
    std::cout << "  Player at: (" << simple_player.x << ", " << simple_player.y 
              << ") angle: " << simple_player.angle << std::endl;
    std::cout << "  Ball at: (" << simple_ball.x << ", " << simple_ball.y 
              << ") vel: (" << simple_ball.vx << ", " << simple_ball.vy << ")" << std::endl;
    std::cout << "  Opponents: " << opponents.size() << std::endl;
    
    // Check if robot1 is directly in our path to the ball
    auto robot1_it = game_objects.find("robot1");
    if (robot1_it != game_objects.end()) {
        glm::vec2 player_pos = player.GetCenterPosition();
        glm::vec2 ball_pos = ball.GetCenterPosition();
        glm::vec2 robot1_pos = robot1_it->second.GetCenterPosition();
        
        float dist_player_robot1 = glm::length(robot1_pos - player_pos);
        float dist_robot1_ball = glm::length(ball_pos - robot1_pos);
        float dist_player_ball = glm::length(ball_pos - player_pos);
        
        std::cout << "  [COLLISION CHECK] Player to robot1: " << dist_player_robot1 << "px" << std::endl;
        std::cout << "  [COLLISION CHECK] Robot1 to ball: " << dist_robot1_ball << "px" << std::endl;
        std::cout << "  [COLLISION CHECK] Player to ball: " << dist_player_ball << "px" << std::endl;
        
        // Check if robot1 is in direct path (within some tolerance)
        if (dist_player_robot1 + dist_robot1_ball <= dist_player_ball + 50) {
            std::cout << "  ⚠️ [COLLISION WARNING] Robot1 is blocking direct path to ball!" << std::endl;
        }
    }
    
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
        std::cout << "  Target (normalized): (" << current_action_.target_position.x 
                  << ", " << current_action_.target_position.y << ")" << std::endl;
        
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

void IntelligentMovement2::ExecuteCurrentAction(GameObject& player, float dt) {
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

void IntelligentMovement2::ApplyDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
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

void IntelligentMovement2::ApplyTurnAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
    // TURN should move toward the target, not just stop
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

void IntelligentMovement2::ApplyTurnAndDashAction(GameObject& player, const algos::BallInterceptAction::Action& action, float dt) {
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

algos::SimplePlayer IntelligentMovement2::ConvertToSimplePlayer(const GameObject& obj) const {
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

algos::SimpleBall IntelligentMovement2::ConvertToSimpleBall(const GameObject& obj) const {
    glm::vec2 pos = obj.GetCenterPosition();
    
    // Normalize coordinates and velocity to algorithm scale
    double norm_x = pos.x / 100.0;
    double norm_y = pos.y / 100.0;
    double norm_vx = obj.velocity.x / 100.0;
    double norm_vy = obj.velocity.y / 100.0;
    
    return algos::SimpleBall(norm_x, norm_y, norm_vx, norm_vy);
}

glm::vec2 IntelligentMovement2::ConvertToGLMVec2(const algos::Vector2D& vec) const {
    // Scale back up from normalized coordinates
    return glm::vec2(vec.x * 100.0, vec.y * 100.0);
}

algos::Vector2D IntelligentMovement2::ConvertToVector2D(const glm::vec2& vec) const {
    // Scale down to normalized coordinates
    return algos::Vector2D(vec.x / 100.0, vec.y / 100.0);
}

std::vector<algos::SimplePlayer> IntelligentMovement2::GetOpponents(const std::map<std::string, GameObject>& game_objects) const {
    std::vector<algos::SimplePlayer> opponents;
    
    // For now, only consider the specified opponent
    auto opponent_it = game_objects.find(opponent_name_);
    if (opponent_it != game_objects.end()) {
        opponents.push_back(ConvertToSimplePlayer(opponent_it->second));
        
        std::cout << "[IntelligentMovement2] Added opponent: " << opponent_name_ 
                  << " at (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    }
    
    return opponents;
}

bool IntelligentMovement2::ShouldPursue(const algos::SimplePlayer& player, const algos::SimpleBall& ball, 
                                       const std::vector<algos::SimplePlayer>& opponents) const {
    // Calculate minimum steps to intercept for player
    int self_min_step = ball_intercept_action_->calculateMinInterceptSteps(player, ball);
    
    // Calculate minimum steps for opponents
    int opponent_min_step = 1000; // Default high value
    if (!opponents.empty()) {
        opponent_min_step = ball_intercept_action_->calculateMinInterceptSteps(opponents[0], ball);
    }
    
    // Use shouldChase logic from BallInterceptAction
    bool should_chase = ball_intercept_action_->shouldChase(player, ball, self_min_step, 1000, opponent_min_step);
    
    std::cout << "[IntelligentMovement2] Interception analysis:" << std::endl;
    std::cout << "  - Self min steps: " << self_min_step << std::endl;
    std::cout << "  - Opponent min steps: " << opponent_min_step << std::endl;
    std::cout << "  - Should chase: " << (should_chase ? "YES" : "NO") << std::endl;
    
    return should_chase;
}

} // namespace vis