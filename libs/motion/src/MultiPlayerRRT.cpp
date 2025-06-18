#include "MultiPlayerRRT.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace vis {

// ============================================================================
// RRTPlayerController Implementation
// ============================================================================

RRTPlayerController::RRTPlayerController(const std::string& player_name)
    : player_name_(player_name),
      max_speed_(90.0),
      max_acceleration_(45.0),
      enabled_(true),
      current_trajectory_index_(0),
      trajectory_update_timer_(0.0f) {
    
    // Create path finder with RRT configuration
    path_finder_ = std::make_unique<algos::PathTrajectory>();
    
    // Configure path finder for RRT-based navigation
    path_finder_->setTimeStep(0.1); // 100ms time steps for RRT
    path_finder_->setLookaheadDistance(30.0);
    path_finder_->setObstacleAvoidanceWeight(1000.0); 
    path_finder_->setGoalAttractionWeight(100.0);
    
    std::cout << "[RRTPlayerController] Created RRT controller for " << player_name_ << std::endl;
}

RRTPlayerController::~RRTPlayerController() = default;

void RRTPlayerController::UpdatePlayer(std::map<std::string, GameObject>& game_objects, 
                                      const std::vector<std::string>& opponent_names,
                                      const std::string& ball_name, float dt) {
    if (!enabled_) return;
    
    // Find the player we're controlling
    auto player_it = game_objects.find(player_name_);
    if (player_it == game_objects.end()) {
        std::cerr << "[RRTPlayerController] Player '" << player_name_ << "' not found" << std::endl;
        return;
    }
    
    // Find the ball
    auto ball_it = game_objects.find(ball_name);
    if (ball_it == game_objects.end()) {
        std::cerr << "[RRTPlayerController] Ball '" << ball_name << "' not found" << std::endl;
        return;
    }
    
    GameObject& player = player_it->second;
    GameObject& ball = ball_it->second;
    
    // Update trajectory periodically with RRT  
    trajectory_update_timer_ += dt;
    if (trajectory_update_timer_ >= TRAJECTORY_UPDATE_INTERVAL || current_trajectory_.empty()) {
        trajectory_update_timer_ = 0.0f;
        UpdateTrajectory(game_objects, opponent_names, ball_name);
    }
    
    // Follow current trajectory (RRT handles obstacle avoidance in planning)
    FollowTrajectory(player, dt);
}

void RRTPlayerController::UpdateTrajectory(const std::map<std::string, GameObject>& game_objects,
                                          const std::vector<std::string>& opponent_names,
                                          const std::string& ball_name) {
    auto player_it = game_objects.find(player_name_);
    auto ball_it = game_objects.find(ball_name);
    
    if (player_it == game_objects.end() || ball_it == game_objects.end()) {
        return;
    }
    
    const GameObject& player = player_it->second;
    const GameObject& ball = ball_it->second;
    
    // Check if we should pursue the ball
    if (!ShouldPursue(player, ball, opponent_names, game_objects)) {
        std::cout << "[" << player_name_ << "] Decision: WAIT (other player closer to ball)" << std::endl;
        current_trajectory_.clear();
        return;
    }
    
    // Get current player position
    glm::vec2 player_pos = player.GetCenterPosition();
    glm::vec2 ball_pos = ball.GetCenterPosition();
    
    // Convert to trajectory algorithm format
    algos::Vector2D start_pos = ConvertToVector2D(player_pos);
    algos::Vector2D target_pos = ConvertToVector2D(ball_pos);
    
    // Get obstacles (other players)
    std::vector<algos::Obstacle> obstacles = GetObstacles(game_objects, opponent_names);
    
    // Calculate new trajectory using RRT
    std::cout << "[" << player_name_ << "] Using RRT for path planning..." << std::endl;
    std::cout << "  From: (" << start_pos.x << ", " << start_pos.y << ") to (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
    std::cout << "  Obstacles detected: " << obstacles.size() << std::endl;
    auto result = path_finder_->findRRTPath(start_pos, target_pos, obstacles, max_speed_, 500);
    
    if (result.success && !result.trajectory.empty()) {
        current_trajectory_ = result.trajectory;
        current_trajectory_index_ = 0;
        
        std::cout << "[" << player_name_ << "] New trajectory calculated:" << std::endl;
        std::cout << "  - Points: " << current_trajectory_.size() << std::endl;
        std::cout << "  - Distance: " << result.total_distance << std::endl;
        std::cout << "  - Time: " << result.total_time << "s" << std::endl;
        std::cout << "  - Obstacles: " << obstacles.size() << std::endl;
    } else {
        std::cout << "[" << player_name_ << "] Failed to find trajectory to ball" << std::endl;
        current_trajectory_.clear();
    }
}

void RRTPlayerController::FollowTrajectory(GameObject& player, float dt) {
    if (current_trajectory_.empty()) {
        player.velocity = glm::vec2(0.0f, 0.0f);
        return;
    }
    
    // Get current player position
    glm::vec2 current_pos = player.GetCenterPosition();
    
    // Find the next trajectory point to follow
    if (current_trajectory_index_ < current_trajectory_.size()) {
        algos::TrajectoryPoint target_point = current_trajectory_[current_trajectory_index_];
        glm::vec2 target_pos = ConvertToGLMVec2(target_point.position);
        
        // Calculate distance to current target point
        float distance_to_target = glm::length(target_pos - current_pos);
        
        // If we're close enough to current point, move to next one
        if (distance_to_target < 15.0f && current_trajectory_index_ < current_trajectory_.size() - 1) {
            current_trajectory_index_++;
            target_point = current_trajectory_[current_trajectory_index_];
            target_pos = ConvertToGLMVec2(target_point.position);
        }
        
        // Calculate direction and set velocity
        glm::vec2 direction = target_pos - current_pos;
        float distance = glm::length(direction);
        
        if (distance > 2.0f) {
            direction = glm::normalize(direction);
            
            // Follow RRT path strictly - don't blend velocities to avoid cutting corners
            player.velocity = direction * static_cast<float>(max_speed_);
            
            std::cout << "[" << player_name_ << "] Following trajectory point " 
                      << current_trajectory_index_ << "/" << current_trajectory_.size() 
                      << " at (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
        } else {
            player.velocity = glm::vec2(0.0f, 0.0f);
            std::cout << "[" << player_name_ << "] Reached ball!" << std::endl;
        }
    } else {
        // Reached end of trajectory
        player.velocity = glm::vec2(0.0f, 0.0f);
        std::cout << "[" << player_name_ << "] Trajectory completed" << std::endl;
    }
}

algos::Vector2D RRTPlayerController::ConvertToVector2D(const glm::vec2& vec) const {
    return algos::Vector2D(vec.x, vec.y);
}

glm::vec2 RRTPlayerController::ConvertToGLMVec2(const algos::Vector2D& vec) const {
    return glm::vec2(vec.x, vec.y);
}

std::vector<algos::Obstacle> RRTPlayerController::GetObstacles(const std::map<std::string, GameObject>& game_objects,
                                                              const std::vector<std::string>& opponent_names) const {
    std::vector<algos::Obstacle> obstacles;
    
    std::cout << "[" << player_name_ << "] ============ OBSTACLE DETECTION DEBUG ============" << std::endl;
    std::cout << "[" << player_name_ << "] Controlled player = " << player_name_ << std::endl;
    
    // Add specified opponents as obstacles
    for (const std::string& opponent_name : opponent_names) {
        auto opponent_it = game_objects.find(opponent_name);
        if (opponent_it != game_objects.end() && opponent_name != player_name_) {
            const GameObject& opponent = opponent_it->second;
            glm::vec2 pos = opponent.GetCenterPosition();
            glm::vec2 vel = opponent.velocity;
            
            algos::Vector2D obstacle_pos = ConvertToVector2D(pos);
            algos::Vector2D obstacle_vel = ConvertToVector2D(vel);
            
            // Robot obstacle with appropriate radius for RRT planning
            // Robots are 100x100px, so radius is 50px + safety margin
            double safety_radius = 80.0; // Robot radius (50) + safety margin (30)
            obstacles.push_back(algos::Obstacle(obstacle_pos, safety_radius, obstacle_vel));
            
            std::cout << "[" << player_name_ << "] *** ADDED OBSTACLE: " << opponent_name 
                      << " at (" << pos.x << ", " << pos.y << ") with radius " << safety_radius << " ***" << std::endl;
        }
    }
    
    std::cout << "[" << player_name_ << "] DEBUG: Total obstacles found: " << obstacles.size() << std::endl;
    return obstacles;
}

bool RRTPlayerController::ShouldPursue(const GameObject& player, const GameObject& ball,
                                      const std::vector<std::string>& opponent_names,
                                      const std::map<std::string, GameObject>& game_objects) const {
    // Simple strategy: pursue if we're relatively close to the ball
    glm::vec2 player_pos = player.GetCenterPosition();
    glm::vec2 ball_pos = ball.GetCenterPosition();
    float distance_to_ball = glm::length(ball_pos - player_pos);
    
    // Check if any other controlled player is significantly closer
    float min_other_distance = std::numeric_limits<float>::max();
    for (const auto& [name, obj] : game_objects) {
        if (name != player_name_ && name.find("robot") != std::string::npos && 
            std::find(opponent_names.begin(), opponent_names.end(), name) == opponent_names.end()) {
            // This is another controlled player
            glm::vec2 other_pos = obj.GetCenterPosition();
            float other_distance = glm::length(ball_pos - other_pos);
            min_other_distance = std::min(min_other_distance, other_distance);
        }
    }
    
    // Pursue if we're the closest or within reasonable range
    bool should_pursue = (distance_to_ball <= min_other_distance + 50.0f); // 50px tolerance
    
    std::cout << "[" << player_name_ << "] Pursuit analysis:" << std::endl;
    std::cout << "  - Distance to ball: " << distance_to_ball << "px" << std::endl;
    std::cout << "  - Closest other player distance: " << min_other_distance << "px" << std::endl;
    std::cout << "  - Should pursue: " << (should_pursue ? "YES" : "NO") << std::endl;
    
    return should_pursue;
}

// ============================================================================
// MultiPlayerRRT Implementation
// ============================================================================

MultiPlayerRRT::MultiPlayerRRT()
    : ball_name_("ball"),
      auto_mode_enabled_(true) {
    
    // Add default controlled players
    AddControlledPlayer("robot0");
    // AddControlledPlayer("robot2");
    // AddControlledPlayer("robot3");
    
    // Set default opponents (includes other robots and obstacles)
    SetOpponents({"robot1", "robot2", "robot3", "robot4", "robot5", "robot6", "robot7", "robot8", "robot9"});
    
    std::cout << "[MultiPlayerRRT] Initialized multi-player RRT system" << std::endl;
    PrintStatus();
}

MultiPlayerRRT::~MultiPlayerRRT() = default;

void MultiPlayerRRT::AddControlledPlayer(const std::string& player_name) {
    if (player_controllers_.find(player_name) != player_controllers_.end()) {
        std::cout << "[MultiPlayerRRT] Player " << player_name << " already controlled" << std::endl;
        return;
    }
    
    player_controllers_[player_name] = std::make_unique<RRTPlayerController>(player_name);
    
    std::cout << "[MultiPlayerRRT] Added player: " << player_name << std::endl;
}

void MultiPlayerRRT::RemoveControlledPlayer(const std::string& player_name) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        player_controllers_.erase(it);
        std::cout << "[MultiPlayerRRT] Removed player: " << player_name << std::endl;
    }
}

void MultiPlayerRRT::SetOpponents(const std::vector<std::string>& opponent_names) {
    opponent_names_ = opponent_names;
    std::cout << "[MultiPlayerRRT] Set opponents: ";
    for (const auto& name : opponent_names_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
}

void MultiPlayerRRT::SetMaxSpeed(double speed) {
    for (auto& [name, controller] : player_controllers_) {
        controller->SetMaxSpeed(speed);
    }
}

void MultiPlayerRRT::SetMaxAcceleration(double accel) {
    for (auto& [name, controller] : player_controllers_) {
        controller->SetMaxAcceleration(accel);
    }
}

void MultiPlayerRRT::SetPlayerEnabled(const std::string& player_name, bool enabled) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        it->second->SetEnabled(enabled);
        std::cout << "[MultiPlayerRRT] " << player_name 
                  << (enabled ? " enabled" : " disabled") << std::endl;
    }
}

void MultiPlayerRRT::SetPlayerMaxSpeed(const std::string& player_name, double speed) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        it->second->SetMaxSpeed(speed);
        std::cout << "[MultiPlayerRRT] " << player_name 
                  << " max speed set to " << speed << std::endl;
    }
}

void MultiPlayerRRT::UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt) {
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

void MultiPlayerRRT::TogglePlayer(const std::string& player_name) {
    auto it = player_controllers_.find(player_name);
    if (it != player_controllers_.end()) {
        bool current_state = it->second->IsEnabled();
        it->second->SetEnabled(!current_state);
        std::cout << "[MultiPlayerRRT] " << player_name 
                  << (current_state ? " disabled" : " enabled") << std::endl;
    }
}

std::vector<std::string> MultiPlayerRRT::GetControlledPlayers() const {
    std::vector<std::string> players;
    for (const auto& [name, controller] : player_controllers_) {
        players.push_back(name);
    }
    return players;
}

std::vector<std::string> MultiPlayerRRT::GetEnabledPlayers() const {
    std::vector<std::string> players;
    for (const auto& [name, controller] : player_controllers_) {
        if (controller->IsEnabled()) {
            players.push_back(name);
        }
    }
    return players;
}

void MultiPlayerRRT::CoordinateTeamMovement(std::map<std::string, GameObject>& game_objects) {
    // Simple coordination: determine which player should pursue the ball most aggressively
    std::string best_player = SelectBestPlayerForBall(game_objects);
    
    // For now, just log the coordination decision
    if (!best_player.empty()) {
        std::cout << "[TEAM COORDINATION] Best RRT player for ball: " << best_player << std::endl;
    }
}

std::string MultiPlayerRRT::SelectBestPlayerForBall(const std::map<std::string, GameObject>& game_objects) const {
    auto ball_it = game_objects.find(ball_name_);
    if (ball_it == game_objects.end()) return "";
    
    const GameObject& ball = ball_it->second;
    glm::vec2 ball_pos = ball.GetCenterPosition();
    
    std::string best_player;
    float best_score = std::numeric_limits<float>::max();
    
    for (const auto& [player_name, controller] : player_controllers_) {
        if (!controller->IsEnabled()) continue;
        
        auto player_it = game_objects.find(player_name);
        if (player_it != game_objects.end()) {
            const GameObject& player = player_it->second;
            glm::vec2 player_pos = player.GetCenterPosition();
            
            // Score based on distance and path complexity
            float distance = glm::length(ball_pos - player_pos);
            float path_length = CalculatePathLength(player, ball, opponent_names_, game_objects);
            
            // Lower score is better (closer distance + simpler path)
            float score = distance + (path_length * 0.5f);
            
            if (score < best_score) {
                best_score = score;
                best_player = player_name;
            }
        }
    }
    
    return best_player;
}

float MultiPlayerRRT::CalculatePathLength(const GameObject& player, const GameObject& ball,
                                         const std::vector<std::string>& opponent_names,
                                         const std::map<std::string, GameObject>& game_objects) const {
    // Simple heuristic: return straight-line distance if no obstacles, 
    // or distance + penalty if obstacles are in the way
    glm::vec2 player_pos = player.GetCenterPosition();
    glm::vec2 ball_pos = ball.GetCenterPosition();
    float direct_distance = glm::length(ball_pos - player_pos);
    
    // Check for obstacles in direct path (simplified)
    int obstacles_in_path = 0;
    for (const std::string& opponent_name : opponent_names) {
        auto opponent_it = game_objects.find(opponent_name);
        if (opponent_it != game_objects.end()) {
            glm::vec2 opponent_pos = opponent_it->second.GetCenterPosition();
            
            // Simple check: if opponent is roughly between player and ball
            glm::vec2 to_ball = ball_pos - player_pos;
            glm::vec2 to_opponent = opponent_pos - player_pos;
            
            float dot_product = glm::dot(glm::normalize(to_ball), glm::normalize(to_opponent));
            float opponent_distance = glm::length(to_opponent);
            
            // If opponent is in front of player and within reasonable range
            if (dot_product > 0.7f && opponent_distance < direct_distance) {
                obstacles_in_path++;
            }
        }
    }
    
    // Add penalty for obstacles
    return direct_distance + (obstacles_in_path * 50.0f);
}

void MultiPlayerRRT::AssignPlayerRoles(const std::map<std::string, GameObject>& game_objects) {
    // Future implementation: assign roles like primary pursuer, support, defender
    // For now, all players pursue the ball with coordination
}

void MultiPlayerRRT::PrintStatus() const {
    std::cout << "[MultiPlayerRRT] === STATUS ===" << std::endl;
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