#include "IntelligentMovement.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace vis {

IntelligentMovement::IntelligentMovement() 
    : path_finder_(std::make_unique<algos::PathTrajectory>()),
      auto_mode_enabled_(false),  // Disable RRT by default
      ball_name_("ball"),
      player_name_("robot0"),
      max_speed_(80.0),
      max_acceleration_(40.0),
      current_trajectory_index_(0),
      trajectory_update_timer_(0.0f) {
    
    // Configure path finder for RRT-based navigation
    path_finder_->setTimeStep(0.1); // 100ms time steps for RRT
    path_finder_->setLookaheadDistance(30.0);
    path_finder_->setObstacleAvoidanceWeight(1000.0); 
    path_finder_->setGoalAttractionWeight(100.0);
    
    std::cout << "[IntelligentMovement] Initialized with trajectory-based movement" << std::endl;
    std::cout << "  - Target player: " << player_name_ << std::endl;
    std::cout << "  - Ball: " << ball_name_ << std::endl;
    std::cout << "  - Max speed: " << max_speed_ << std::endl;
}

IntelligentMovement::~IntelligentMovement() = default;

void IntelligentMovement::UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!auto_mode_enabled_) return;
    
    // Find the player we're controlling
    auto player_it = game_objects.find(player_name_);
    if (player_it == game_objects.end()) {
        std::cerr << "[IntelligentMovement] Player '" << player_name_ << "' not found" << std::endl;
        return;
    }
    
    // Find the ball
    auto ball_it = game_objects.find(ball_name_);
    if (ball_it == game_objects.end()) {
        std::cerr << "[IntelligentMovement] Ball '" << ball_name_ << "' not found" << std::endl;
        return;
    }
    
    GameObject& player = player_it->second;
    GameObject& ball = ball_it->second;
    
    // Update trajectory periodically with RRT  
    trajectory_update_timer_ += dt;
    if (trajectory_update_timer_ >= TRAJECTORY_UPDATE_INTERVAL || current_trajectory_.empty()) {
        trajectory_update_timer_ = 0.0f;
        UpdateTrajectory(game_objects);
    }
    
    // Follow current trajectory (RRT handles obstacle avoidance in planning)
    FollowTrajectory(player, dt);
}

void IntelligentMovement::UpdateTrajectory(const std::map<std::string, GameObject>& game_objects) {
    auto player_it = game_objects.find(player_name_);
    auto ball_it = game_objects.find(ball_name_);
    
    if (player_it == game_objects.end() || ball_it == game_objects.end()) {
        return;
    }
    
    const GameObject& player = player_it->second;
    const GameObject& ball = ball_it->second;
    
    // Get current player position
    glm::vec2 player_pos = player.GetCenterPosition();
    glm::vec2 ball_pos = ball.GetCenterPosition();
    
    // Convert to trajectory algorithm format
    algos::Vector2D start_pos = ConvertToVector2D(player_pos);
    algos::Vector2D target_pos = ConvertToVector2D(ball_pos);
    
    // Get obstacles (other players)
    std::vector<algos::Obstacle> obstacles = GetObstacles(game_objects);
    
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

void IntelligentMovement::FollowTrajectory(GameObject& player, float dt) {
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
        
        // If we're close enough to current point, move to next one (smaller threshold for precise path following)
        if (distance_to_target < 10.0f && current_trajectory_index_ < current_trajectory_.size() - 1) {
            current_trajectory_index_++;
            target_point = current_trajectory_[current_trajectory_index_];
            target_pos = ConvertToGLMVec2(target_point.position);
        }
        
        // Calculate direction and set velocity
        glm::vec2 direction = target_pos - current_pos;
        float distance = glm::length(direction);
        
        if (distance > 2.0f) {
            direction = glm::normalize(direction);
            
            // Use the trajectory's planned velocity, but limit to our max speed
            glm::vec2 planned_velocity = ConvertToGLMVec2(target_point.velocity);
            float planned_speed = glm::length(planned_velocity);
            
            if (planned_speed > max_speed_) {
                planned_velocity = glm::normalize(planned_velocity) * static_cast<float>(max_speed_);
            }
            
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

algos::Vector2D IntelligentMovement::ConvertToVector2D(const glm::vec2& vec) {
    return algos::Vector2D(vec.x, vec.y);
}

glm::vec2 IntelligentMovement::ConvertToGLMVec2(const algos::Vector2D& vec) {
    return glm::vec2(vec.x, vec.y);
}

std::vector<algos::Obstacle> IntelligentMovement::GetObstacles(const std::map<std::string, GameObject>& game_objects) {
    std::vector<algos::Obstacle> obstacles;
    
    std::cout << "[IntelligentMovement] ============ OBSTACLE DETECTION DEBUG ============" << std::endl;
    std::cout << "[IntelligentMovement] Controlled player = " << player_name_ << std::endl;
    std::cout << "[IntelligentMovement] Ball = " << ball_name_ << std::endl;
    std::cout << "[IntelligentMovement] All game objects:" << std::endl;
    for (const auto& [name, obj] : game_objects) {
        glm::vec2 pos = obj.GetCenterPosition();
        std::cout << "  - " << name << " at (" << pos.x << ", " << pos.y << ")" << std::endl;
    }
    
    for (const auto& [name, obj] : game_objects) {
        std::cout << "[IntelligentMovement] DEBUG: Checking " << name << std::endl;
        
        // Skip the controlled player, ball, and background
        if (name == player_name_) {
            std::cout << "  -> Skipping controlled player" << std::endl;
            continue;
        }
        if (name == ball_name_) {
            std::cout << "  -> Skipping ball" << std::endl;
            continue;
        }
        if (name == "background") {
            std::cout << "  -> Skipping background" << std::endl;
            continue;
        }
        
        // For now, ONLY add robot1 as obstacle
        if (name == "robot1") {
            glm::vec2 pos = obj.GetCenterPosition();
            glm::vec2 vel = obj.velocity;
            
            algos::Vector2D obstacle_pos = ConvertToVector2D(pos);
            algos::Vector2D obstacle_vel = ConvertToVector2D(vel);
            
            // Robot obstacle with appropriate radius for RRT planning
            // Robots are 100x100px, so radius is 50px + safety margin
            double safety_radius = 80.0; // Robot radius (50) + safety margin (25)
            obstacles.push_back(algos::Obstacle(obstacle_pos, safety_radius, obstacle_vel));
            
            std::cout << "[IntelligentMovement] *** ADDED OBSTACLE: " << name 
                      << " at (" << pos.x << ", " << pos.y << ") with radius " << safety_radius << " ***" << std::endl;
        } else if (name.find("robot") != std::string::npos) {
            std::cout << "  -> Skipping other robot: " << name << std::endl;
        } else {
            std::cout << "  -> Ignoring: " << name << std::endl;
        }
    }
    
    std::cout << "[IntelligentMovement] DEBUG: Total obstacles found: " << obstacles.size() << std::endl;
    return obstacles;
}

}