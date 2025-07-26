#define _USE_MATH_DEFINES
#include "RobotManager.h"
#include "Kinematics.h"
#include "Dribble.h"
#include "BallModel.h"
#include "Utils.h"
#include <cmath>
#include <iostream>

void rob::RobotManager::KickBall() { 
    robot_action = RobotAction::KICK_BALL; 
    // The actual kick will be executed in the control loop when a ball is near
}

void rob::RobotManager::PassBall() { 
    robot_action = RobotAction::PASS_BALL; 
    // Pass functionality - currently same as kick but could be different
}

void rob::RobotManager::ExecuteKickAction(std::vector<state::SoccerObject>& soccer_objects) {
    // Find the ball and this robot in the soccer_objects
    state::SoccerObject* ball = nullptr;
    state::SoccerObject* robot = nullptr;
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "ball") {
            ball = &obj;
        } else if (obj.name == "robot0") {  // Matches the name created in InitSoccerObjects
            robot = &obj;
        }
    }

    if (!ball || !robot) return;
    
    // Check if ball is close enough to kick
    Eigen::Vector3d ball_center = ball->GetCenterPosition();
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    
    double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                               std::pow(ball_center.y() - robot_center.y(), 2));
    
    // Only kick if ball is close (within kicking range)
    if (distance < 0.3) {  // 30cm kicking range
        // Calculate kick direction based on robot orientation
        double robot_angle = robot->position[2];
        Eigen::Vector2d kick_direction(std::cos(robot_angle), std::sin(robot_angle));
        
        // Apply kick with moderate power
        double kick_power = 5.0;  // m/s kick velocity
        kin::ApplyKickToBall(*ball, kick_direction, kick_power);
        
        // Reset action after kicking
        robot_action = RobotAction::MOVE;
    }
}

void rob::RobotManager::ExecutePassAction(std::vector<state::SoccerObject>& soccer_objects, 
                                          const Eigen::Vector2d& target_position) {
    // Find the ball and this robot
    state::SoccerObject* ball = nullptr;
    state::SoccerObject* robot = nullptr;
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "ball") {
            ball = &obj;
        } else if (obj.name == "robot0") {  // Matches the name created in InitSoccerObjects
            robot = &obj;
        }
    }
    
    if (!ball || !robot) return;
    
    // Check if ball is close enough to pass
    Eigen::Vector3d ball_center = ball->GetCenterPosition();
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    
    double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                               std::pow(ball_center.y() - robot_center.y(), 2));
    
    if (distance < 0.3) {  // 30cm passing range
        // Calculate pass direction toward target
        Eigen::Vector2d pass_direction = target_position - Eigen::Vector2d(robot_center.x(), robot_center.y());
        pass_direction.normalize();
        
        // Apply pass with lighter power than kick
        double pass_power = 3.0;  // m/s pass velocity
        kin::ApplyKickToBall(*ball, pass_direction, pass_power);

        // Reset action after passing
        robot_action = RobotAction::MOVE;
    }
}

void rob::RobotManager::ExecuteDribbleAction(std::vector<state::SoccerObject>& soccer_objects) {
    // Find the ball and this robot in the soccer_objects
    state::SoccerObject* ball = nullptr;
    state::SoccerObject* robot = nullptr;
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "ball") {
            ball = &obj;
        } else if (obj.name == "robot0") {  // Updated to use correct robot name
            robot = &obj;
        }
    }

    if (!ball || !robot) return;
    
    state::Ball* ball_ptr = dynamic_cast<state::Ball*>(ball);
    if (!ball_ptr) return; // Ensure it's actually a Ball object

    // Check if ball is close enough to dribble
    Eigen::Vector3d ball_center = ball_ptr->GetCenterPosition();
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    
    double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                               std::pow(ball_center.y() - robot_center.y(), 2));
    
    // Debug output to understand what's happening
    static double last_debug_print = 0;
    double current_time = util::GetCurrentTime();
    if (current_time - last_debug_print > 1.0) {  // Print every second
        std::cout << "[DEBUG] Robot at (" << robot_center.x() << ", " << robot_center.y() 
                  << "), Ball at (" << ball_center.x() << ", " << ball_center.y() 
                  << "), Distance: " << distance << "m" << std::endl;
        last_debug_print = current_time;
    }
    
    // SSL RULE: Check if robot is facing the ball before allowing dribbling
    Eigen::Vector3d robot_to_ball = ball_center - robot_center;
    robot_to_ball.z() = 0;  // 2D only
    double ball_angle = atan2(robot_to_ball.y(), robot_to_ball.x());
    double robot_angle = robot->position[2];
    
    // Normalize angles to [-pi, pi]
    auto normalize_angle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };
    
    double angle_diff = normalize_angle(ball_angle - robot_angle);
    bool robot_facing_ball = std::abs(angle_diff) < M_PI/3;  // Within 60 degrees (±30°)
    
    
    // Debug output for orientation checking
    static double last_orientation_debug = 0;
    if (current_time - last_orientation_debug > 2.0) {
        std::cout << "[DRIBBLE DEBUG] Robot angle: " << robot_angle << " rad (" << (robot_angle * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "[DRIBBLE DEBUG] Ball angle: " << ball_angle << " rad (" << (ball_angle * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "[DRIBBLE DEBUG] Angle diff: " << angle_diff << " rad (" << (angle_diff * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "[DRIBBLE DEBUG] Robot facing ball: " << (robot_facing_ball ? "YES" : "NO") << std::endl;
        last_orientation_debug = current_time;
    }
    
    // Dribble if ball is within dribbling range AND robot is facing the ball (SSL compliance)
    if (distance < 0.3 && robot_facing_ball) {  // 30cm dribbling range + orientation check
        // SSL RULE: Set ball to dribbling mode (physics-based, not attached)
        ball->is_dribbling = true;
        
        // Apply continuous dribbling force using the Dribble function
        // This applies forces to keep ball close without attaching (SSL legal)
        double dribble_power = 15.0;  // Higher power to make ball keep up with robot movement
        bool success = kin::Dribble(*robot, *ball_ptr, dribble_power, true);  // continuous = true
        
        // Get robot velocity to help ball follow robot's movement
        Eigen::Vector3d robot_velocity = robot->velocity;
        double robot_speed = std::sqrt(robot_velocity.x()*robot_velocity.x() + robot_velocity.y()*robot_velocity.y());
        
        // If robot is moving, apply velocity-matching force to ball
        if (robot_speed > 0.05) {  // Robot is moving
            Eigen::Vector2d velocity_match_force;
            velocity_match_force.x() = robot_velocity.x() * 3.0;  // Match robot velocity
            velocity_match_force.y() = robot_velocity.y() * 3.0;
            
            // Apply velocity matching force to keep ball moving with robot
            if (auto* ball_model = dynamic_cast<state::Ball*>(ball)) {
                ball_model->ApplyDribbleForce(ball->velocity, velocity_match_force);
            } else {
                ball->velocity.x() += velocity_match_force.x() * 0.3;
                ball->velocity.y() += velocity_match_force.y() * 0.3;
            }
        }
        
        // Additional boost when ball is falling behind during movement
        if (distance > 0.2) {  // Ball is getting too far, apply stronger force
            // Calculate direction from ball to robot desired position
            Eigen::Vector3d robot_front = robot_center;
            robot_front.x() += 0.08 * cos(robot->position.z());  // 8cm in front of robot
            robot_front.y() += 0.08 * sin(robot->position.z());
            
            Eigen::Vector2d pull_force;
            pull_force.x() = (robot_front.x() - ball_center.x()) * 20.0;  // Strong pull force
            pull_force.y() = (robot_front.y() - ball_center.y()) * 20.0;
            
            // Apply pull force to bring ball closer
            if (auto* ball_model = dynamic_cast<state::Ball*>(ball)) {
                ball_model->ApplyDribbleForce(ball->velocity, pull_force);
            } else {
                ball->velocity.x() += pull_force.x() * 0.1;
                ball->velocity.y() += pull_force.y() * 0.1;
            }
            
            std::cout << "[DEBUG] Applying pull force to ball (distance=" << distance << "m)" << std::endl;
        }
        
        // Ensure ball is not attached (SSL rule compliance)
        if (ball->is_attached) {
            ball->is_attached = false;
            ball->attached_to = nullptr;
        }
    } else {
        // Not dribbling - clear dribbling flag
        ball->is_dribbling = false;
        
        if (distance < 0.3 && !robot_facing_ball) {
            // Ball is close but robot is not facing it - SSL rule violation prevention
            static double last_warning = 0;
            if (current_time - last_warning > 2.0) {
                std::cout << "[SSL RULE] Robot not facing ball (angle diff: " << angle_diff 
                          << " rad). Dribbling disabled." << std::endl;
                last_warning = current_time;
            }
        }
    }
}

