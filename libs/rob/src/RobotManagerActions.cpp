#include "RobotManager.h"
#include "Kinematics.h"
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
        } else if (obj.name == "robot0") {  // Fixed: actual name is robot0, not robot_0
            robot = &obj;
        }
    }
    
    if (!ball || !robot) {
        std::cout << "[ExecuteKickAction] ERROR: Could not find ball or robot!" << std::endl;
        return;
    }
    
    // Check if ball is close enough to kick
    Eigen::Vector3d ball_center = ball->GetCenterPosition();
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    
    double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                               std::pow(ball_center.y() - robot_center.y(), 2));
    
    std::cout << "[ExecuteKickAction] Robot pos: (" << robot_center.x() << ", " << robot_center.y() 
              << "), Ball pos: (" << ball_center.x() << ", " << ball_center.y() 
              << "), Distance: " << distance << "m" << std::endl;
    
    // Only kick if ball is close (within kicking range)
    if (distance < 0.3) {  // 30cm kicking range
        // Calculate kick direction based on robot orientation
        double robot_angle = robot->position[2];
        Eigen::Vector2d kick_direction(std::cos(robot_angle), std::sin(robot_angle));
        
        std::cout << "[ExecuteKickAction] KICKING! Robot angle: " << robot_angle 
                  << " rad, Kick direction: (" << kick_direction.x() << ", " << kick_direction.y() 
                  << "), Power: 3.0 m/s" << std::endl;
        
        // Store ball velocity before kick for comparison
        Eigen::Vector3d old_velocity = ball->velocity;
        
        // Apply kick with SSL realistic power
        double kick_power = 3.0;  // m/s kick velocity (SSL typical: 2-4 m/s)
        kin::ApplyKickToBall(*ball, kick_direction, kick_power);
        
        std::cout << "[ExecuteKickAction] Ball velocity changed from (" << old_velocity.x() << ", " 
                  << old_velocity.y() << ") to (" << ball->velocity.x() << ", " << ball->velocity.y() << ")" << std::endl;
        
        // Reset action after kicking
        robot_action = RobotAction::MOVE;
    } else {
        std::cout << "[ExecuteKickAction] Ball too far away! Distance: " << distance 
                  << "m (need < 0.3m)" << std::endl;
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
        } else if (obj.name == "robot0") {  // Fixed: actual name is robot0, not robot_0
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
        double pass_power = 2.0;  // m/s pass velocity (SSL typical pass speed)
        kin::ApplyKickToBall(*ball, pass_direction, pass_power);
        
        // Reset action after passing
        robot_action = RobotAction::MOVE;
    }
}