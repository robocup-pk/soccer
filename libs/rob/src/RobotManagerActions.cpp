#include "RobotManager.h"
#include "Kinematics.h"
#include "Kick.h"
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
    
    // Use the enhanced SSL RoboCup kick function
    double kick_power = 3.0;  // m/s kick velocity (SSL typical: 2-4 m/s)
    
    std::cout << "[ExecuteKickAction] Attempting SSL kick with power: " << kick_power << " m/s" << std::endl;
    
    // Store ball velocity before kick for comparison
    Eigen::Vector3d old_velocity = ball->velocity;
    
    // Use the new enhanced kick function with SSL constraints
    bool kick_successful = kin::Kick(*robot, *ball, kick_power, false);
    
    if (kick_successful) {
        std::cout << "[ExecuteKickAction] SSL kick successful! Ball velocity changed from (" 
                  << old_velocity.x() << ", " << old_velocity.y() << ") to (" 
                  << ball->velocity.x() << ", " << ball->velocity.y() << ")" << std::endl;
        
        // Reset action after successful kick
        robot_action = RobotAction::MOVE;
    } else {
        std::cout << "[ExecuteKickAction] SSL kick failed - constraints not met" << std::endl;
        // Don't reset action, robot will try again on next cycle
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
    
    // For passing, we temporarily override robot orientation to face target
    // Store original orientation
    double original_orientation = robot->position[2];
    
    // Calculate direction to target
    Eigen::Vector3d robot_center = robot->GetCenterPosition();
    Eigen::Vector2d pass_direction = target_position - Eigen::Vector2d(robot_center.x(), robot_center.y());
    double target_angle = std::atan2(pass_direction.y(), pass_direction.x());
    
    // Temporarily set robot orientation toward target
    robot->position[2] = target_angle;
    
    // Use enhanced kick function with pass power (lighter than kick)
    double pass_power = 2.0;  // m/s pass velocity (SSL typical pass speed)
    
    std::cout << "[ExecutePassAction] Attempting SSL pass toward target (" 
              << target_position.x() << ", " << target_position.y() << ") with power: " 
              << pass_power << " m/s" << std::endl;
    
    // Use the new enhanced kick function with SSL constraints
    bool pass_successful = kin::Kick(*robot, *ball, pass_power, false);
    
    // Restore original orientation
    robot->position[2] = original_orientation;
    
    if (pass_successful) {
        std::cout << "[ExecutePassAction] SSL pass successful!" << std::endl;
        // Reset action after successful pass
        robot_action = RobotAction::MOVE;
    } else {
        std::cout << "[ExecutePassAction] SSL pass failed - constraints not met" << std::endl;
        // Don't reset action, robot will try again on next cycle
    }
}