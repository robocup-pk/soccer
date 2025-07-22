#include "RobotManager.h"
#include "Kinematics.h"
#include <cmath>

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
        } else if (obj.name == "robot_0") {  // Assuming this robot is robot_0
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
        } else if (obj.name == "robot_0") {  // Assuming this robot is robot_0
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