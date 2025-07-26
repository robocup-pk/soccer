#include "KickExecutor.h"
#include "Kinematics.h"
#include "BallModel.h"
#include <iostream>
#include <cmath>

namespace kin {

// KickValidator implementation
bool KickValidator::ValidateKick(const ActionContext& context) const {
    // Check if ball is within kicking range
    if (!ActionUtils::IsWithinRange(context.robot, context.ball, config::DEFAULT_ACTION_DISTANCE) && 
        !context.force_execute) {
        std::cout << "[KickValidator] Ball too far for kick action" << std::endl;
        return false;
    }
    
    // Check if robot is facing the ball (SSL requirement)
    Eigen::Vector2d ball_pos(context.ball.position.x(), context.ball.position.y());
    if (!context.robot.IsPointInFrontSector(ball_pos) && !context.force_execute) {
        std::cout << "[KickValidator] Ball not in robot's front sector" << std::endl;
        return false;
    }
    
    return true;
}

bool KickValidator::ValidatePass(const ActionContext& context, const Eigen::Vector2d& target) const {
    // Same validation as kick for now, but could be different
    return ValidateKick(context);
}

// KickEngine implementation
bool KickEngine::ExecuteKick(const ActionContext& context) const {
    // Calculate kick direction based on robot orientation
    double robot_angle = context.robot.position[2];
    Eigen::Vector2d kick_direction(std::cos(robot_angle), std::sin(robot_angle));
    
    // Use provided power or default
    double kick_power = (context.power > 0) ? context.power : config::KICK_POWER_DEFAULT;
    
    std::cout << "[KickEngine] Executing kick with power: " << kick_power << " m/s" << std::endl;
    
    // Apply the kick using existing kinematics function
    kin::ApplyKickToBall(context.ball, kick_direction, kick_power);
    
    return true;
}

bool KickEngine::ExecutePass(const ActionContext& context, const Eigen::Vector2d& target) const {
    Eigen::Vector3d robot_center = context.robot.GetCenterPosition();
    
    // Calculate pass direction toward target
    Eigen::Vector2d pass_direction = target - Eigen::Vector2d(robot_center.x(), robot_center.y());
    pass_direction.normalize();
    
    // Use provided power or default pass power
    double pass_power = (context.power > 0) ? context.power : config::PASS_POWER_DEFAULT;
    
    std::cout << "[KickEngine] Executing pass with power: " << pass_power << " m/s" << std::endl;
    
    // Apply the pass using existing kinematics function
    kin::ApplyKickToBall(context.ball, pass_direction, pass_power);
    
    return true;
}

// KickExecutor implementation
bool KickExecutor::Execute(const ActionContext& context) {
    if (!validator.ValidateKick(context)) {
        return false;
    }
    
    return engine.ExecuteKick(context);
}

bool KickExecutor::ExecutePass(const ActionContext& context, const Eigen::Vector2d& target_position) {
    if (!validator.ValidatePass(context, target_position)) {
        return false;
    }
    
    return engine.ExecutePass(context, target_position);
}

// Static convenience methods for backward compatibility
bool KickExecutor::ExecuteKickAction(std::vector<state::SoccerObject>& soccer_objects) {
    state::SoccerObject* robot = nullptr;
    state::SoccerObject* ball = nullptr;
    
    if (!ObjectFinder::FindRobotAndBall(soccer_objects, robot, ball)) {
        std::cout << "[KickExecutor] Could not find robot or ball" << std::endl;
        return false;
    }
    
    ActionContext context(*robot, *ball);
    context.power = config::KICK_POWER_DEFAULT;
    
    KickExecutor executor;
    return executor.Execute(context);
}

bool KickExecutor::ExecutePassAction(std::vector<state::SoccerObject>& soccer_objects, 
                                    const Eigen::Vector2d& target_position) {
    state::SoccerObject* robot = nullptr;
    state::SoccerObject* ball = nullptr;
    
    if (!ObjectFinder::FindRobotAndBall(soccer_objects, robot, ball)) {
        std::cout << "[KickExecutor] Could not find robot or ball for pass" << std::endl;
        return false;
    }
    
    ActionContext context(*robot, *ball);
    context.power = config::PASS_POWER_DEFAULT;
    context.target_position = target_position;
    
    KickExecutor executor;
    return executor.ExecutePass(context, target_position);
}

}  // namespace kin