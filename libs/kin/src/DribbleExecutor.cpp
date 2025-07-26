#include "DribbleExecutor.h"
#include "Dribble.h"
#include "Utils.h"
#include <iostream>
#include <cmath>

namespace kin {

// DribbleValidator implementation
bool DribbleValidator::ValidateDribble(const ActionContext& context) const {
    // Check if ball is within dribbling range
    if (!ActionUtils::IsWithinRange(context.robot, context.ball, config::DEFAULT_ACTION_DISTANCE)) {
        return false;  // Silent fail for dribble - it's continuous
    }
    
    // SSL RULE: Check if robot is facing the ball
    if (!ActionUtils::IsRobotFacingBall(context.robot, context.ball, config::ANGLE_TOLERANCE_RAD)) {
        static double last_warning = 0;
        double current_time = util::GetCurrentTime();
        if (current_time - last_warning > 2.0) {
            std::cout << "[SSL RULE] Robot not facing ball. Dribbling disabled." << std::endl;
            last_warning = current_time;
        }
        return false;
    }
    
    return true;
}

// DribbleEngine implementation
bool DribbleEngine::ExecuteDribble(const ActionContext& context) const {
    // Cast to Ball for dribble-specific operations
    state::Ball* ball_ptr = dynamic_cast<state::Ball*>(&context.ball);
    if (!ball_ptr) {
        std::cout << "[DribbleEngine] Ball object is not of correct type" << std::endl;
        return false;
    }
    
    // SSL RULE: Set ball to dribbling mode (physics-based, not attached)
    context.ball.is_dribbling = true;
    
    // Use provided power or default
    double dribble_power = (context.power > 0) ? context.power : config::DRIBBLE_POWER_DEFAULT;
    
    // Apply continuous dribbling force using existing Dribble function
    bool success = kin::Dribble(context.robot, *ball_ptr, dribble_power, true);
    
    if (success) {
        ApplyVelocityMatchingForce(context);
        
        // Check if ball is falling behind and apply pull force
        double distance = ActionUtils::CalculateDistance(context.robot, context.ball);
        if (distance > 0.2) {  // Ball getting too far
            ApplyPullForce(context, distance);
        }
    }
    
    // Ensure ball is not attached (SSL rule compliance)
    if (context.ball.is_attached) {
        context.ball.is_attached = false;
        context.ball.attached_to = nullptr;
    }
    
    return success;
}

void DribbleEngine::ApplyVelocityMatchingForce(const ActionContext& context) const {
    // Get robot velocity to help ball follow robot's movement
    Eigen::Vector3d robot_velocity = context.robot.velocity;
    double robot_speed = std::sqrt(robot_velocity.x()*robot_velocity.x() + robot_velocity.y()*robot_velocity.y());
    
    // If robot is moving, apply velocity-matching force to ball
    if (robot_speed > 0.05) {  // Robot is moving
        Eigen::Vector2d velocity_match_force;
        velocity_match_force.x() = robot_velocity.x() * 3.0;  // Match robot velocity
        velocity_match_force.y() = robot_velocity.y() * 3.0;
        
        // Apply velocity matching force to keep ball moving with robot
        if (auto* ball_model = dynamic_cast<state::Ball*>(&context.ball)) {
            ball_model->ApplyDribbleForce(context.ball.velocity, velocity_match_force);
        } else {
            context.ball.velocity.x() += velocity_match_force.x() * 0.3;
            context.ball.velocity.y() += velocity_match_force.y() * 0.3;
        }
    }
}

void DribbleEngine::ApplyPullForce(const ActionContext& context, double distance) const {
    // Calculate direction from ball to robot desired position
    Eigen::Vector3d robot_center = context.robot.GetCenterPosition();
    Eigen::Vector3d ball_center = context.ball.GetCenterPosition();
    
    Eigen::Vector3d robot_front = robot_center;
    robot_front.x() += 0.08 * cos(context.robot.position.z());  // 8cm in front of robot
    robot_front.y() += 0.08 * sin(context.robot.position.z());
    
    Eigen::Vector2d pull_force;
    pull_force.x() = (robot_front.x() - ball_center.x()) * 20.0;  // Strong pull force
    pull_force.y() = (robot_front.y() - ball_center.y()) * 20.0;
    
    // Apply pull force to bring ball closer
    if (auto* ball_model = dynamic_cast<state::Ball*>(&context.ball)) {
        ball_model->ApplyDribbleForce(context.ball.velocity, pull_force);
    } else {
        context.ball.velocity.x() += pull_force.x() * 0.1;
        context.ball.velocity.y() += pull_force.y() * 0.1;
    }
    
    std::cout << "[DribbleEngine] Applying pull force (distance=" << distance << "m)" << std::endl;
}

// DribbleExecutor implementation
bool DribbleExecutor::Execute(const ActionContext& context) {
    if (!validator.ValidateDribble(context)) {
        // Not dribbling - clear dribbling flag
        context.ball.is_dribbling = false;
        return false;
    }
    
    return engine.ExecuteDribble(context);
}

// Static convenience method for backward compatibility
bool DribbleExecutor::ExecuteDribbleAction(std::vector<state::SoccerObject>& soccer_objects) {
    state::SoccerObject* robot = nullptr;
    state::SoccerObject* ball = nullptr;
    
    if (!ObjectFinder::FindRobotAndBall(soccer_objects, robot, ball)) {
        return false;  // Silent fail for dribble
    }
    
    ActionContext context(*robot, *ball);
    context.power = config::DRIBBLE_POWER_DEFAULT;
    
    DribbleExecutor executor;
    return executor.Execute(context);
}

}  // namespace kin