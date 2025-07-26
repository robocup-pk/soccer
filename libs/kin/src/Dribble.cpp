#include "Dribble.h"
#include "BallModel.h"
#include "Kinematics.h"
#include "Utils.h"
#include <iostream>
#include <cmath>

namespace kin {

bool Dribble(state::SoccerObject& robot, state::Ball& ball, double dribble_power, bool continuous) {
    // Dribble logic based on robot and ball positions

    // Calculate desired ball position relative to robot (e.g., 8cm in front)
    double desired_ball_distance = 0.08; // 8cm
    Eigen::Vector2d robot_pos(robot.position.x(), robot.position.y());
    double robot_angle = robot.position.z();
    
    Eigen::Vector2d desired_ball_pos_world(
        robot_pos.x() + desired_ball_distance * cos(robot_angle),
        robot_pos.y() + desired_ball_distance * sin(robot_angle)
    );

    Eigen::Vector2d ball_pos(ball.position.x(), ball.position.y());
    
    // Calculate distance between robot and ball centers
    double distance = (robot_pos - ball_pos).norm();

    // Define maximum dribble distance (robot radius + ball radius + small offset)
    double max_dribble_distance = robot.radius_m + ball.radius_m + 0.1;

    // Only dribble if ball is within range
    if (distance > max_dribble_distance) {
        return false; // Ball too far to dribble
    }

    Eigen::Vector2d force_direction = desired_ball_pos_world - ball_pos;
    
    // Apply force to ball
    ball.ApplyDribbleForce(ball.velocity, force_direction.normalized() * dribble_power);

    // Set dribbling state for compliance monitoring
    robot.is_dribbling = true;
    ball.is_attached = false; // Ensure ball is not attached during dribbling

    return true; // Always succeeds in this simplified model
}

// Old ExecuteDribble function removed
// Now handled by DribbleExecutor class with proper OOP architecture
// Backward compatibility maintained through inline function in header

}  // namespace kin