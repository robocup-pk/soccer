#include "Dribble.h"
#include "SoccerObject.h"
#include "BallModel.h"
#include <cmath>
#include <iostream>

namespace kin {

bool Dribble(state::SoccerObject& robot, state::SoccerObject& ball, double power, bool continuous) {
    // Calculate vector from robot to ball
    Eigen::Vector3d robot_center = robot.GetCenterPosition();
    Eigen::Vector3d ball_center = ball.GetCenterPosition();
    Eigen::Vector2d robot_to_ball(ball_center.x() - robot_center.x(), 
                                  ball_center.y() - robot_center.y());
    
    double distance = robot_to_ball.norm();
    
    // Effective dribbling range (approximately robot radius + ball radius + small buffer)
    double max_dribble_distance = robot.radius_m + ball.radius_m + 0.1; // 10cm buffer
    
    // Only apply dribble force if ball is within dribbling range
    if (distance > max_dribble_distance) {
        return false;  // Ball too far to dribble
    }
    
    // Calculate dribble force to keep ball close to robot front
    Eigen::Vector2d desired_ball_offset(0.08, 0.0); // 8cm in front of robot
    
    // Rotate desired offset by robot orientation
    double robot_theta = robot.position.z();
    Eigen::Vector2d rotated_offset;
    rotated_offset.x() = desired_ball_offset.x() * cos(robot_theta) - desired_ball_offset.y() * sin(robot_theta);
    rotated_offset.y() = desired_ball_offset.x() * sin(robot_theta) + desired_ball_offset.y() * cos(robot_theta);
    
    // Desired ball position relative to robot
    Eigen::Vector2d desired_ball_pos(robot_center.x() + rotated_offset.x(), 
                                     robot_center.y() + rotated_offset.y());
    
    // Calculate force needed to move ball to desired position
    Eigen::Vector2d ball_error(desired_ball_pos.x() - ball_center.x(),
                               desired_ball_pos.y() - ball_center.y());
    
    // Scale force by power and limit maximum force
    Eigen::Vector2d dribble_force = ball_error * power;
    double max_force = 25.0; // Increased maximum dribble force for better control
    if (dribble_force.norm() > max_force) {
        dribble_force = dribble_force.normalized() * max_force;
    }
    
    // Apply dribble force to ball using BallModel if available
    if (auto* ball_model = dynamic_cast<kin::BallModel*>(&ball)) {
        ball_model->ApplyDribbleForce(ball.velocity, dribble_force);
        std::cout << "[DRIBBLE] Applied BallModel force: (" << dribble_force.x() << ", " << dribble_force.y() << ")" << std::endl;
    } else {
        // Fallback: directly modify ball velocity
        ball.velocity.x() += dribble_force.x() * 0.1;
        ball.velocity.y() += dribble_force.y() * 0.1;
        std::cout << "[DRIBBLE] Applied direct velocity: (" << dribble_force.x()*0.1 << ", " << dribble_force.y()*0.1 << ")" << std::endl;
    }
    
    return true;  // Dribble successful
}

}  // namespace kin