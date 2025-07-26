#include "Kick.h"
#include "Kinematics.h"
#include "BallModel.h"
#include "Utils.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace kin {

bool Kick(state::SoccerObject& robot, state::SoccerObject& ball, double kick_power, bool force_kick) {
  // SSL RoboCup kick constraints:
  // 1. Robot must be facing the ball (orientation check)
  // 2. Ball must be within kicking range
  // 3. Kick power limited to SSL regulations (max ~6 m/s)
  // 4. Robot can only kick when ball is in front sector
  
  // Get robot and ball center positions
  Eigen::Vector3d robot_center = robot.GetCenterPosition();
  Eigen::Vector3d ball_center = ball.GetCenterPosition();
  
  // Calculate distance between robot and ball
  double distance = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                             std::pow(ball_center.y() - robot_center.y(), 2));
  
  // SSL constraint 1: Ball must be within kicking range (typically 20-30cm)
  if (distance > ssl::MAX_KICK_DISTANCE && !force_kick) {
    std::cout << "[Kick] Ball too far away: " << distance << "m (max: " << ssl::MAX_KICK_DISTANCE << "m)" << std::endl;
    return false;
  }
  
  // SSL constraint 2: Robot must be facing the ball (ball in front sector)
  Eigen::Vector2d ball_point(ball_center.x(), ball_center.y());
  if (!robot.IsPointInFrontSector(ball_point) && !force_kick) {
    std::cout << "[Kick] Ball not in robot's front sector - robot must face the ball to kick" << std::endl;
    return false;
  }
  
  // SSL constraint 3: Limit kick power to SSL regulations
  double actual_kick_power = std::min(kick_power, ssl::MAX_KICK_POWER);
  
  if (kick_power > ssl::MAX_KICK_POWER) {
    std::cout << "[Kick] Warning: Kick power limited from " << kick_power 
              << " to " << actual_kick_power << " m/s (SSL max)" << std::endl;
  }
  
  // Calculate kick direction based on robot's orientation
  double robot_angle = robot.position[2]; // Robot's orientation angle
  Eigen::Vector2d kick_direction(std::cos(robot_angle), std::sin(robot_angle));
  
  std::cout << "[Kick] Executing SSL kick - Power: " << actual_kick_power 
            << " m/s, Direction: (" << kick_direction.x() << ", " << kick_direction.y() 
            << "), Distance: " << distance << "m" << std::endl;
  
  // Apply the kick using existing ApplyKickToBall function
  ApplyKickToBall(ball, kick_direction, actual_kick_power);
  
  // SSL behavior: Brief pause after kick (robot can't immediately kick again)
  // This simulates the physical constraint of SSL robots
  std::this_thread::sleep_for(std::chrono::milliseconds(ssl::KICK_DELAY_MS));
  
  return true;
}

// Old ExecuteKick and ExecutePass functions removed
// Now handled by KickExecutor class with proper OOP architecture
// Backward compatibility maintained through inline functions in header

}  // namespace kin