#include "ActionExecutor.h"
#include "BallModel.h"
#include <cmath>
#include <iostream>

namespace kin {

bool ActionUtils::IsWithinRange(const state::SoccerObject& robot, const state::SoccerObject& ball, 
                               double max_distance) {
    return CalculateDistance(robot, ball) <= max_distance;
}

bool ActionUtils::IsRobotFacingBall(const state::SoccerObject& robot, const state::SoccerObject& ball,
                                   double angle_tolerance) {
    Eigen::Vector3d robot_center = const_cast<state::SoccerObject&>(robot).GetCenterPosition();
    Eigen::Vector3d ball_center = const_cast<state::SoccerObject&>(ball).GetCenterPosition();
    
    // Calculate angle from robot to ball
    Eigen::Vector3d robot_to_ball = ball_center - robot_center;
    robot_to_ball.z() = 0;  // 2D only
    double ball_angle = atan2(robot_to_ball.y(), robot_to_ball.x());
    double robot_angle = robot.position[2];
    
    // Normalize angles to [-pi, pi]
    auto normalize_angle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };
    
    double angle_diff = normalize_angle(ball_angle - robot_angle);
    return std::abs(angle_diff) < angle_tolerance;
}

double ActionUtils::CalculateDistance(const state::SoccerObject& obj1, const state::SoccerObject& obj2) {
    Eigen::Vector3d pos1 = const_cast<state::SoccerObject&>(obj1).GetCenterPosition();
    Eigen::Vector3d pos2 = const_cast<state::SoccerObject&>(obj2).GetCenterPosition();
    
    return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2));
}

// ObjectFinder implementation
state::SoccerObject* ObjectFinder::FindRobot(std::vector<state::SoccerObject>& objects, 
                                             const std::string& robot_name) {
    for (auto& obj : objects) {
        if (obj.name == robot_name) {
            return &obj;
        }
    }
    return nullptr;
}

state::SoccerObject* ObjectFinder::FindBall(std::vector<state::SoccerObject>& objects) {
    for (auto& obj : objects) {
        if (obj.name == "ball") {
            return &obj;
        }
    }
    return nullptr;
}

bool ObjectFinder::FindRobotAndBall(std::vector<state::SoccerObject>& objects,
                                   state::SoccerObject*& robot, state::SoccerObject*& ball,
                                   const std::string& robot_name) {
    robot = FindRobot(objects, robot_name);
    ball = FindBall(objects);
    return robot != nullptr && ball != nullptr;
}

}  // namespace kin