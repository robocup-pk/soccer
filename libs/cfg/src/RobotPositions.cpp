#include "RobotPositions.h"

namespace cfg {

// Right side team (faces right / team_two)
const std::map<RobotHomePosition, Eigen::Vector3d> RightRobotHomeCoordinates = {
    {RobotHomePosition::GOALKEEPER, Eigen::Vector3d(0.5f, 0.0f, M_PI)},
    {RobotHomePosition::LEFT_BACK, Eigen::Vector3d(0.7f, -0.3f, M_PI)},
    {RobotHomePosition::RIGHT_BACK, Eigen::Vector3d(0.7f, 0.3f, M_PI)},
    {RobotHomePosition::LEFT_FORWARD, Eigen::Vector3d(0.3f, -0.3f, M_PI)},
    {RobotHomePosition::CENTER_FORWARD, Eigen::Vector3d(0.3f, 0.3f, M_PI)},
    {RobotHomePosition::RIGHT_FORWARD, Eigen::Vector3d(0.9f, 0.0f, M_PI)}};

// Left side team (faces left / team_one)
const std::map<RobotHomePosition, Eigen::Vector3d> LeftRobotHomeCoordinates = {
    {RobotHomePosition::GOALKEEPER, Eigen::Vector3d(-0.5f, 0.0f, 0.0f)},
    {RobotHomePosition::LEFT_BACK, Eigen::Vector3d(-0.7f, -0.3f, 0.0f)},
    {RobotHomePosition::RIGHT_BACK, Eigen::Vector3d(-0.7f, 0.3f, 0.0f)},
    {RobotHomePosition::LEFT_FORWARD, Eigen::Vector3d(-0.3f, -0.3f, 0.0f)},
    {RobotHomePosition::CENTER_FORWARD, Eigen::Vector3d(-0.3f, 0.3f, 0.0f)},
    {RobotHomePosition::RIGHT_FORWARD, Eigen::Vector3d(-0.9f, 0.0f, 0.0f)}};
}  // namespace cfg