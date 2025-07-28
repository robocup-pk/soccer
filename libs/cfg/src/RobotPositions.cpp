#include "RobotPositions.h"

namespace cfg {

const std::map<RobotHomePosition, Eigen::Vector3d> RobotHomeCoordinates = {
    {RobotHomePosition::GOALKEEPER, Eigen::Vector3d(2.0, 0.0, 0.0)},
    {RobotHomePosition::LEFT_BACK, Eigen::Vector3d(1.5, -1, 0.0)},
    {RobotHomePosition::CENTER_BACK, Eigen::Vector3d(1.5, 0, 0.0)},
    {RobotHomePosition::RIGHT_BACK, Eigen::Vector3d(1.5, 1, 0.0)},
    {RobotHomePosition::LEFT_FORWARD, Eigen::Vector3d(0.5, -1.0, 0.0)},
    {RobotHomePosition::CENTER_FORWARD, Eigen::Vector3d(0.0, 0.0, 0.0)},
    {RobotHomePosition::RIGHT_FORWARD, Eigen::Vector3d(0.5, 1.0, 0.0)}};
}