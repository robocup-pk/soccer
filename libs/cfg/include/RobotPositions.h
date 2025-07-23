#ifndef ROBOT_POSITIONS_H
#define ROBOT_POSITIONS_H

#include <Eigen/Dense>
#include <map>

namespace cfg {

enum class RobotHomePosition {
  GOALKEEPER,
  LEFT_BACK,
  CENTER_BACK,
  RIGHT_BACK,
  LEFT_FORWARD,
  CENTER_FORWARD,
  RIGHT_FORWARD
};

extern const std::map<RobotHomePosition, Eigen::Vector3d> RobotHomeCoordinates;

}  // namespace cfg

#endif  // ROBOT_POSITIONS_H