#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <vector>

namespace kin {

/*
  This struct describes the robot shape relating to kinematics.
*/
struct RobotDescription {
  // Radius of the wheel
  double wheel_radius_m;

  // Angles between robot frame x-axis and wheel frame x-axis
  std::vector<double> wheel_angles_rad;

  // Positions of wheel centers from center of robot frame, in robot frame
  std::vector<std::pair<double, double>> wheel_positions_m;
};

}  // namespace kin

#endif  // ROBOT_DESCRIPTION_H