#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <vector>

namespace kin {

/*
  This struct describes the robot shape relating to kinematics.
*/
struct RobotDescription {
  static constexpr int num_wheels = 4;
  static constexpr double wheel_radius_m = 0.05;

  // Angles between robot frame x-axis and wheel frame x-axis
  std::vector<double> wheel_angles_rad;

  // Positions of wheel centers from center of robot frame, in robot frame
  std::vector<std::pair<double, double>> wheel_positions_m;

  RobotDescription() {
    // Square configuration with wheels at corners
    wheel_positions_m = {
        {0.15, 0.15},    // wheel 1: front-left
        {-0.15, 0.15},   // wheel 2: rear-left
        {-0.15, -0.15},  // wheel 3: rear-right
        {0.15, -0.15}    // wheel 4: front-right
    };

    // Wheel angles (perpendicular to radial direction for typical omniwheel setup)
    wheel_angles_rad = {
        -M_PI / 4,     // -45° (wheel 1)
        M_PI / 4,      // 45° (wheel 2)
        3 * M_PI / 4,  // 135° (wheel 3)
        -3 * M_PI / 4  // -135° (wheel 4)
    };
  }
};

// Singleton accessor
inline const RobotDescription& GetRobotDescription() {
  static RobotDescription instance;
  return instance;
}

}  // namespace kin

#endif  // ROBOT_DESCRIPTION_H