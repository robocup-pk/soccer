#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <vector>
#include <cmath>

namespace kin {

/*
  This struct describes the robot shape relating to kinematics.
*/
struct RobotDescription {
  static constexpr int num_wheels = 4;
  static constexpr double wheel_radius_m = 0.03225;  // 0.05; Wheel Diameter 64.5 mm

  // Angles between robot frame x-axis and wheel frame x-axis
  std::vector<double> wheel_angles_rad;

  // Positions of wheel centers from center of robot frame, in robot frame
  std::vector<std::pair<double, double>> wheel_positions_m;

  RobotDescription() {
    // Square configuration with wheels at corners
    // wheel_positions_m = {
    //     {0.15, 0.15},    // wheel 1: front-left
    //     {-0.15, 0.15},   // wheel 2: rear-left
    //     {-0.15, -0.15},  // wheel 3: rear-right
    //     {0.15, -0.15}    // wheel 4: front-right
    // };

    // Wheel angles (perpendicular to radial direction for typical omniwheel setup)
    // wheel_angles_rad = {
    //     -M_PI / 4,     // -45° (wheel 1)
    //     M_PI / 4,      // 45° (wheel 2)
    //     3 * M_PI / 4,  // 135° (wheel 3)
    //     -3 * M_PI / 4  // -135° (wheel 4)
    // };

    wheel_angles_rad = {-30 * M_PI / 180, 45 * M_PI / 180, 135 * M_PI / 180, -150 * M_PI / 180};

    wheel_positions_m = {{0.045601, 0.080113},
                         {-0.064798, 0.065573},
                         {-0.064798, -0.065573},
                         {0.045601, -0.080113}};

    // wheel_angles_rad = {-0.5323254, 0.7766715, 2.3649211, -2.6092672};

    // wheel_positions_m = {
    //     {0.039634, 0.067378}, {-0.055, 0.055722}, {-0.055, -0.055722}, {0.039634, -0.067378}};
  }
};

// Singleton accessor
inline const RobotDescription& GetRobotDescription() {
  static RobotDescription instance;
  return instance;
}

}  // namespace kin

#endif  // ROBOT_DESCRIPTION_H