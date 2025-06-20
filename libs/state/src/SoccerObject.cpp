#include "Coordinates.h"
#include "SoccerObject.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  soccer_objects.push_back(state::SoccerObject(
      "ball",
      Eigen::Vector3d(-cfg::SystemConfig::ball_radius_ft, cfg::SystemConfig::ball_radius_ft, 0),
      Eigen::Vector2d(cfg::SystemConfig::ball_radius_ft * 2,
                      cfg::SystemConfig::ball_radius_ft * 2),
      cfg::SystemConfig::init_ball_velocity_ftps, cfg::SystemConfig::init_ball_acceleration_ftpsps,
      1));

  // Robots
  for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_ft(i * 3, 0, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_ft, cfg::SystemConfig::robot_size_ft,
                            cfg::SystemConfig::init_robot_velocity_ftps,
                            cfg::SystemConfig::init_robot_acceleration_ftpsps, 10));
  }
}