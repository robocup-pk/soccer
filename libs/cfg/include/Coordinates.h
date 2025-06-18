#ifndef COORDINATES_H
#define COORDINATES_H

namespace cfg {

struct Coordinates {
  //    |-----> (+x)
  //    |
  //    |
  //    v (+y)

  static constexpr float field_width_ft = 18;
  static constexpr float field_height_ft = 11;

  static constexpr int window_width_px = 1040;
  static constexpr int window_height_px = 740;

  static constexpr float px_per_ft = window_width_px / field_width_ft;

  static constexpr glm::vec2 ft_to_px_coords = glm::vec2(1, -1);
};

struct SystemConfig {
  static constexpr int num_robots = 2;

  static constexpr float wall_velocity_damping_factor = 0.5;

  static constexpr glm::vec2 init_ball_velocity_ftps = glm::vec2(2, 2);
  static constexpr glm::vec2 init_ball_acceleration_ftpsps = glm::vec2(-7, -7);
  static constexpr float ball_radius_ft = 0.4;

  static constexpr glm::vec2 robot_size_ft = glm::vec2(1.0, 1.0);
  static constexpr float init_robot_speed_ftps = 3;
  static constexpr float max_robot_speed_ftps = 7.5;
  static constexpr glm::vec2 init_robot_acceleration_ftpsps = glm::vec2(0, 0);

  // Kick Speed: Speed when ball is released
  static constexpr float kick_speed_ftps = 15.0f;
};

}  // namespace cfg

#endif  // COORDINATES_H