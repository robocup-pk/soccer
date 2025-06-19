#include "GLCallback.h"
#include "GLSimulation.h"

void vis::GLSimulation::ProcessInput(float dt) {
  // Calculate movement speed (with acceleration for held keys)
  static float speed_multiplier = 1.0f;
  static float move_speed = cfg::SystemConfig::init_robot_speed_ftps * cfg::Coordinates::px_per_ft;

  // Check if any movement keys are pressed
  bool any_movement_key = GLCallback::keys[GLFW_KEY_W] || GLCallback::keys[GLFW_KEY_S] ||
                          GLCallback::keys[GLFW_KEY_A] || GLCallback::keys[GLFW_KEY_D] ||
                          GLCallback::keys[GLFW_KEY_UP] || GLCallback::keys[GLFW_KEY_DOWN] ||
                          GLCallback::keys[GLFW_KEY_LEFT] || GLCallback::keys[GLFW_KEY_RIGHT];

  if (any_movement_key) {
    // Increase speed for held keys (acceleration)
    move_speed = std::min(cfg::SystemConfig::max_robot_speed_ftps * cfg::Coordinates::px_per_ft,
                          move_speed + 10);
  } else {
    move_speed = cfg::SystemConfig::init_robot_speed_ftps * cfg::Coordinates::px_per_ft;
  }

  if (game_objects.find("robot0") != game_objects.end()) {
    game_objects["robot0"].velocity = glm::vec3(0, 0, 0);
    // Robot 0 movement (WASD)
    if (GLCallback::keys[GLFW_KEY_W]) {
      game_objects["robot0"].velocity.y = -move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_S]) {
      game_objects["robot0"].velocity.y = move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_A]) {
      game_objects["robot0"].velocity.x = -move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_D]) {
      game_objects["robot0"].velocity.x = move_speed;
    }
  }

  // Robot 1 movement (Arrow keys, UP, DOWN, LEFT, RIGHT)
  if (game_objects.find("robot1") != game_objects.end()) {
    game_objects["robot1"].velocity = glm::vec3(0, 0, 0);
    if (GLCallback::keys[GLFW_KEY_UP]) {
      game_objects["robot1"].velocity.y = -move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_DOWN]) {
      game_objects["robot1"].velocity.y = move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_LEFT]) {
      game_objects["robot1"].velocity.x = -move_speed;
    }
    if (GLCallback::keys[GLFW_KEY_RIGHT]) {
      game_objects["robot1"].velocity.x = move_speed;
    }
  }
}