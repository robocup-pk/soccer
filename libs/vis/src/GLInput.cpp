#include "GLCallback.h"
#include "GLWindow.h"

void vis::GLWindow::ProcessInput(float dt) {
  // Calculate movement speed (with acceleration for held keys)
  static float speed_multiplier = 1.0f;
  static float move_speed = GLConfig::init_robot_speed;
  static float rotation_speed = GLConfig::init_robot_rotation_speed;

  // Check if any movement keys are pressed
  bool any_movement_key = GLCallback::keys[GLFW_KEY_W] || GLCallback::keys[GLFW_KEY_S] ||
                          GLCallback::keys[GLFW_KEY_A] || GLCallback::keys[GLFW_KEY_D] ||
                          GLCallback::keys[GLFW_KEY_UP] || GLCallback::keys[GLFW_KEY_DOWN] ||
                          GLCallback::keys[GLFW_KEY_LEFT] || GLCallback::keys[GLFW_KEY_RIGHT];

  if (any_movement_key) {
    // Increase speed for held keys (acceleration)
    move_speed = std::min(GLConfig::max_robot_speed, move_speed + 10);
  } else {
    move_speed = GLConfig::init_robot_speed;
  }

  game_objects["robot0"].velocity = glm::vec2(0, 0);
  game_objects["robot1"].velocity = glm::vec2(0, 0);

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

  // Robot 1 movement (Arrow keys, UP, DOWN, LEFT, RIGHT)
  if (GLCallback::keys[GLFW_KEY_UP]) {
    GLCallback::y_offset_robot1 = -move_speed;
    game_objects["robot1"].velocity.y = -move_speed;
  }
  if (GLCallback::keys[GLFW_KEY_DOWN]) {
    GLCallback::y_offset_robot1 = move_speed;
    game_objects["robot1"].velocity.y = move_speed;
  }
  if (GLCallback::keys[GLFW_KEY_LEFT]) {
    GLCallback::x_offset_robot1 = -move_speed;
    game_objects["robot1"].velocity.x = -move_speed;
  }
  if (GLCallback::keys[GLFW_KEY_RIGHT]) {
    GLCallback::x_offset_robot1 = move_speed;
    game_objects["robot1"].velocity.x = move_speed;
  }

  // Robot 0 rotation (C, V)
  if (GLCallback::keys[GLFW_KEY_C]) {
    game_objects["robot0"].rotation += rotation_speed * dt;
  }
  if (GLCallback::keys[GLFW_KEY_V]) {
    game_objects["robot0"].rotation -= rotation_speed * dt;
  }

  // Robot 1 rotation (Z,X)
  if (GLCallback::keys[GLFW_KEY_Z]) {
    game_objects["robot1"].rotation += rotation_speed * dt;
  }
  if (GLCallback::keys[GLFW_KEY_X]) {
    game_objects["robot1"].rotation -= rotation_speed * dt;
  }

  // Handle Kick Input
  static bool k_key_was_pressed = false;
  bool k_key_is_pressed = GLCallback::keys[GLFW_KEY_K];

  // Handle K key press (only trigger on key press, not hold)
  if (k_key_is_pressed && !k_key_was_pressed) {
    // Check if ball exists
    if (game_objects.find("ball") != game_objects.end()) {
      GameObject& ball = game_objects["ball"];

      // Find which robot has the ball attached and kick it
      for (auto& [name, robot] : game_objects) {
        if (name.find("robot") != std::string::npos && robot.has_ball_attached) {
          robot.KickBall(ball);
          break;  // Only one robot can have the ball at a time
        }
      }
    }
  }

  k_key_was_pressed = k_key_is_pressed;
}