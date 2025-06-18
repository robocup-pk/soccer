#include "GLCallback.h"
#include "GLWindow.h"
#include <iostream>

void vis::GLWindow::ProcessInput(float dt) {
  // Toggle movement system with SPACE key
  static bool space_pressed_last_frame = false;
  bool space_pressed_this_frame = GLCallback::keys[GLFW_KEY_SPACE];
  
  if (space_pressed_this_frame && !space_pressed_last_frame) {
    if (current_movement_mode == RRT_MOVEMENT && intelligent_movement) {
      bool current_mode = intelligent_movement->IsAutoModeEnabled();
      intelligent_movement->SetAutoMode(!current_mode);
      std::cout << "[GLWindow] RRT Movement " << (!current_mode ? "ENABLED" : "DISABLED") << std::endl;
    } else if (current_movement_mode == INTERCEPT_MOVEMENT && intelligent_movement2) {
      bool current_mode = intelligent_movement2->IsAutoModeEnabled();
      intelligent_movement2->SetAutoMode(!current_mode);
      std::cout << "[GLWindow] Ball Intercept Movement " << (!current_mode ? "ENABLED" : "DISABLED") << std::endl;
    }
  }
  space_pressed_last_frame = space_pressed_this_frame;
  
  // Switch to RRT Movement (key '1')
  static bool key1_pressed_last_frame = false;
  bool key1_pressed_this_frame = GLCallback::keys[GLFW_KEY_1];
  
  if (key1_pressed_this_frame && !key1_pressed_last_frame) {
    current_movement_mode = RRT_MOVEMENT;
    // Disable both systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to RRT Movement (obstacle avoidance)" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key1_pressed_last_frame = key1_pressed_this_frame;
  
  // Switch to Ball Intercept Movement (key '2')
  static bool key2_pressed_last_frame = false;
  bool key2_pressed_this_frame = GLCallback::keys[GLFW_KEY_2];
  
  if (key2_pressed_this_frame && !key2_pressed_last_frame) {
    current_movement_mode = INTERCEPT_MOVEMENT;
    // Disable both systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to Ball Intercept Movement (strategic interception)" << std::endl;
    std::cout << "  robot0 will strategically intercept ball while avoiding robot1" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key2_pressed_last_frame = key2_pressed_this_frame;
  
  // Only process manual input if no movement system is active
  bool any_movement_active = false;
  if (intelligent_movement && intelligent_movement->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  if (intelligent_movement2 && intelligent_movement2->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  
  if (any_movement_active) {
    return; // Skip manual controls when any movement system is active
  }

  // Calculate movement speed (with acceleration for held keys)
  static float speed_multiplier = 1.0f;
  static float move_speed = GLConfig::init_robot_speed;

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
}