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
    } else if (current_movement_mode == MULTIPLAYER_MOVEMENT && multi_player_movement) {
      bool current_mode = multi_player_movement->IsAutoModeEnabled();
      multi_player_movement->SetAutoMode(!current_mode);
      std::cout << "[GLWindow] Multi-Player Movement " << (!current_mode ? "ENABLED" : "DISABLED") << std::endl;
      if (!current_mode) {
        multi_player_movement->PrintStatus();
      }
    } else if (current_movement_mode == MULTIPLAYER_RRT && multi_player_rrt) {
      bool current_mode = multi_player_rrt->IsAutoModeEnabled();
      multi_player_rrt->SetAutoMode(!current_mode);
      std::cout << "[GLWindow] Multi-Player RRT Movement " << (!current_mode ? "ENABLED" : "DISABLED") << std::endl;
      if (!current_mode) {
        multi_player_rrt->PrintStatus();
      }
    } else if (current_movement_mode == TEAM_COMPETITION && team_competition) {
      bool current_mode = team_competition->IsAutoModeEnabled();
      team_competition->SetAutoMode(!current_mode);
      std::cout << "[GLWindow] Team Competition " << (!current_mode ? "ENABLED" : "DISABLED") << std::endl;
      if (!current_mode) {
        team_competition->PrintStatus();
      }
    }
  }
  space_pressed_last_frame = space_pressed_this_frame;
  
  // Switch to RRT Movement (key '1')
  static bool key1_pressed_last_frame = false;
  bool key1_pressed_this_frame = GLCallback::keys[GLFW_KEY_1];
  
  if (key1_pressed_this_frame && !key1_pressed_last_frame) {
    current_movement_mode = RRT_MOVEMENT;
    // Disable all systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    if (multi_player_movement) multi_player_movement->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to RRT Movement (obstacle avoidance)" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key1_pressed_last_frame = key1_pressed_this_frame;
  
  // Switch to Ball Intercept Movement (key '2')
  static bool key2_pressed_last_frame = false;
  bool key2_pressed_this_frame = GLCallback::keys[GLFW_KEY_2];
  
  if (key2_pressed_this_frame && !key2_pressed_last_frame) {
    current_movement_mode = INTERCEPT_MOVEMENT;
    // Disable all systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    if (multi_player_movement) multi_player_movement->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to Ball Intercept Movement (strategic interception)" << std::endl;
    std::cout << "  robot0 will strategically intercept ball while avoiding robot1" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key2_pressed_last_frame = key2_pressed_this_frame;
  
  // Switch to Multi-Player Movement (key '3')
  static bool key3_pressed_last_frame = false;
  bool key3_pressed_this_frame = GLCallback::keys[GLFW_KEY_3];
  
  if (key3_pressed_this_frame && !key3_pressed_last_frame) {
    current_movement_mode = MULTIPLAYER_MOVEMENT;
    // Disable all systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    if (multi_player_movement) multi_player_movement->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to Multi-Player Movement (team ball interception)" << std::endl;
    std::cout << "  robot0, robot2, robot3 will coordinate to intercept ball" << std::endl;
    std::cout << "  robot1, robot4-robot9 act as opponents" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key3_pressed_last_frame = key3_pressed_this_frame;
  
  // Switch to Multi-Player RRT Movement (key '4')
  static bool key4_pressed_last_frame = false;
  bool key4_pressed_this_frame = GLCallback::keys[GLFW_KEY_4];
  
  if (key4_pressed_this_frame && !key4_pressed_last_frame) {
    current_movement_mode = MULTIPLAYER_RRT;
    // Disable all systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    if (multi_player_movement) multi_player_movement->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to Multi-Player RRT Movement (team RRT navigation)" << std::endl;
    std::cout << "  robot0, robot2, robot3 will use RRT path planning to reach ball" << std::endl;
    std::cout << "  robot1, robot4-robot9 act as obstacles" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
  }
  key4_pressed_last_frame = key4_pressed_this_frame;
  
  // Switch to Team Competition (key '5')
  static bool key5_pressed_last_frame = false;
  bool key5_pressed_this_frame = GLCallback::keys[GLFW_KEY_5];
  
  if (key5_pressed_this_frame && !key5_pressed_last_frame) {
    current_movement_mode = TEAM_COMPETITION;
    // Disable all systems first
    if (intelligent_movement) intelligent_movement->SetAutoMode(false);
    if (intelligent_movement2) intelligent_movement2->SetAutoMode(false);
    if (multi_player_movement) multi_player_movement->SetAutoMode(false);
    if (multi_player_rrt) multi_player_rrt->SetAutoMode(false);
    if (team_competition) team_competition->SetAutoMode(false);
    std::cout << "[GLWindow] Switched to Team Competition (5v5 team vs team)" << std::endl;
    std::cout << "  Team A (robot0-4) vs Team B (robot5-9)" << std::endl;
    std::cout << "  Each team can use different AI strategies" << std::endl;
    std::cout << "  Press SPACE to enable this movement system" << std::endl;
    std::cout << "  Press T to toggle Team A strategy, Y to toggle Team B strategy" << std::endl;
  }
  key5_pressed_last_frame = key5_pressed_this_frame;
  
  // Team strategy switching keys (only when in TEAM_COMPETITION mode)
  if (current_movement_mode == TEAM_COMPETITION && team_competition) {
    // Toggle Team A strategy (key 'T')
    static bool keyT_pressed_last_frame = false;
    bool keyT_pressed_this_frame = GLCallback::keys[GLFW_KEY_T];
    
    if (keyT_pressed_this_frame && !keyT_pressed_last_frame) {
      auto current_strategy = team_competition->GetTeamAStrategy();
      auto new_strategy = (current_strategy == TeamCompetition::BALL_INTERCEPT) ? 
                         TeamCompetition::RRT_NAVIGATION : TeamCompetition::BALL_INTERCEPT;
      team_competition->SetTeamAStrategy(new_strategy);
      std::cout << "[TeamCompetition] Team A strategy switched to: " 
                << (new_strategy == TeamCompetition::BALL_INTERCEPT ? "Ball Intercept" : "RRT Navigation") << std::endl;
    }
    keyT_pressed_last_frame = keyT_pressed_this_frame;
    
    // Toggle Team B strategy (key 'Y')
    static bool keyY_pressed_last_frame = false;
    bool keyY_pressed_this_frame = GLCallback::keys[GLFW_KEY_Y];
    
    if (keyY_pressed_this_frame && !keyY_pressed_last_frame) {
      auto current_strategy = team_competition->GetTeamBStrategy();
      auto new_strategy = (current_strategy == TeamCompetition::BALL_INTERCEPT) ? 
                         TeamCompetition::RRT_NAVIGATION : TeamCompetition::BALL_INTERCEPT;
      team_competition->SetTeamBStrategy(new_strategy);
      std::cout << "[TeamCompetition] Team B strategy switched to: " 
                << (new_strategy == TeamCompetition::BALL_INTERCEPT ? "Ball Intercept" : "RRT Navigation") << std::endl;
    }
    keyY_pressed_last_frame = keyY_pressed_this_frame;
  }
  
  // Only process manual input if no movement system is active
  bool any_movement_active = false;
  if (intelligent_movement && intelligent_movement->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  if (intelligent_movement2 && intelligent_movement2->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  if (multi_player_movement && multi_player_movement->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  if (multi_player_rrt && multi_player_rrt->IsAutoModeEnabled()) {
    any_movement_active = true;
  }
  if (team_competition && team_competition->IsAutoModeEnabled()) {
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