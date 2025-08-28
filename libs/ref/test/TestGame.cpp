#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include "Game.h"
#include "AutoRef.h"
#include "Controller.h"
#include "SoccerObject.h"
#include "SystemConfig.h"

class GameTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create test soccer objects (robots + ball)
    soccer_objects = CreateTestSoccerObjects();
    game = std::make_unique<ref::Game>(soccer_objects);
    num_robots = 12;
    }

  void TearDown() override {
    soccer_objects.clear();
    game.reset();
  }

  std::vector<state::SoccerObject> CreateTestSoccerObjects() {
    std::vector<state::SoccerObject> soccer_objects;

    for (int i = 0; i < num_robots / 2; ++i) {
      // Robots (team one)
      std::string name = "robot" + std::to_string(i);
      Eigen::Vector3d robot_position_m =
          i < cfg::RightRobotHomeCoordinates.size()
              ? cfg::RightRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(i))
              : Eigen::Vector3d(0.3, 0, 0);
      soccer_objects.push_back(state::SoccerObject(
          name, robot_position_m, cfg::SystemConfig::robot_size_m, 1,
          cfg::SystemConfig::init_robot_velocity_mps,
          cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
    }

    // Robots (team two)
    for (int i = num_robots / 2; i < num_robots; ++i) {
      std::string name = "robot" + std::to_string(i);
      int index = i - num_robots / 2;
      Eigen::Vector3d robot_position_m =
          index < cfg::LeftRobotHomeCoordinates.size()
              ? cfg::LeftRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(index))
              : Eigen::Vector3d(-0.3, 0, 0);
      soccer_objects.push_back(state::SoccerObject(
          name, robot_position_m, cfg::SystemConfig::robot_size_m, 2,
          cfg::SystemConfig::init_robot_velocity_mps,
          cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
    }

    // ball
    soccer_objects.push_back(state::SoccerObject(
        "ball", cfg::SystemConfig::init_ball_position,
        Eigen::Vector2d(cfg::SystemConfig::ball_radius_m * 2,
                        cfg::SystemConfig::ball_radius_m * 2),
        0, cfg::SystemConfig::init_ball_velocity_mps,
        cfg::SystemConfig::init_ball_acceleration_mpsps, cfg::SystemConfig::ball_mass_kg));

    return soccer_objects;
  }

  std::vector<state::SoccerObject> soccer_objects;
  std::unique_ptr<ref::Game> game;
  int num_robots;
};

TEST_F(GameTest, TestGameConstructor) {
  // Test initial state
  EXPECT_EQ(game->state, ref::Game::Kickoff);
  EXPECT_EQ(game->state_start_ball_pos, cfg::SystemConfig::init_ball_position);
  EXPECT_EQ(game->last_bot_touched_ball.name,
            soccer_objects[cfg::SystemConfig::team_one_kicker].name);
  EXPECT_EQ(game->last_valid_pos, cfg::SystemConfig::init_ball_position);
}

// Test SetUpFreeKick velocity reset functionality
TEST_F(GameTest, TestSetUpFreeKickVelocityReset) {
  // Set up initial conditions
  game->last_bot_touched_ball = soccer_objects[5];  // Team 1 robot
  game->last_bot_touched_ball.team_id = 1;

  // Set various velocities for all objects
  for (int i = 0; i < soccer_objects.size(); i++) {
    soccer_objects[i].velocity = Eigen::Vector3d(i * 0.5, -i * 0.3, i * 0.1);
  }

  Eigen::Vector3d free_kick_pos(0.0, 0.0, 0.0);

  // Execute SetUpFreeKick
  game->SetUpFreeKick(soccer_objects, free_kick_pos);

  // Verify ALL velocities are reset to zero
  for (const auto& obj : soccer_objects) {
    EXPECT_EQ(obj.velocity, Eigen::Vector3d(0, 0, 0));
  }
}

// Test DoGoals multiple times to verify score accumulation
TEST_F(GameTest, TestDoGoalsMultipleScores) {
  // Set initial scores
  game->team_one_score = 2;
  game->team_two_score = 1;

  // Team 1 scores
  game->DoGoals(soccer_objects, 1);
  EXPECT_EQ(game->team_one_score, 3);
  EXPECT_EQ(game->team_two_score, 1);
  EXPECT_EQ(game->team_with_ball, 2);

  // Team 2 scores
  game->DoGoals(soccer_objects, 2);
  EXPECT_EQ(game->team_one_score, 3);
  EXPECT_EQ(game->team_two_score, 2);
  EXPECT_EQ(game->team_with_ball, 1);

  // Team 2 scores again
  game->DoGoals(soccer_objects, 2);
  EXPECT_EQ(game->team_one_score, 3);
  EXPECT_EQ(game->team_two_score, 3);
  EXPECT_EQ(game->team_with_ball, 1);
}

// Test UpdateGameState transition from Kickoff/FreeKick to Run
TEST_F(GameTest, TestUpdateGameStateTransitionToRun) {
  // Test Kickoff to Run transition
  game->state = ref::Game::Kickoff;
  game->prev_state = ref::Game::Stop;

  Eigen::Vector3d initial_pos(0.0, 0.0, 0.0);
  game->state_start_ball_pos = initial_pos;
  soccer_objects.back().position = initial_pos;
  vis::button_pressed = 0;

  // Move ball beyond threshold (0.2m)
  soccer_objects.back().position = Eigen::Vector3d(0.25, 0.0, 0.0);
  game->UpdateGameState(soccer_objects);

  EXPECT_EQ(game->state, ref::Game::Run);
  EXPECT_EQ(game->prev_state, ref::Game::Kickoff);

  // Test FreeKick to Run transition
  game->state = ref::Game::FreeKick;
  game->prev_state = ref::Game::Run;
  game->state_start_ball_pos = Eigen::Vector3d(0.5, 0.3, 0.0);
  soccer_objects.back().position = Eigen::Vector3d(0.5, 0.55, 0.0);  // 0.25m displacement

  game->UpdateGameState(soccer_objects);

  EXPECT_EQ(game->state, ref::Game::Run);
  EXPECT_EQ(game->prev_state, ref::Game::FreeKick);
}

// Test UpdateGameState when conditions are not met for transition
TEST_F(GameTest, TestUpdateGameStateNoTransition) {
  // Test ball movement below threshold
  game->state = ref::Game::Kickoff;
  game->prev_state = ref::Game::Stop;

  Eigen::Vector3d initial_pos(0.0, 0.0, 0.0);
  game->state_start_ball_pos = initial_pos;
  soccer_objects.back().position = Eigen::Vector3d(0.15, 0.1, 0.0);  // ~0.18m < 0.2m
  vis::button_pressed = 0;

  game->UpdateGameState(soccer_objects);

  EXPECT_EQ(game->state, ref::Game::Kickoff);
  EXPECT_EQ(game->prev_state, ref::Game::Stop);

  // Test non-transitional state (Run) - should remain unchanged
  game->state = ref::Game::Run;
  soccer_objects.back().position = Eigen::Vector3d(1.0, 1.0, 0.0);  // Large movement

  game->UpdateGameState(soccer_objects);
}