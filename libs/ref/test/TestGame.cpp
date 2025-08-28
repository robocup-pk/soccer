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

// Test SetUpKickOff with team 1 having the ball
TEST_F(GameTest, TestSetUpKickOffTeamOne) {
  // Set team with ball to 1
  game->team_with_ball = 1;

  // Store original positions for comparison
  std::vector<Eigen::Vector3d> original_positions;
  for (const auto& obj : soccer_objects) {
    original_positions.push_back(obj.position);
  }

  // Execute SetUpKickOff
  game->SetUpKickOff(soccer_objects);

  // Verify team 1 kicker position (should be positioned for kickoff)
  double expected_kicker_x =
      -1 * cfg::SystemConfig::ball_radius_m - cfg::SystemConfig::robot_size_m[0] / 2.0;
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[0], expected_kicker_x,
              1e-6);
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[1], 0.0, 1e-6);
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[2], 0.0, 1e-6);

  // Verify team 2 robots are positioned at their kickoff positions
  // The first team 2 robot (index 6) should be at the designated kickoff position
  EXPECT_NEAR(soccer_objects[6].position[0], 0.5, 1e-6);
  EXPECT_NEAR(soccer_objects[6].position[1], 0.0, 1e-6);
  EXPECT_NEAR(soccer_objects[6].position[2], M_PI, 1e-6);

  // Verify ball position
  EXPECT_EQ(soccer_objects.back().position, cfg::SystemConfig::init_ball_position);

  // Verify ball is attached to team 1 kicker
  EXPECT_EQ(soccer_objects.back().attached_to,
            &soccer_objects[cfg::SystemConfig::team_one_kicker]);

  // Verify all velocities are zero
  for (const auto& obj : soccer_objects) {
    EXPECT_EQ(obj.velocity, Eigen::Vector3d(0, 0, 0));
  }

  // Verify game state variables
  EXPECT_EQ(game->state, ref::Game::Kickoff);
  EXPECT_EQ(game->prev_state, ref::Game::Kickoff);
  EXPECT_FALSE(game->kicker_released);
  EXPECT_FALSE(game->nonkicking_robot_intercepted);
  EXPECT_EQ(game->state_start_ball_pos, soccer_objects.back().position);
  EXPECT_EQ(game->last_valid_pos, soccer_objects.back().position);
  EXPECT_EQ(game->last_attached_pos, cfg::SystemConfig::init_ball_position);
}

// Test SetUpKickOff with team 2 having the ball
TEST_F(GameTest, TestSetUpKickOffTeamTwo) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }
  // Set team with ball to 2
  game->team_with_ball = 2;

  // Execute SetUpKickOff
  game->SetUpKickOff(soccer_objects);

  // Verify team 2 kicker position (should be positioned for kickoff)
  double expected_kicker_x =
      cfg::SystemConfig::ball_radius_m + cfg::SystemConfig::robot_size_m[0] / 2.0;
  int team2_kicker = 6;  // First team 2 robot
  EXPECT_NEAR(soccer_objects[team2_kicker].position[0], expected_kicker_x, 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[1], 0.0, 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[2], M_PI, 1e-6);

  // Verify team 1 robots are positioned at their kickoff positions
  // The first team 1 robot should be at the designated kickoff position
  EXPECT_NEAR(soccer_objects[0].position[0], -0.5, 1e-6);
  EXPECT_NEAR(soccer_objects[0].position[1], 0.0, 1e-6);
  EXPECT_NEAR(soccer_objects[0].position[2], 0.0, 1e-6);

  // Verify ball position (negated for team 2)
  EXPECT_EQ(soccer_objects.back().position, -1 * cfg::SystemConfig::init_ball_position);

  // Verify ball is attached to team 2 kicker
  EXPECT_EQ(soccer_objects.back().attached_to, &soccer_objects[team2_kicker]);
}

// Test SetUpFreeKick with team 1 having the ball (last touch by team 2)
TEST_F(GameTest, TestSetUpFreeKickTeamOne) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }
  // Set up initial conditions - team 2 robot touched ball last
  game->last_bot_touched_ball = soccer_objects[6];  // Team 2 robot
  game->last_bot_touched_ball.team_id = 2;

  // Set a test position for the free kick
  Eigen::Vector3d free_kick_pos(0.2, 0.1, 0.0);

  // Give some objects initial velocities to test they get reset
  soccer_objects[0].velocity = Eigen::Vector3d(1.0, 2.0, 0.5);
  soccer_objects[6].velocity = Eigen::Vector3d(-1.0, -2.0, -0.5);
  soccer_objects.back().velocity = Eigen::Vector3d(0.5, 0.5, 0.5);

  // Execute SetUpFreeKick
  game->SetUpFreeKick(soccer_objects, free_kick_pos);

  // Verify team with ball is set to 1 (opposite of last touch)
  EXPECT_EQ(game->team_with_ball, 1);

  // Verify ball position (may be adjusted by HandleBallSticking)
  // The ball position will be adjusted to stick to the kicker robot
  EXPECT_NEAR(soccer_objects.back().position[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(soccer_objects.back().position[2], free_kick_pos[2], 1e-6);

  // Verify team 1 kicker position (positioned behind the ball)
  double expected_kicker_x = free_kick_pos[0] - cfg::SystemConfig::ball_radius_m -
                             cfg::SystemConfig::robot_size_m[0] / 2.0;
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[0], expected_kicker_x,
              1e-6);
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[1], free_kick_pos[1],
              1e-6);
  EXPECT_NEAR(soccer_objects[cfg::SystemConfig::team_one_kicker].position[2], 0.0, 1e-6);

  // Verify other team 1 robots are in formation positions (excluding kicker)
  for (int i = 0; i < soccer_objects.size() / 2; i++) {
    if (i != cfg::SystemConfig::team_one_kicker) {
      EXPECT_EQ(soccer_objects[i].position, cfg::SystemConfig::team_one_start_formation[i]);
    }
  }

  // Verify team 2 robots are in their formation positions
  for (int i = soccer_objects.size() / 2; i < soccer_objects.size() - 1; i++) {
    int formation_index = i - soccer_objects.size() / 2;
    EXPECT_EQ(soccer_objects[i].position,
              cfg::SystemConfig::team_two_start_formation[formation_index]);
  }

  // Verify ball is attached to team 1 kicker
  EXPECT_EQ(soccer_objects.back().attached_to,
            &soccer_objects[cfg::SystemConfig::team_one_kicker]);

  // Verify all velocities are zero
  for (const auto& obj : soccer_objects) {
    EXPECT_EQ(obj.velocity, Eigen::Vector3d(0, 0, 0));
  }

  // Verify game state variables (ball position may be adjusted by HandleBallSticking)
  EXPECT_EQ(game->state, ref::Game::FreeKick);
  EXPECT_EQ(game->prev_state, ref::Game::FreeKick);
  EXPECT_FALSE(game->kicker_released);
  EXPECT_FALSE(game->nonkicking_robot_intercepted);
  EXPECT_NEAR(game->state_start_ball_pos[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(game->state_start_ball_pos[2], free_kick_pos[2], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[2], free_kick_pos[2], 1e-6);
  EXPECT_EQ(game->last_attached_pos, free_kick_pos);
  EXPECT_FALSE(game->bot_got_foul_for_overdribbling);
  EXPECT_EQ(game->displacement, 0);

  // Verify robot flags are reset
  for (const auto& obj : soccer_objects) {
    EXPECT_FALSE(obj.is_selected_player);
    EXPECT_FALSE(obj.was_given_speeding_foul_in_stop);
  }
}

// Test SetUpFreeKick with team 2 having the ball (last touch by team 1)
TEST_F(GameTest, TestSetUpFreeKickTeamTwo) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }

  // Set up initial conditions - team 1 robot touched ball last
  game->last_bot_touched_ball = soccer_objects[0];  // Team 1 robot
  game->last_bot_touched_ball.team_id = 1;

  // Set a test position for the free kick
  Eigen::Vector3d free_kick_pos(-0.3, -0.2, 0.0);

  // Give some objects initial velocities to test they get reset
  soccer_objects[1].velocity = Eigen::Vector3d(2.0, 1.0, 1.0);
  soccer_objects[7].velocity = Eigen::Vector3d(-2.0, -1.0, -1.0);
  soccer_objects.back().velocity = Eigen::Vector3d(1.5, -1.5, 0.8);

  // Execute SetUpFreeKick
  game->SetUpFreeKick(soccer_objects, free_kick_pos);

  // Verify team with ball is set to 2 (opposite of last touch)
  EXPECT_EQ(game->team_with_ball, 2);

  // Verify ball position (may be adjusted by HandleBallSticking)
  // The ball position will be adjusted to stick to the kicker robot
  EXPECT_NEAR(soccer_objects.back().position[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(soccer_objects.back().position[2], free_kick_pos[2], 1e-6);

  // Verify team 2 kicker position (positioned in front of the ball)
  double expected_kicker_x = free_kick_pos[0] + cfg::SystemConfig::ball_radius_m +
                             cfg::SystemConfig::robot_size_m[0] / 2.0;
  int team2_kicker = cfg::SystemConfig::team_two_kicker;
  EXPECT_NEAR(soccer_objects[team2_kicker].position[0], expected_kicker_x, 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[2], M_PI, 1e-6);

  // Verify other team 2 robots are in formation positions (excluding kicker)
  for (int i = soccer_objects.size() / 2; i < soccer_objects.size() - 1; i++) {
    int formation_index = i - soccer_objects.size() / 2;
    if (i != team2_kicker) {
      EXPECT_EQ(soccer_objects[i].position,
                cfg::SystemConfig::team_two_start_formation[formation_index]);
    }
  }

  // Verify team 1 robots are in their formation positions
  for (int i = 0; i < soccer_objects.size() / 2; i++) {
    EXPECT_EQ(soccer_objects[i].position, cfg::SystemConfig::team_one_start_formation[i]);
  }

  // Verify ball is attached to team 2 kicker
  EXPECT_EQ(soccer_objects.back().attached_to, &soccer_objects[team2_kicker]);

  // Verify all velocities are zero
  for (const auto& obj : soccer_objects) {
    EXPECT_EQ(obj.velocity, Eigen::Vector3d(0, 0, 0));
  }

  // Verify game state variables (ball position may be adjusted by HandleBallSticking)
  EXPECT_EQ(game->state, ref::Game::FreeKick);
  EXPECT_EQ(game->prev_state, ref::Game::FreeKick);
  EXPECT_FALSE(game->kicker_released);
  EXPECT_FALSE(game->nonkicking_robot_intercepted);
  EXPECT_NEAR(game->state_start_ball_pos[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(game->state_start_ball_pos[2], free_kick_pos[2], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[1], free_kick_pos[1], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[2], free_kick_pos[2], 1e-6);
  EXPECT_EQ(game->last_attached_pos, free_kick_pos);
  EXPECT_FALSE(game->bot_got_foul_for_overdribbling);
  EXPECT_EQ(game->displacement, 0);

  // Verify robot flags are reset
  for (const auto& obj : soccer_objects) {
    EXPECT_FALSE(obj.is_selected_player);
    EXPECT_FALSE(obj.was_given_speeding_foul_in_stop);
  }
}

// Test SetUpFreeKick with edge case: free kick position at field boundary
TEST_F(GameTest, TestSetUpFreeKickAtBoundary) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }
  // Set up initial conditions
  game->last_bot_touched_ball = soccer_objects[2];  // Team 1 robot
  game->last_bot_touched_ball.team_id = 1;

  // Set a boundary position for the free kick
  Eigen::Vector3d boundary_pos(0.9, 0.6, 0.0);  // Near field edge

  // Execute SetUpFreeKick
  game->SetUpFreeKick(soccer_objects, boundary_pos);

  // Verify team with ball is set to 2 (opposite of last touch)
  EXPECT_EQ(game->team_with_ball, 2);

  // Verify ball position is set correctly even at boundary (may be adjusted by HandleBallSticking)
  // The ball position will be adjusted to stick to the kicker robot
  EXPECT_NEAR(soccer_objects.back().position[1], boundary_pos[1], 1e-6);
  EXPECT_NEAR(soccer_objects.back().position[2], boundary_pos[2], 1e-6);

  // Verify team 2 kicker is positioned relative to boundary position
  double expected_kicker_x = boundary_pos[0] + cfg::SystemConfig::ball_radius_m +
                             cfg::SystemConfig::robot_size_m[0] / 2.0;
  int team2_kicker = cfg::SystemConfig::team_two_kicker;
  EXPECT_NEAR(soccer_objects[team2_kicker].position[0], expected_kicker_x, 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[1], boundary_pos[1], 1e-6);
  EXPECT_NEAR(soccer_objects[team2_kicker].position[2], M_PI, 1e-6);

  // Verify all other game state variables are properly set (ball position may be adjusted)
  EXPECT_EQ(game->state, ref::Game::FreeKick);
  EXPECT_NEAR(game->state_start_ball_pos[1], boundary_pos[1], 1e-6);
  EXPECT_NEAR(game->state_start_ball_pos[2], boundary_pos[2], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[1], boundary_pos[1], 1e-6);
  EXPECT_NEAR(game->last_valid_pos[2], boundary_pos[2], 1e-6);
  EXPECT_EQ(game->last_attached_pos, boundary_pos);
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

// Test DoGoals with team 1 scoring
TEST_F(GameTest, TestDoGoalsTeamOne) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }
  // Set up initial conditions
  int initial_team_one_score = game->team_one_score;
  int initial_team_two_score = game->team_two_score;

  // Attach ball to a team 1 robot to test detachment
  state::SoccerObject* kicker_robot = &soccer_objects[cfg::SystemConfig::team_one_kicker];
  soccer_objects.back().is_attached = true;
  soccer_objects.back().attached_to = kicker_robot;
  kicker_robot->is_attached = true;
  kicker_robot->attached_to = &soccer_objects.back();

  // Set initial team with ball
  game->team_with_ball = 1;

  // Execute DoGoals for team 1 scoring
  game->DoGoals(soccer_objects, 1);

  // Verify team 1 score is incremented
  EXPECT_EQ(game->team_one_score, initial_team_one_score + 1);
  EXPECT_EQ(game->team_two_score, initial_team_two_score);

  // Verify team with ball is switched to team 2 (for kickoff)
  EXPECT_EQ(game->team_with_ball, 2);

  // Verify original robot detachment (the ball gets reattached to kickoff robot)
  EXPECT_FALSE(kicker_robot->is_attached);
  EXPECT_EQ(kicker_robot->attached_to, nullptr);

  // Verify kickoff setup - ball should be at center and attached to team 2
  EXPECT_EQ(soccer_objects.back().position, -1 * cfg::SystemConfig::init_ball_position);
  EXPECT_EQ(soccer_objects.back().attached_to,
            &soccer_objects[cfg::SystemConfig::team_two_kicker]);

  // Verify game state is set to Kickoff
  EXPECT_EQ(game->state, ref::Game::Kickoff);
  EXPECT_EQ(game->prev_state, ref::Game::Kickoff);
  EXPECT_FALSE(game->kicker_released);
  EXPECT_FALSE(game->nonkicking_robot_intercepted);
}

// Test DoGoals with team 2 scoring
TEST_F(GameTest, TestDoGoalsTeamTwo) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }

  // Set up initial conditions
  int initial_team_one_score = game->team_one_score;
  int initial_team_two_score = game->team_two_score;

  // Attach ball to a team 2 robot to test detachment
  int team2_kicker = cfg::SystemConfig::team_two_kicker;
  state::SoccerObject* kicker_robot = &soccer_objects[team2_kicker];
  soccer_objects.back().is_attached = true;
  soccer_objects.back().attached_to = kicker_robot;
  kicker_robot->is_attached = true;
  kicker_robot->attached_to = &soccer_objects.back();

  // Set initial team with ball
  game->team_with_ball = 2;

  // Execute DoGoals for team 2 scoring
  game->DoGoals(soccer_objects, 2);

  // Verify team 2 score is incremented
  EXPECT_EQ(game->team_two_score, initial_team_two_score + 1);
  EXPECT_EQ(game->team_one_score, initial_team_one_score);

  // Verify team with ball is switched to team 1 (for kickoff)
  EXPECT_EQ(game->team_with_ball, 1);

  // Verify original robot detachment (the ball gets reattached to kickoff robot)
  EXPECT_FALSE(kicker_robot->is_attached);
  EXPECT_EQ(kicker_robot->attached_to, nullptr);

  // Verify kickoff setup - ball should be at center and attached to team 1
  EXPECT_EQ(soccer_objects.back().position, cfg::SystemConfig::init_ball_position);
  EXPECT_EQ(soccer_objects.back().attached_to,
            &soccer_objects[cfg::SystemConfig::team_one_kicker]);

  // Verify game state is set to Kickoff
  EXPECT_EQ(game->state, ref::Game::Kickoff);
  EXPECT_EQ(game->prev_state, ref::Game::Kickoff);
  EXPECT_FALSE(game->kicker_released);
  EXPECT_FALSE(game->nonkicking_robot_intercepted);
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