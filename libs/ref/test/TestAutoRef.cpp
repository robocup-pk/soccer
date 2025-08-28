#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include "Game.h"
#include "AutoRef.h"
#include "Controller.h"
#include "SoccerObject.h"
#include "SystemConfig.h"

class AutoRefTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create test soccer objects (robots + ball)
    soccer_objects = CreateTestSoccerObjects();
    game = std::make_unique<ref::Game>(soccer_objects);
    auto_ref = std::make_unique<ref::AutoRef>();
    controller = std::make_unique<ref::Controller>();
    num_robots = 12;
  }

  void TearDown() override {
    soccer_objects.clear();
    game.reset();
    auto_ref.reset();
    controller.reset();
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
  std::unique_ptr<ref::AutoRef> auto_ref;
  std::unique_ptr<ref::Controller> controller;
  int num_robots;
};

// Test AttackerDoubleTouchedBall for team 1 double touch foul
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallTeamOne) {
  // Set up conditions for double touch detection
  game->state = ref::Game::Run;
  game->prev_state = ref::Game::Kickoff;
  game->team_with_ball = 1;
  game->kicker_released = true;
  game->nonkicking_robot_intercepted = false;

  // Attach ball to team 1 kicker
  soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached = true;

  // Execute AttackerDoubleTouchedBall
  int result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);

  // Verify team 1 foul is detected
  EXPECT_EQ(result, 1);
}

// Test AttackerDoubleTouchedBall for team 2 double touch foul
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallTeamTwo) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }

  // Set up conditions for double touch detection
  game->state = ref::Game::Run;
  game->prev_state = ref::Game::FreeKick;
  game->team_with_ball = 2;
  game->kicker_released = true;
  game->nonkicking_robot_intercepted = false;

  // Attach ball to team 2 kicker
  soccer_objects[cfg::SystemConfig::team_two_kicker].is_attached = true;

  // Execute AttackerDoubleTouchedBall
  int result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);

  // Verify team 2 foul is detected
  EXPECT_EQ(result, 2);
}

// Test AttackerDoubleTouchedBall with conditions not met for foul
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallNoFoul) {
  // Test wrong game state
  game->state = ref::Game::Stop;
  game->prev_state = ref::Game::Kickoff;
  game->team_with_ball = 1;
  game->kicker_released = true;
  game->nonkicking_robot_intercepted = false;
  soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached = true;

  int result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test wrong prev_state
  game->state = ref::Game::Run;
  game->prev_state = ref::Game::Stop;

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test kicker not released
  game->prev_state = ref::Game::Kickoff;
  game->kicker_released = false;

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test nonkicking robot intercepted
  game->kicker_released = true;
  game->nonkicking_robot_intercepted = true;

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test kicker not attached to ball
  game->nonkicking_robot_intercepted = false;
  soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached = false;

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);
}

// Test AttackerDoubleTouchedBall with valid states but different scenarios
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallValidStates) {
  // Test with FreeKick prev_state (should work)
  game->state = ref::Game::Run;
  game->prev_state = ref::Game::FreeKick;
  game->team_with_ball = 1;
  game->kicker_released = true;
  game->nonkicking_robot_intercepted = false;
  soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached = true;

  int result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 1);

  // Test with Kickoff prev_state (should work)
  game->prev_state = ref::Game::Kickoff;

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 1);

  // Test edge case: valid conditions but no team_with_ball match
  game->team_with_ball = 3;  // Invalid team

  result = auto_ref->AttackerDoubleTouchedBall(soccer_objects, *game);
  EXPECT_EQ(result, 0);
}

// Test CheckGoal for team 1 scoring (right goal)
TEST_F(AutoRefTest, TestCheckGoalTeamOneScores) {
  // Calculate right goal boundaries
  double right_goal_x1 = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f -
                          vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                         1000.0f;
  double goal_y_center = 0.0;

  // Position ball clearly inside right goal
  Eigen::Vector3d goal_position(right_goal_x1 + 0.1, goal_y_center, 0.0);
  soccer_objects.back().position = goal_position;

  // Execute CheckGoal
  int result = auto_ref->CheckGoal(soccer_objects, *game);

  // Verify team 1 goal is detected
  EXPECT_EQ(result, 1);
}

// Test CheckGoal for team 2 scoring (left goal)
TEST_F(AutoRefTest, TestCheckGoalTeamTwoScores) {
  // Calculate left goal boundaries
  double left_goal_x1 = (-vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f +
                         vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                        1000.0f;
  double goal_y_center = 0.0;

  // Position ball clearly inside left goal
  Eigen::Vector3d goal_position(left_goal_x1 - 0.1, goal_y_center, 0.0);
  soccer_objects.back().position = goal_position;

  // Execute CheckGoal
  int result = auto_ref->CheckGoal(soccer_objects, *game);

  // Verify team 2 goal is detected
  EXPECT_EQ(result, 2);
}

// Test CheckGoal with ball at goal boundaries (edge cases)
TEST_F(AutoRefTest, TestCheckGoalBoundaries) {
  // Get goal dimensions
  double right_goal_x1 = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f -
                          vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                         1000.0f;
  double goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
  double goal_y2 = -(vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;

  // Test ball just at the goal line threshold (accounting for ball radius)
  Eigen::Vector3d threshold_position(right_goal_x1 + cfg::SystemConfig::ball_radius_m + 0.001, 0.0,
                                     0.0);
  soccer_objects.back().position = threshold_position;

  int result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 1);  // Should be a goal

  // Test ball just outside goal height (above)
  Eigen::Vector3d above_goal(right_goal_x1 + 0.1,
                             goal_y1 + cfg::SystemConfig::ball_radius_m + 0.001, 0.0);
  soccer_objects.back().position = above_goal;

  result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);  // Should not be a goal

  // Test ball just outside goal height (below)
  Eigen::Vector3d below_goal(right_goal_x1 + 0.1,
                             goal_y2 - cfg::SystemConfig::ball_radius_m - 0.001, 0.0);
  soccer_objects.back().position = below_goal;

  result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);  // Should not be a goal
}

// Test CheckGoal with ball in various non-goal positions
TEST_F(AutoRefTest, TestCheckGoalNoGoal) {
  // Test ball in center field
  soccer_objects.back().position = Eigen::Vector3d(0.0, 0.0, 0.0);
  int result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test ball near but not in right goal
  double right_goal_x1 = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f -
                          vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                         1000.0f;
  soccer_objects.back().position = Eigen::Vector3d(right_goal_x1 - 0.1, 0.0, 0.0);
  result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test ball near but not in left goal
  double left_goal_x1 = (-vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f +
                         vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                        1000.0f;
  soccer_objects.back().position = Eigen::Vector3d(left_goal_x1 + 0.1, 0.0, 0.0);
  result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);

  // Test ball at extreme field positions (corners)
  double field_width = vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0f;
  double field_height = vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0f;

  soccer_objects.back().position = Eigen::Vector3d(field_width, field_height, 0.0);
  result = auto_ref->CheckGoal(soccer_objects, *game);
  EXPECT_EQ(result, 0);
}

// Test BallLeftFieldTouchLine with ball crossing boundaries and foul counting
TEST_F(AutoRefTest, TestBallLeftFieldTouchLineFouls) {
  // Get field dimensions
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;

  // Reset foul counters
  auto_ref->team_one_foul_counter = 0;
  auto_ref->team_two_foul_counter = 0;

  // Test Team 1 foul - ball crosses top boundary
  game->last_bot_touched_ball.team_id = 1;
  soccer_objects.back().is_attached = false;
  soccer_objects.back().position =
      Eigen::Vector3d(0.0, half_height + 0.1, 0.0);  // Outside top boundary

  bool result = auto_ref->BallLeftFieldTouchLine(soccer_objects, *game);

  EXPECT_TRUE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 1);
  EXPECT_EQ(auto_ref->team_two_foul_counter, 0);

  // Test Team 2 foul - ball crosses bottom boundary
  game->last_bot_touched_ball.team_id = 2;
  soccer_objects.back().position =
      Eigen::Vector3d(0.0, -half_height - 0.1, 0.0);  // Outside bottom boundary

  result = auto_ref->BallLeftFieldTouchLine(soccer_objects, *game);

  EXPECT_TRUE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 1);
  EXPECT_EQ(auto_ref->team_two_foul_counter, 1);
}

// Test BallLeftFieldTouchLine with conditions that prevent foul detection
TEST_F(AutoRefTest, TestBallLeftFieldTouchLineNoFoul) {
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;

  // Reset foul counters
  auto_ref->team_one_foul_counter = 0;
  auto_ref->team_two_foul_counter = 0;

  // Test ball inside field - no foul
  game->last_bot_touched_ball.team_id = 1;
  soccer_objects.back().is_attached = false;
  soccer_objects.back().position = Eigen::Vector3d(0.0, 0.0, 0.0);  // Center of field

  bool result = auto_ref->BallLeftFieldTouchLine(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);

  // Test ball outside field but attached - no foul
  soccer_objects.back().is_attached = true;
  soccer_objects.back().position =
      Eigen::Vector3d(0.0, half_height + 0.1, 0.0);  // Outside boundary but attached

  result = auto_ref->BallLeftFieldTouchLine(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);

  // Test ball at exact boundary (considering ball size) - should be inside
  soccer_objects.back().is_attached = false;
  float ball_radius = soccer_objects.back().size[1] / 2.0;
  soccer_objects.back().position =
      Eigen::Vector3d(0.0, half_height - ball_radius, 0.0);  // Ball edge at boundary

  result = auto_ref->BallLeftFieldTouchLine(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);
}

// Test BallLeftFieldGoalLines with goals and aimless kicks
TEST_F(AutoRefTest, TestBallLeftFieldGoalLinesGoalsAndAimlessKicks) {
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;

  // Test Team 1 scoring with valid kick (should be goal)
  double right_goal_x = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f -
                         vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                            1000.0f +
                        0.1;
  soccer_objects.back().position = Eigen::Vector3d(right_goal_x, 0.0, 0.0);
  vis::last_ball_kick_pos = Eigen::Vector3d(0.1, 0.0, 0.0);  // Kicked from right side (valid)

  std::vector<bool> result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  // Should not be aimless kick - valid goal
  EXPECT_FALSE(result[0]);  // Not aimless kick no goal
  EXPECT_FALSE(result[1]);  // Not aimless kick
  EXPECT_TRUE(result[4]);   // Crossed right

  // Test Team 1 scoring with aimless kick (ball kicked from wrong side)
  vis::last_ball_kick_pos = Eigen::Vector3d(-0.1, 0.0, 0.0);  // Kicked from left side (aimless)

  result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  EXPECT_TRUE(result[0]);   // Aimless kick no goal
  EXPECT_FALSE(result[1]);  // Not regular aimless kick
  EXPECT_TRUE(result[4]);   // Still crossed right

  // Test Team 2 scoring with valid kick
  double left_goal_x = (-vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f +
                        vis::SoccerField::GetInstance().goal_width_mm / 2.0f) /
                           1000.0f -
                       0.1;
  soccer_objects.back().position = Eigen::Vector3d(left_goal_x, 0.0, 0.0);
  vis::last_ball_kick_pos = Eigen::Vector3d(-0.1, 0.0, 0.0);  // Kicked from left side (valid)

  result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  EXPECT_FALSE(result[0]);  // Not aimless kick no goal
  EXPECT_FALSE(result[1]);  // Not aimless kick
  EXPECT_TRUE(result[3]);   // Crossed left
}

// Test BallLeftFieldGoalLines with boundary crossings and aimless kicks (no goals)
TEST_F(AutoRefTest, TestBallLeftFieldGoalLinesBoundaryAndAimless) {
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;

  // Reset foul counters
  auto_ref->team_one_foul_counter = 0;
  auto_ref->team_two_foul_counter = 0;

  // Test aimless kick crossing left boundary (no goal)
  game->last_bot_touched_ball.team_id = 1;
  soccer_objects.back().position =
      Eigen::Vector3d(-half_width - 0.1, 0.5, 0.0);          // Outside left, not in goal
  vis::last_ball_kick_pos = Eigen::Vector3d(0.1, 0.0, 0.0);  // Kicked from right side

  std::vector<bool> result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  EXPECT_FALSE(result[0]);  // Not aimless kick no goal (no goal scored)
  EXPECT_TRUE(result[1]);   // Aimless kick detected
  EXPECT_FALSE(result[2]);  // Not boundary crossing (it's aimless)
  EXPECT_TRUE(result[3]);   // Crossed left
  EXPECT_FALSE(result[4]);  // Did not cross right

  // Test regular boundary crossing (not aimless) - Team 2
  game->last_bot_touched_ball.team_id = 2;
  soccer_objects.back().position =
      Eigen::Vector3d(half_width + 0.1, 0.5, 0.0);  // Outside right, not in goal
  vis::last_ball_kick_pos =
      Eigen::Vector3d(0.1, 0.0, 0.0);  // Kicked from right side (not aimless)

  result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  EXPECT_FALSE(result[0]);                        // Not aimless kick no goal
  EXPECT_FALSE(result[1]);                        // Not aimless kick
  EXPECT_TRUE(result[2]);                         // Boundary crossing foul
  EXPECT_FALSE(result[3]);                        // Did not cross left
  EXPECT_TRUE(result[4]);                         // Crossed right
  EXPECT_EQ(auto_ref->team_two_foul_counter, 1);  // Foul counter incremented

  // Test ball inside field - no violations
  soccer_objects.back().position = Eigen::Vector3d(0.0, 0.0, 0.0);  // Center field

  result = auto_ref->BallLeftFieldGoalLines(soccer_objects, *game);

  EXPECT_FALSE(result[0]);  // No violations
  EXPECT_FALSE(result[1]);
  EXPECT_FALSE(result[2]);
  EXPECT_FALSE(result[3]);  // Did not cross boundaries
  EXPECT_FALSE(result[4]);
}

// Test BotDribbledBallTooFar with legal and illegal dribbling distances
TEST_F(AutoRefTest, TestBotDribbledBallTooFarDistanceLogic) {
  // Reset foul counters and state
  auto_ref->team_one_foul_counter = 0;
  auto_ref->team_two_foul_counter = 0;
  game->bot_got_foul_for_overdribbling = false;

  // Set initial attachment position
  Eigen::Vector3d start_pos(0.0, 0.0, 0.0);
  game->last_attached_pos = start_pos;
  soccer_objects.back().is_attached = true;
  game->last_bot_touched_ball.team_id = 1;

  // Test legal dribbling distance (< 1m) - should update last_valid_pos
  Eigen::Vector3d legal_pos(0.8, 0.0, 0.0);  // 0.8m from start
  soccer_objects.back().position = legal_pos;

  bool result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);
  EXPECT_FALSE(game->bot_got_foul_for_overdribbling);
  EXPECT_NEAR(game->displacement, 0.8, 1e-6);
  EXPECT_EQ(game->last_valid_pos, legal_pos);  // Should be updated

  // Test illegal dribbling distance (> 1m) with Team 1
  Eigen::Vector3d illegal_pos(1.2, 0.0, 0.0);  // 1.2m from start
  soccer_objects.back().position = illegal_pos;

  result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_TRUE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 1);
  EXPECT_TRUE(game->bot_got_foul_for_overdribbling);
  EXPECT_NEAR(game->displacement, 1.2, 1e-6);
  EXPECT_EQ(game->last_valid_pos, legal_pos);  // Should remain at last legal position

  // Test illegal dribbling distance with Team 2 (reset state first)
  game->bot_got_foul_for_overdribbling = false;
  game->last_attached_pos = Eigen::Vector3d(2.0, 1.0, 0.0);
  game->last_bot_touched_ball.team_id = 2;
  soccer_objects.back().position = Eigen::Vector3d(2.0, 2.5, 0.0);  // 1.5m displacement

  result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_TRUE(result);
  EXPECT_EQ(auto_ref->team_two_foul_counter, 1);
  EXPECT_TRUE(game->bot_got_foul_for_overdribbling);
  EXPECT_NEAR(game->displacement, 1.5, 1e-6);
}

// Test BotDribbledBallTooFar with conditions that prevent foul detection
TEST_F(AutoRefTest, TestBotDribbledBallTooFarNoFoul) {
  // Reset state
  auto_ref->team_one_foul_counter = 0;
  game->bot_got_foul_for_overdribbling = false;
  game->last_bot_touched_ball.team_id = 1;

  // Test: already got foul for overdribbling - should return false immediately
  game->bot_got_foul_for_overdribbling = true;
  game->last_attached_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  soccer_objects.back().position = Eigen::Vector3d(2.0, 0.0, 0.0);  // 2m displacement
  soccer_objects.back().is_attached = true;

  bool result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);

  // Reset and test: ball not attached - should not trigger foul
  game->bot_got_foul_for_overdribbling = false;
  soccer_objects.back().is_attached = false;

  result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);
  EXPECT_FALSE(game->bot_got_foul_for_overdribbling);
  EXPECT_NEAR(game->displacement, 2.0, 1e-6);  // Displacement still calculated

  // Test: exactly 1m displacement - should not trigger foul (condition is > 1)
  soccer_objects.back().is_attached = true;
  game->last_attached_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  soccer_objects.back().position = Eigen::Vector3d(1.0, 0.0, 0.0);  // Exactly 1m

  result = auto_ref->BotDribbledBallTooFar(soccer_objects, *game);

  EXPECT_FALSE(result);
  EXPECT_EQ(auto_ref->team_one_foul_counter, 0);
  EXPECT_FALSE(game->bot_got_foul_for_overdribbling);
  EXPECT_NEAR(game->displacement, 1.0, 1e-6);
  EXPECT_EQ(game->last_valid_pos, soccer_objects.back().position);  // Should be updated since <= 1
}

// Test AttackerDoubleTouchedBallInOpponentDefenseArea for team 1 defender in team 2 penalty area
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallInOpponentDefenseAreaTeamOne) {
  // Position team 1 robot in team 2's penalty area
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_penalty_width =
      (vis::SoccerField::GetInstance().penalty_area_width_mm / 2.0f) / 1000.0f;

  // Place robot in team 2's penalty area (left side)
  soccer_objects[0].position = Eigen::Vector3d(half_width - half_penalty_width + 0.1, 0.0, 0.0);
  soccer_objects[0].is_attached = true;
  game->was_ball_prev_attached = false;

  int initial_foul_count = auto_ref->team_one_foul_counter;

  // Execute the function
  auto_ref->AttackerDoubleTouchedBallInOpponentDefenseArea(soccer_objects, *game);

  // Verify foul was detected
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count + 1);
}

// Test AttackerDoubleTouchedBallInOpponentDefenseArea for team 2 defender in team 1 penalty area
TEST_F(AutoRefTest, TestAttackerDoubleTouchedBallInOpponentDefenseAreaTeamTwo) {
  if (cfg::SystemConfig::num_robots < 12) {
    GTEST_SKIP() << "Skipping: requires >= 12 robots";
  }

  // Position team 2 robot in team 1's penalty area
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_penalty_width =
      (vis::SoccerField::GetInstance().penalty_area_width_mm / 2.0f) / 1000.0f;

  // Place robot in team 1's penalty area (right side)
  soccer_objects[6].position = Eigen::Vector3d(-half_width + half_penalty_width - 0.1, 0.0, 0.0);
  soccer_objects[6].is_attached = true;
  game->was_ball_prev_attached = false;

  int initial_foul_count = auto_ref->team_two_foul_counter;

  // Execute the function
  auto_ref->AttackerDoubleTouchedBallInOpponentDefenseArea(soccer_objects, *game);

  // Verify foul was detected
  EXPECT_EQ(auto_ref->team_two_foul_counter, initial_foul_count + 1);
}

// Test BotKickedBallTooFast with ball velocity exceeding limit
TEST_F(AutoRefTest, TestBotKickedBallTooFastExceedsLimit) {
  // Set ball velocity to exceed 6.5 m/s
  soccer_objects.back().velocity = Eigen::Vector3d(7.0, 0.0, 0.0);
  game->last_bot_touched_ball.team_id = 1;

  int initial_foul_count = auto_ref->team_one_foul_counter;

  // Execute the function
  auto_ref->BotKickedBallTooFast(soccer_objects, *game);

  // Verify foul was detected
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count + 1);
}

// Test BotKickedBallTooFast with ball velocity within limit
TEST_F(AutoRefTest, TestBotKickedBallTooFastWithinLimit) {
  // Set ball velocity to be within 6.5 m/s limit
  soccer_objects.back().velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
  game->last_bot_touched_ball.team_id = 2;

  int initial_foul_count = auto_ref->team_two_foul_counter;

  // Execute the function
  auto_ref->BotKickedBallTooFast(soccer_objects, *game);

  // Verify no foul was detected
  EXPECT_EQ(auto_ref->team_two_foul_counter, initial_foul_count);
}

// Test BotCrashUnique with robots colliding and speed difference > 1.5 m/s
TEST_F(AutoRefTest, TestBotCrashUniqueSpeedViolation) {
  // Position two robots close to each other (simulate collision)
  // Robot radius is ~0.102m, so distance between centers should be < 0.204m for collision
  soccer_objects[0].position = Eigen::Vector3d(0.0, 0.0, 0.0);
  soccer_objects[6].position = Eigen::Vector3d(0.15, 0.0, 0.0);  // Close enough for collision

  // Set velocities with large difference
  soccer_objects[0].velocity = Eigen::Vector3d(2.0, 0.0, 0.0);  // Fast robot
  soccer_objects[6].velocity = Eigen::Vector3d(0.1, 0.0, 0.0);  // Slow robot

  int initial_foul_count_team1 = auto_ref->team_one_foul_counter;
  int initial_foul_count_team2 = auto_ref->team_two_foul_counter;

  // Execute the function
  auto_ref->BotCrashUnique(soccer_objects, *game);

  // Verify foul was detected (faster robot should get foul)
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count_team1 + 1);
  EXPECT_EQ(auto_ref->team_two_foul_counter, initial_foul_count_team2);
}

// Test BotCrashUnique with no collision
TEST_F(AutoRefTest, TestBotCrashUniqueNoCollision) {
  // Position robots far apart (no collision)
  soccer_objects[0].position = Eigen::Vector3d(0.0, 0.0, 0.0);
  soccer_objects[6].position = Eigen::Vector3d(2.0, 0.0, 0.0);  // Far apart

  int initial_foul_count_team1 = auto_ref->team_one_foul_counter;
  int initial_foul_count_team2 = auto_ref->team_two_foul_counter;

  // Execute the function
  auto_ref->BotCrashUnique(soccer_objects, *game);

  // Verify no fouls were detected
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count_team1);
  EXPECT_EQ(auto_ref->team_two_foul_counter, initial_foul_count_team2);
}

// Test BotTooFastInStop with robot moving too fast during stop
TEST_F(AutoRefTest, TestBotTooFastInStopViolation) {
  // Set game state to Stop
  game->state = ref::Game::Stop;

  // Set robot velocity to exceed 1.2 m/s
  soccer_objects[0].velocity = Eigen::Vector3d(1.5, 0.0, 0.0);
  soccer_objects[0].was_given_speeding_foul_in_stop = false;

  int initial_foul_count = auto_ref->team_one_foul_counter;

  // Execute the function
  auto_ref->BotTooFastInStop(soccer_objects, *game);

  // Verify foul was detected
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count + 1);
  EXPECT_TRUE(soccer_objects[0].was_given_speeding_foul_in_stop);
}

// Test BotTooFastInStop with robot moving within speed limit
TEST_F(AutoRefTest, TestBotTooFastInStopWithinLimit) {
  // Set game state to Stop
  game->state = ref::Game::Stop;

  // Set robot velocity to be within 1.2 m/s limit
  soccer_objects[0].velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  soccer_objects[0].was_given_speeding_foul_in_stop = false;

  int initial_foul_count = auto_ref->team_one_foul_counter;

  // Execute the function
  auto_ref->BotTooFastInStop(soccer_objects, *game);

  // Verify no foul was detected
  EXPECT_EQ(auto_ref->team_one_foul_counter, initial_foul_count);
  EXPECT_FALSE(soccer_objects[0].was_given_speeding_foul_in_stop);
}
