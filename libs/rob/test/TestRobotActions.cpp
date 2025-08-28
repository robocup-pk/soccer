#include <gtest/gtest.h>
#include "RobotManager.h"

class RobotManagerActionsTest : public ::testing::Test {
 protected:
  void SetUp() override { robot_manager = std::make_unique<rob::RobotManager>(); }

  std::unique_ptr<rob::RobotManager> robot_manager;
};

TEST_F(RobotManagerActionsTest, TestKickBallSetsAction) {
  robot_manager->KickBall();
  EXPECT_EQ(robot_manager->GetRobotAction(), rob::RobotAction::KICK_BALL);
}

TEST_F(RobotManagerActionsTest, TestPassBallSetsAction) {
  robot_manager->PassBall();
  EXPECT_EQ(robot_manager->GetRobotAction(), rob::RobotAction::PASS_BALL);
}

TEST_F(RobotManagerActionsTest, TestSetRobotAction) {
  robot_manager->SetRobotAction(rob::RobotAction::MOVE);
  EXPECT_EQ(robot_manager->GetRobotAction(), rob::RobotAction::MOVE);
}