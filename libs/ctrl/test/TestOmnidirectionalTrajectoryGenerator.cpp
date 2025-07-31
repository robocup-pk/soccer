#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "OmnidirectionalTrajectoryGenerator.h"

class OmnidirectionalTrajectoryGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    generator = std::make_unique<ctrl::OmnidirectionalTrajectoryGenerator>();
  }

  std::unique_ptr<ctrl::OmnidirectionalTrajectoryGenerator> generator;
  const double tolerance = 1e-6;
};

TEST_F(OmnidirectionalTrajectoryGeneratorTest, Initialization) {
  ASSERT_NE(generator, nullptr);

  double a, b, angle;
  generator->getEnvelopeParameters(a, b, angle);

  EXPECT_GT(a, 0.0);
  EXPECT_GT(b, 0.0);
}

TEST_F(OmnidirectionalTrajectoryGeneratorTest, GoalReached) {
  Eigen::Vector3d current_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d goal_pose(0.005, 0.005, 0.01);  // Within tolerance

  EXPECT_TRUE(generator->isGoalReached(current_pose, goal_pose));
}

TEST_F(OmnidirectionalTrajectoryGeneratorTest, GoalNotReached) {
  Eigen::Vector3d current_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d goal_pose(1.0, 1.0, 1.0);  // Outside tolerance

  EXPECT_FALSE(generator->isGoalReached(current_pose, goal_pose));
}

TEST_F(OmnidirectionalTrajectoryGeneratorTest, TrajectoryGeneration) {
  Eigen::Vector3d current_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d goal_pose(1.0, 1.0, 0.0);

  auto [finished, velocity] = generator->Update(current_pose, goal_pose);

  EXPECT_FALSE(finished);           // Should not be finished immediately
  EXPECT_GT(velocity.norm(), 0.0);  // Should generate non-zero velocity
}

TEST_F(OmnidirectionalTrajectoryGeneratorTest, TrajectoryFinishedWhenGoalReached) {
  Eigen::Vector3d current_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d goal_pose(0.005, 0.005, 0.01);  // Within tolerance

  auto [finished, velocity] = generator->Update(current_pose, goal_pose);

  EXPECT_TRUE(finished);                         // Should be finished when goal is reached
  EXPECT_NEAR(velocity.norm(), 0.0, tolerance);  // Should generate zero velocity
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
