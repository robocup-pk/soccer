#include <gtest/gtest.h>

#include "RRT.h"
#include "SoccerField.h"

TEST(RRTTest, TestFindSingleRobotPath) {
  state::Waypoint start(0, 0);
  state::Waypoint goal(1, 1);

  state::Path path = algos::FindSinglePath(start, goal);

  EXPECT_GT(path.size(), 0);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
}

TEST(RRTTest, TestRandomWaypointGeneration) {
  state::Waypoint start(0, 0);
  state::Waypoint goal(1, 1);

  state::Waypoint random_wp = algos::FindRandomWaypoint(goal);

  EXPECT_TRUE(random_wp.x >= 0 &&
              abs(random_wp.x) <= vis::SoccerField::GetInstance().width_mm / 2);
  EXPECT_TRUE(random_wp.y >= 0 &&
              abs(random_wp.y) <= vis::SoccerField::GetInstance().height_mm / 2);
}