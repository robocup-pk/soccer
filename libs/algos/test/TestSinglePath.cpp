#include <gtest/gtest.h>

#include "RRT.h"

TEST(PathPlannerTest, FindSingleRobotPath) {
  state::Waypoint start(0, 0);
  state::Waypoint goal(10, 10);

  state::Path path = algos::FindSinglePath(start, goal);

  std::cout << "Path: " << path << std::endl;
}