#include <gtest/gtest.h>

#include "RRT.h"

TEST(PathPlannerTest, FindSingleRobotPath) {
  state::Waypoint start(0, 0);
  state::Waypoint goal(10, 10);

  algos::RRTParams params = algos::DefaultRRTParams();
  params.max_iterations = 100; // faster test
  state::Path path = algos::FindSinglePath(start, goal, params);

  std::cout << "Path: " << path << std::endl;
}