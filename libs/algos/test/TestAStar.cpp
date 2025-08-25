#include <gtest/gtest.h>
#include <memory>
#include "AStar.h"
#include "SoccerObject.h"
#include "Waypoint.h"

class AStarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gridResolution = 0.1f;  // 10cm per grid cell
    astar = std::make_unique<algos::AStar>(gridResolution);

    start = state::Waypoint(0.0f, 0.0f, 0.0f);
    goal = state::Waypoint(1.0f, 1.0f, 0.0f);

    SetupTestObstacles();
  }

  void SetupTestObstacles() {
    // Circular obstacle near the center
    static_obstacle = state::SoccerObject("static_obs", Eigen::Vector3d(0.5, 0.5, 0.0),  // pos
                                          Eigen::Vector2d(0.2, 0.2));  // radius 0.2m
    obstacles = {static_obstacle};
  }

  std::unique_ptr<algos::AStar> astar;
  float gridResolution;
  state::Waypoint start, goal;
  state::SoccerObject static_obstacle;
  std::vector<state::SoccerObject> obstacles;
};

// ============= BASIC TESTS =============

TEST_F(AStarTest, ConstructorInitializesGrid) {
  EXPECT_GT(astar->gridWidth, 0);
  EXPECT_GT(astar->gridHeight, 0);
  EXPECT_EQ(astar->obstacleGrid.size(), astar->gridWidth);
}

TEST_F(AStarTest, HeuristicCalculatesEuclideanDistance) {
  algos::Node a(0, 0);
  algos::Node b(3, 4);

  float dist = astar->heuristic(a, b);
  EXPECT_NEAR(dist, 0.5f, 1e-6);
  // Explanation: dx=3*0.1=0.3, dy=4*0.1=0.4 → sqrt(0.3²+0.4²)=0.5
}

TEST_F(AStarTest, WorldToGridAndBack) {
  state::Waypoint wp(0.2f, 0.2f, 0.0f);
  algos::Node n = astar->worldToGrid(wp);
  state::Waypoint wp2 = astar->gridToWorld(n);

  EXPECT_GE(n.x, 0);
  EXPECT_GE(n.y, 0);
  EXPECT_NEAR(wp2.x, wp.x, gridResolution);
  EXPECT_NEAR(wp2.y, wp.y, gridResolution);
}

// ============= PATHFINDING TESTS =============

TEST_F(AStarTest, FindsStraightLinePathWithoutObstacles) {
  state::Path path = astar->findPath(start, goal, {});
  EXPECT_FALSE(path.empty());

  // Map start and goal to grid and back to world coordinates
  algos::Node startNode = astar->worldToGrid(start);
  algos::Node goalNode = astar->worldToGrid(goal);
  state::Waypoint startMapped = astar->gridToWorld(startNode);
  state::Waypoint goalMapped = astar->gridToWorld(goalNode);

  double tol = 1e-6;  // ~half grid cell
  EXPECT_NEAR(path.front().x, startMapped.x, tol);
  EXPECT_NEAR(path.back().x, goalMapped.x, tol);

  EXPECT_GT(path.size(), 1);  // must contain at least 2 waypoints
}

TEST_F(AStarTest, NoPathIfGoalInsideObstacle) {
  // Place goal inside the static obstacle
  state::Waypoint blockedGoal(0.5f, 0.5f, 0.0f);

  state::Path path = astar->findPath(start, blockedGoal, obstacles);
  EXPECT_TRUE(path.empty());
}

TEST_F(AStarTest, FindsPathAroundObstacle) {
  state::Path path = astar->findPath(start, goal, obstacles);
  EXPECT_FALSE(path.empty());

  // Map start and goal to grid and back to world coordinates
  algos::Node startNode = astar->worldToGrid(start);
  algos::Node goalNode = astar->worldToGrid(goal);
  state::Waypoint startMapped = astar->gridToWorld(startNode);
  state::Waypoint goalMapped = astar->gridToWorld(goalNode);

  double tol = 1e-6;  // ~half grid cell
  EXPECT_NEAR(path.front().x, startMapped.x, tol);
  EXPECT_NEAR(path.front().y, startMapped.y, tol);
  EXPECT_NEAR(path.back().x, goalMapped.x, tol);
  EXPECT_NEAR(path.back().y, goalMapped.y, tol);

  // Ensure path does not directly pass through the obstacle
  for (auto& wp : path) {
    float dx = wp.x - 0.5f;
    float dy = wp.y - 0.5f;
    float dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_GT(dist, 0.2f - 1e-3);  // outside obstacle radius
  }
}

TEST_F(AStarTest, PathSmoothingRemovesUnnecessaryPoints) {
  state::Path rawPath = {{0, 0, 0}, {0.5, 0.25, 0}, {1, 1, 0}};
  state::Path smoothed = astar->smoothPath(rawPath);

  // Expect the middle point removed if direct line is clear
  EXPECT_EQ(smoothed.size(), 2);
  EXPECT_EQ(smoothed.front().x, 0.0);
  EXPECT_EQ(smoothed.back().x, 1.0);
}