#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>

#include "RRTX.h"
#include "Waypoint.h"
#include "SoccerObject.h"
#include "SoccerField.h"

class RRTXTest : public ::testing::Test {
 protected:
  void SetUp() override {
    start = state::Waypoint(0.0, 0.0, 0.0);
    goal = state::Waypoint(1.0, 1.0, 0.0);

    // Create RRTX instance
    rrtx = std::make_unique<algos::RRTX>(start, goal);

    // Setup test obstacles
    SetupTestObstacles();
  }

  void SetupTestObstacles() {
    // Static obstacle in the middle
    static_obstacle = state::SoccerObject("static_obs", Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector2d(0.2, 0.2));  // 20cm radius

    // Moving obstacle
    moving_obstacle = state::SoccerObject("moving_obs", Eigen::Vector3d(0.5, 0.5, 0.0),
                                          Eigen::Vector2d(0.15, 0.15));  // 15cm radius

    obstacles = {static_obstacle, moving_obstacle};
  }

  std::unique_ptr<algos::RRTX> rrtx;
  state::Waypoint start, goal;
  double epsilon;
  state::SoccerObject static_obstacle, moving_obstacle;
  std::vector<state::SoccerObject> obstacles;
};

// ============= BASIC FUNCTIONALITY TESTS =============

TEST_F(RRTXTest, ConstructorInitializesCorrectly) {
  EXPECT_EQ(rrtx->Vertices.size(), 2);  // Goal + Start vertices
  EXPECT_EQ(rrtx->v_goal_idx, 0);
  EXPECT_EQ(rrtx->v_start_idx, 1);
  EXPECT_EQ(rrtx->v_bot_idx, 1);

  // Check goal vertex initialization
  EXPECT_EQ(rrtx->Vertices[0].wp.x, goal.x);
  EXPECT_EQ(rrtx->Vertices[0].wp.y, goal.y);
  EXPECT_EQ(rrtx->Vertices[0].g, 0.0);
  EXPECT_EQ(rrtx->Vertices[0].lmc, 0.0);
  EXPECT_EQ(rrtx->Vertices[0].parent_idx, -1);

  // Check start vertex initialization
  EXPECT_EQ(rrtx->Vertices[1].wp.x, start.x);
  EXPECT_EQ(rrtx->Vertices[1].wp.y, start.y);
  EXPECT_EQ(rrtx->Vertices[1].g, std::numeric_limits<double>::infinity());
  EXPECT_EQ(rrtx->Vertices[1].lmc, std::numeric_limits<double>::infinity());
  EXPECT_EQ(rrtx->Vertices[1].parent_idx, -1);
}

TEST_F(RRTXTest, DistanceCalculation) {
  state::Waypoint wp1(0.0, 0.0, 0.0);
  state::Waypoint wp2(3.0, 4.0, 0.0);

  double distance = rrtx->d_pi(wp1, wp2);
  EXPECT_NEAR(distance, 5.0, 1e-6);
}

TEST_F(RRTXTest, ShrinkingBallRadius) {
  // Initially should be large
  double initial_radius = rrtx->ShrinkingBallRadius();
  EXPECT_GT(initial_radius, 0.0);

  // Add more vertices and check if radius shrinks
  for (int i = 0; i < 50; ++i) {
    rrtx->Vertices.emplace_back();
    rrtx->n_samples++;
  }

  double new_radius = rrtx->ShrinkingBallRadius();
  EXPECT_LT(new_radius, initial_radius);
}

// ============= VERTEX MANAGEMENT TESTS =============

TEST_F(RRTXTest, RandomNodeGeneration) {
  for (int i = 0; i < 10; ++i) {
    state::Waypoint random_node = rrtx->RandomNode();

    // Check if node is within field boundaries (approximately)
    EXPECT_GT(random_node.x, vis::SoccerField::GetInstance().playing_area_width_mm / -2000.0);
    EXPECT_LT(random_node.x, vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0);
    EXPECT_GT(random_node.y, vis::SoccerField::GetInstance().playing_area_height_mm / -2000.0);
    EXPECT_LT(random_node.y, vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0);
  }
}

TEST_F(RRTXTest, NearestVertexSearch) {
  // Add test vertices far from existing goal/start
  state::Waypoint v1(0.5, 0.0, 0.0);
  state::Waypoint v2(-0.5, 0.0, 0.0);

  int index2 = rrtx->AddVertex(v1);  // Index 2
  int index3 = rrtx->AddVertex(v2);  // Index 3

  // Query point very close to first added vertex
  state::Waypoint query_point(0.51, 0.01, 0.0);
  int nearest_idx = rrtx->Nearest(query_point);

  EXPECT_EQ(nearest_idx, index2);  // Should find vertex at (0.5, 0)
}

TEST_F(RRTXTest, NearVerticesQuery) {
  // Add test vertices
  for (int i = 0; i < 5; ++i) {
    state::Waypoint wp(i * 0.5, 0.0, 0.0);
    rrtx->AddVertex(wp);
  }

  state::Waypoint query_point(1.0, 0.0, 0.0);
  double radius = 1.0;
  std::vector<int> near_vertices = rrtx->Near(query_point, radius);

  EXPECT_GT(near_vertices.size(), 0);

  // Verify all returned vertices are within radius
  for (int idx : near_vertices) {
    double distance = rrtx->d_pi(query_point, rrtx->Vertices[idx].wp);
    EXPECT_LE(distance, radius);
  }
}

// ============= TREE STRUCTURE TESTS =============

TEST_F(RRTXTest, MakeParentOfBasic) {
  // Add a test vertex
  rrtx->Vertices.emplace_back();
  rrtx->Vertices.back().wp = state::Waypoint(1.0, 0.0, 0.0);
  int child_idx = 2;
  int parent_idx = 0;  // Goal vertex

  rrtx->MakeParentOf(parent_idx, child_idx);

  EXPECT_EQ(rrtx->Vertices[child_idx].parent_idx, parent_idx);
  EXPECT_TRUE(std::find(rrtx->Vertices[parent_idx].C_minus_T.begin(),
                        rrtx->Vertices[parent_idx].C_minus_T.end(),
                        child_idx) != rrtx->Vertices[parent_idx].C_minus_T.end());
}

TEST_F(RRTXTest, MakeParentOfCyclePrevention) {
  // Create chain: 0 -> 2 -> 3
  rrtx->Vertices.emplace_back();  // idx 2
  rrtx->Vertices.emplace_back();  // idx 3

  rrtx->MakeParentOf(0, 2);
  rrtx->MakeParentOf(2, 3);

  // Try to create cycle: 3 -> 0 (should be prevented)
  rrtx->MakeParentOf(3, 0);

  EXPECT_NE(rrtx->Vertices[0].parent_idx, 3);  // Cycle should be prevented
}

TEST_F(RRTXTest, KeyComparisonFunction) {
  std::pair<double, double> key1{1.0, 2.0};
  std::pair<double, double> key2{1.0, 3.0};
  std::pair<double, double> key3{2.0, 1.0};

  EXPECT_TRUE(rrtx->KeyLess(key1, key2));   // Same first, smaller second
  EXPECT_TRUE(rrtx->KeyLess(key1, key3));   // Smaller first
  EXPECT_FALSE(rrtx->KeyLess(key2, key1));  // Same first, larger second
}

TEST_F(RRTXTest, GetKeyFunction) {
  rrtx->Vertices[0].g = 5.0;
  rrtx->Vertices[0].lmc = 3.0;

  auto key = rrtx->getKey(0);
  EXPECT_EQ(key.first, 3.0);   // min(g, lmc)
  EXPECT_EQ(key.second, 5.0);  // g
}

// ============= OBSTACLE HANDLING TESTS =============

TEST_F(RRTXTest, TrajectoryValidityWithoutObstacles) {
  state::Waypoint wp1(0.0, 0.0, 0.0);
  state::Waypoint wp2(1.0, 1.0, 0.0);

  std::vector<state::SoccerObject> empty_obstacles;
  rrtx->UpdateObstacles(empty_obstacles);

  EXPECT_TRUE(rrtx->TrajectoryValid(wp1, wp2));
}

TEST_F(RRTXTest, TrajectoryValidityWithObstacles) {
  state::Waypoint wp1(0.0, 0.0, 0.0);
  state::Waypoint wp2(2.0, 2.0, 0.0);  // Goes through obstacle at (1,1)

  rrtx->UpdateObstacles(obstacles);
  EXPECT_FALSE(rrtx->TrajectoryValid(wp1, wp2));
}

TEST_F(RRTXTest, ObstacleChangeDetection) {
  // Initially no obstacles
  EXPECT_TRUE(rrtx->HasObstaclesChanged(obstacles));

  // Update with obstacles
  rrtx->UpdateObstacles(obstacles);

  // Same obstacles should return false
  EXPECT_FALSE(rrtx->HasObstaclesChanged(obstacles));

  // Modified obstacles should return true
  obstacles[0].position.x() += 0.1;
  EXPECT_TRUE(rrtx->HasObstaclesChanged(obstacles));
}

// ============= PLANNING ALGORITHM TESTS =============

TEST_F(RRTXTest, PlanStepExecution) {
  size_t initial_vertices = rrtx->Vertices.size();

  rrtx->PlanStep();

  // Should add vertices (assuming successful planning)
  EXPECT_GE(rrtx->Vertices.size(), initial_vertices);
}

TEST_F(RRTXTest, SolutionExistence) {
  // Initially no solution should exist
  EXPECT_FALSE(rrtx->SolutionExists());

  // Run planning steps to find solution
  for (int i = 0; i < 100; ++i) {
    rrtx->PlanStep();
    if (rrtx->SolutionExists()) break;
  }
}

TEST_F(RRTXTest, PathReconstruction) {
  // First ensure we have a solution
  for (int i = 0; i < 100; ++i) {
    rrtx->PlanStep();
    if (rrtx->SolutionExists()) break;
  }

  if (rrtx->SolutionExists()) {
    state::Path path = rrtx->ReconstructPath();

    EXPECT_GT(path.size(), 0);

    // Path should start near robot position and end near goal
    EXPECT_NEAR(path.front().x, start.x, 0.1);
    EXPECT_NEAR(path.front().y, start.y, 0.1);
    EXPECT_NEAR(path.back().x, goal.x, 0.1);
    EXPECT_NEAR(path.back().y, goal.y, 0.1);
  }
}

// ============= PERFORMANCE & ROBUSTNESS TESTS =============

TEST_F(RRTXTest, QueueManagement) {
  // Add many vertices and check queue doesn't grow unbounded
  for (int i = 0; i < 200; ++i) {
    rrtx->PlanStep();
  }

  // Queue size should be reasonable relative to vertex count
  EXPECT_LT(rrtx->Q.Size(), rrtx->Vertices.size() * 2);
}