#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include "Trajectory3D.h"
#include "TrajectoryManager.h"

class TrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

TEST_F(TrajectoryTest, TestIsFeasibleOnlyForwardMotion) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  Eigen::Vector3d distance_m_rad = pose_end - pose_start;
  double t_start_s = 0;
  double t_finish_s = 4.0;

  auto [feasible, t_acc] = ctrl::Trajectory3D::IsFeasible(distance_m_rad, t_finish_s);

  EXPECT_TRUE(feasible);

  auto traj = ctrl::TrapezoidalTrajectory3D(pose_start, pose_end, t_start_s, t_finish_s);

  traj.Print();

  // Check if it covers the distance
  Eigen::Vector3d distance = traj.TotalDistance();

  EXPECT_NEAR(distance[0], pose_end[0], 0.01);  // 1 cm error tolerance
  EXPECT_NEAR(distance[1], pose_end[1], 0.01);
  EXPECT_NEAR(distance[2], pose_end[2], 0.01);
}

TEST_F(TrajectoryTest, TestIsFeasibleOnlyBackwardMotion) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(-1, 0, 0);
  Eigen::Vector3d distance_m_rad = pose_end - pose_start;
  double t_start_s = 0;
  double t_finish_s = 4.0;

  auto [feasible, t_acc] = ctrl::Trajectory3D::IsFeasible(distance_m_rad, t_finish_s);

  EXPECT_TRUE(feasible);

  auto traj = ctrl::TrapezoidalTrajectory3D(pose_start, pose_end, t_start_s, t_finish_s);

  traj.Print();

  // Check if it covers the distance
  Eigen::Vector3d distance = traj.TotalDistance();

  EXPECT_NEAR(distance[0], pose_end[0], 0.01);
  EXPECT_NEAR(distance[1], pose_end[1], 0.01);
  EXPECT_NEAR(distance[2], pose_end[2], 0.01);
}

TEST_F(TrajectoryTest, TestIsFeasibleSidewaysMotion) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(0, -1, 0);
  Eigen::Vector3d distance_m_rad = pose_end - pose_start;
  double t_start_s = 0;
  double t_finish_s = 4.0;

  auto [feasible, t_acc] = ctrl::Trajectory3D::IsFeasible(distance_m_rad, t_finish_s);

  EXPECT_TRUE(feasible);

  auto traj = ctrl::TrapezoidalTrajectory3D(pose_start, pose_end, t_start_s, t_finish_s);

  traj.Print();

  // Check if it covers the distance
  Eigen::Vector3d distance = traj.TotalDistance();

  EXPECT_NEAR(distance[0], pose_end[0], 0.01);
  EXPECT_NEAR(distance[1], pose_end[1], 0.01);
  EXPECT_NEAR(distance[2], pose_end[2], 0.01);
}

TEST_F(TrajectoryTest, TestIsFeasibleDiagonalMotion) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, -1, 0);
  Eigen::Vector3d distance_m_rad = pose_end - pose_start;
  double t_start_s = 0;
  double t_finish_s = 4.0;

  auto [feasible, t_acc] = ctrl::Trajectory3D::IsFeasible(distance_m_rad, t_finish_s);

  EXPECT_TRUE(feasible);

  auto traj = ctrl::TrapezoidalTrajectory3D(pose_start, pose_end, t_start_s, t_finish_s);

  traj.Print();

  // Check if it covers the distance
  Eigen::Vector3d distance = traj.TotalDistance();

  EXPECT_NEAR(distance[0], pose_end[0], 0.01);
  EXPECT_NEAR(distance[1], pose_end[1], 0.01);
  EXPECT_NEAR(distance[2], pose_end[2], 0.01);
}

TEST_F(TrajectoryTest, TestIsFeasibleDiagonalAndRotationMotion) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, -1, 1);
  Eigen::Vector3d distance_m_rad = pose_end - pose_start;
  double t_start_s = 0;
  double t_finish_s = 4.0;

  auto [feasible, t_acc] = ctrl::Trajectory3D::IsFeasible(distance_m_rad, t_finish_s);

  EXPECT_TRUE(feasible);

  auto traj = ctrl::TrapezoidalTrajectory3D(pose_start, pose_end, t_start_s, t_finish_s);

  traj.Print();

  // Check if it covers the distance
  Eigen::Vector3d distance = traj.TotalDistance();

  EXPECT_NEAR(distance[0], pose_end[0], 0.01);
  EXPECT_NEAR(distance[1], pose_end[1], 0.01);
  EXPECT_NEAR(distance[2], pose_end[2], 0.01);
}

TEST_F(TrajectoryTest, TestPathToTrajectoryConversation) {
  std::vector<Eigen::Vector3d> path;
  path.push_back(Eigen::Vector3d(0, 0, 0));
  path.push_back(Eigen::Vector3d(-1, 0, 0));
  path.push_back(Eigen::Vector3d(0, 0, 0));
  path.push_back(Eigen::Vector3d(1, 0, 0));
  path.push_back(Eigen::Vector3d(2, 0, 0));

  ctrl::TrajectoryManager trajectory_manager;
  bool valid_trajectories = trajectory_manager.CreateTrajectoriesFromPath(path);

  EXPECT_TRUE(valid_trajectories);
  EXPECT_TRUE(trajectory_manager.active_trajectories.size() == 4);

  trajectory_manager.Print();
}