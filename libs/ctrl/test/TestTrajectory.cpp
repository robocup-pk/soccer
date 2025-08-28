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

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start_s, t_finish_s);

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

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start_s, t_finish_s);

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

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start_s, t_finish_s);

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

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start_s, t_finish_s);

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

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start_s, t_finish_s);

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
  EXPECT_TRUE(trajectory_manager.active_trajectories.size() == 3);

  trajectory_manager.Print();
}
TEST_F(TrajectoryTest, PositionAtStartTimeIsStartPose) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(t_start);
  EXPECT_NEAR(pos.x(), pose_start.x(), 1e-6);
  EXPECT_NEAR(pos.y(), pose_start.y(), 1e-6);
  EXPECT_NEAR(pos.z(), pose_start.z(), 1e-6);
}

TEST_F(TrajectoryTest, PositionAtEndTimeIsEndPose) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(t_end);
  EXPECT_NEAR(pos.x(), pose_end.x(), 1e-6);
  EXPECT_NEAR(pos.y(), pose_end.y(), 1e-6);
  EXPECT_NEAR(pos.z(), pose_end.z(), 1e-6);
}

TEST_F(TrajectoryTest, PositionAtMidTimeIsMidPose) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(2.0);
  // Midpoint might not be exactly halfway due to acceleration phase
  // so just check it's roughly between
  std::cout << "Position at mid time: " << pos.transpose() << std::endl;

  EXPECT_GT(pos.x(), 0.4);  // Adjust tolerance if needed
  EXPECT_LT(pos.x(), 0.6);
}
TEST_F(TrajectoryTest, PositionAtStartTimeIsStartPose2) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(t_start);
  EXPECT_NEAR(pos.x(), pose_start.x(), 1e-6);
  EXPECT_NEAR(pos.y(), pose_start.y(), 1e-6);
  EXPECT_NEAR(pos.z(), pose_start.z(), 1e-6);
}

TEST_F(TrajectoryTest, PositionAtEndTimeIsEndPose2) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(t_end);
  EXPECT_NEAR(pos.x(), pose_end.x(), 1e-6);
  EXPECT_NEAR(pos.y(), pose_end.y(), 1e-6);
  EXPECT_NEAR(pos.z(), pose_end.z(), 1e-6);
}

TEST_F(TrajectoryTest, PositionAtMidTimeIsMidPose2) {
  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  double t_start = 0.0, t_end = 4.0;

  auto traj = ctrl::TrapezoidalTrajectoryVi3D(pose_start, pose_end, t_start, t_end);
  Eigen::Vector3d pos = traj.PositionAtT(2.0);
  // Midpoint might not be exactly halfway due to acceleration phase
  // so just check it's roughly between
  std::cout << "Position at mid time: " << pos.transpose() << std::endl;
  EXPECT_GT(pos.x(), 0.4);  // Adjust tolerance if needed
  EXPECT_LT(pos.x(), 0.6);
}
TEST_F(TrajectoryTest, PositionInAccelerationPhase) {
  const double top_speed = 0.3;  // m/s
  const Eigen::Vector3d max_acc(0.5, 0.5, 0.5);
  const double t_start = 0.0;
  const double t_end = 4.0;

  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  ctrl::TrapezoidalTrajectoryVi3D traj(pose_start, pose_end, t_start, t_end);

  double t = 0.5;
  Eigen::Vector3d pos = traj.PositionAtT(t);
  std::cout << "[ACCEL] t=" << t << ", pos=" << pos.transpose() << std::endl;

  EXPECT_GT(pos.x(), 0.0);
  EXPECT_LT(pos.x(), 0.1);
}
TEST_F(TrajectoryTest, PositionInCruisePhase) {
  const double top_speed = 0.3;
  const Eigen::Vector3d max_acc(0.5, 0.5, 0.5);
  const double t_start = 0.0;
  const double t_end = 4.0;

  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  ctrl::TrapezoidalTrajectoryVi3D traj(pose_start, pose_end, t_start, t_end);

  double t = 2.0;
  Eigen::Vector3d pos = traj.PositionAtT(t);
  std::cout << "[CRUISE] t=" << t << ", pos=" << pos.transpose() << std::endl;

  EXPECT_GT(pos.x(), 0.3);
  EXPECT_LT(pos.x(), 0.8);
}
TEST_F(TrajectoryTest, PositionInDecelerationPhase) {
  const double top_speed = 0.3;
  const Eigen::Vector3d max_acc(0.5, 0.5, 0.5);
  const double t_start = 0.0;
  const double t_end = 4.0;

  Eigen::Vector3d pose_start(0, 0, 0);
  Eigen::Vector3d pose_end(1, 0, 0);
  ctrl::TrapezoidalTrajectoryVi3D traj(pose_start, pose_end, t_start, t_end);

  double t = 3.7;
  Eigen::Vector3d pos = traj.PositionAtT(t);
  std::cout << "[DECEL] t=" << t << ", pos=" << pos.transpose() << std::endl;

  EXPECT_GT(pos.x(), 0.9);
  EXPECT_LT(pos.x(), 1.01);
}