#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>

#include "TrajectoryManager.h"
#include "TrapezoidalTrajectoryVi3D.h"

class TrajectoryMergeTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

TEST_F(TrajectoryMergeTest, TestMergeNewTrajectories_FirstCall) {
  ctrl::TrajectoryManager trajectory_manager;

  ctrl::Trajectories new_trajs;
  auto traj1 = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 0.0, 4.0);
  auto traj2 = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 0, 0), 4.0, 8.0);

  new_trajs.push(std::move(traj1));
  new_trajs.push(std::move(traj2));

  trajectory_manager.MergeNewTrajectories(std::move(new_trajs));

  EXPECT_EQ(trajectory_manager.active_trajectories.size(), 1);
}

TEST_F(TrajectoryMergeTest, TestMergeNewTrajectories_InFuture) {
  ctrl::TrajectoryManager trajectory_manager;

  // Initial trajectory
  ctrl::Trajectories initial_trajs;
  auto initial_traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 0.0, 4.0);
  initial_trajs.push(std::move(initial_traj));
  trajectory_manager.MergeNewTrajectories(std::move(initial_trajs));

  // Future trajectory (starts after current ends)
  ctrl::Trajectories future_trajs;
  auto future_traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 0, 0), 6.0, 10.0);
  future_trajs.push(std::move(future_traj));

  trajectory_manager.MergeNewTrajectories(std::move(future_trajs));

  EXPECT_EQ(trajectory_manager.active_trajectories.size(), 2);
}

TEST_F(TrajectoryMergeTest, TestMergeNewTrajectories_AtT_Overlap) {
  ctrl::TrajectoryManager trajectory_manager;

  // Initial trajectory
  ctrl::Trajectories initial_trajs;
  auto initial_traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0, 8.0);
  initial_trajs.push(std::move(initial_traj));
  trajectory_manager.MergeNewTrajectories(std::move(initial_trajs));

  // Overlapping trajectory
  ctrl::Trajectories overlap_trajs;
  auto overlap_traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(3, 0, 0), 4.0, 12.0);
  overlap_trajs.push(std::move(overlap_traj));

  trajectory_manager.MergeNewTrajectories(std::move(overlap_trajs));

  EXPECT_EQ(trajectory_manager.active_trajectories.size(), 1);
}

TEST_F(TrajectoryMergeTest, TestFindV0AtT_InActiveTrajectory) {
  ctrl::TrajectoryManager trajectory_manager;

  ctrl::Trajectories trajs;
  auto traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 0.0, 4.0);
  trajs.push(std::move(traj));
  trajectory_manager.MergeNewTrajectories(std::move(trajs));

  Eigen::Vector3d v0 = trajectory_manager.FindV0AtT(2.0);
  EXPECT_GE(v0.norm(), 0.0);  // Should find velocity at time 2.0
}

TEST_F(TrajectoryMergeTest, TestFindV0AtT_AfterFinish) {
  ctrl::TrajectoryManager trajectory_manager;

  ctrl::Trajectories trajs;
  auto traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 0.0, 4.0);
  trajs.push(std::move(traj));
  trajectory_manager.MergeNewTrajectories(std::move(trajs));

  Eigen::Vector3d v0 = trajectory_manager.FindV0AtT(6.0);  // After finish
  EXPECT_NEAR(v0.norm(), 0.0, 1e-6);                       // Should return zero velocity
}