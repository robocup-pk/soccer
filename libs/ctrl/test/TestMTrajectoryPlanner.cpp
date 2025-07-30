#include <gtest/gtest.h>
#include <iostream>
#include <chrono>
#include "M_TrajectoryPlanner.h"

using namespace ctrl;

class MTrajectoryPlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default constraints for testing
        constraints_.vel_max = 1.0;
        constraints_.acc_max = 2.0;
        constraints_.brk_max = 2.0;
        constraints_.vel_max_w = 8.0;
        constraints_.acc_max_w = 15.0;
    }

    M_MoveConstraints constraints_;
};

// Test 1D Bang-Bang trajectory generation
TEST_F(MTrajectoryPlannerTest, BangBang1D_BasicGeneration) {
    M_BangBangTrajectory1D traj;
    traj.generate(0.0, 2.0, 0.0, 1.0, 2.0);  // 2m displacement, 1m/s max vel, 2m/s² acc
    
    EXPECT_GT(traj.getTotalTime(), 0.0);
    EXPECT_NEAR(traj.getPosition1D(0.0), 0.0, 1e-6);
    EXPECT_NEAR(traj.getPosition1D(traj.getTotalTime()), 2.0, 1e-3);
    EXPECT_NEAR(traj.getVelocity1D(0.0), 0.0, 1e-6);
    EXPECT_NEAR(traj.getVelocity1D(traj.getTotalTime()), 0.0, 1e-3);
    
    std::cout << "[Test1D] Distance: 2m, Time: " << traj.getTotalTime() << "s" << std::endl;
    traj.print();
}

// Test 2D trajectory generation and synchronization
TEST_F(MTrajectoryPlannerTest, BangBang2D_Synchronization) {
    M_BangBangTrajectory2D traj;
    
    Eigen::Vector2d start(0.0, 0.0);
    Eigen::Vector2d end(3.0, 4.0);  // 5m diagonal distance
    Eigen::Vector2d vel0(0.0, 0.0);
    
    traj.generate(start, end, vel0, 1.0, 2.0);
    
    EXPECT_GT(traj.getTotalTime(), 0.0);
    
    // Check start and end positions
    Eigen::Vector2d pos_start = traj.getPosition2D(0.0);
    Eigen::Vector2d pos_end = traj.getPosition2D(traj.getTotalTime());
    
    EXPECT_NEAR(pos_start.x(), 0.0, 1e-6);
    EXPECT_NEAR(pos_start.y(), 0.0, 1e-6);
    EXPECT_NEAR(pos_end.x(), 3.0, 1e-2);
    EXPECT_NEAR(pos_end.y(), 4.0, 1e-2);
    
    std::cout << "[Test2D] Diagonal 3-4-5 triangle, Time: " << traj.getTotalTime() << "s" << std::endl;
    traj.print();
}

// Test 3D trajectory (position + rotation)
TEST_F(MTrajectoryPlannerTest, BangBang3D_PositionAndRotation) {
    M_BangBangTrajectory3D traj;
    
    Eigen::Vector3d start(0.0, 0.0, 0.0);
    Eigen::Vector3d end(2.0, 1.5, M_PI/2);  // Move and rotate 90 degrees
    Eigen::Vector3d vel0(0.0, 0.0, 0.0);
    
    traj.generate(start, end, vel0, constraints_);
    
    EXPECT_GT(traj.getTotalTime(), 0.0);
    
    // Check final position
    Eigen::Vector3d pos_end = traj.getPosition(traj.getTotalTime());
    EXPECT_NEAR(pos_end.x(), 2.0, 1e-2);
    EXPECT_NEAR(pos_end.y(), 1.5, 1e-2);
    EXPECT_NEAR(pos_end.z(), M_PI/2, 1e-2);
    
    std::cout << "[Test3D] Move + Rotate, Time: " << traj.getTotalTime() << "s" << std::endl;
    traj.print();
}

// Test Euclidean path following with primary direction
TEST_F(MTrajectoryPlannerTest, EuclideanPathFollowing) {
    M_BangBangTrajectory2D sync_traj, async_traj;
    
    Eigen::Vector2d start(0.0, 0.0);
    Eigen::Vector2d end(3.0, 4.0);  // 5m diagonal
    Eigen::Vector2d vel0(0.0, 0.0);
    Eigen::Vector2d primary_dir = (end - start).normalized();
    
    // Generate synchronized trajectory
    sync_traj.generate(start, end, vel0, 1.0, 2.0);
    
    // Generate async trajectory with primary direction (should be more direct)
    async_traj.generateAsync(start, end, vel0, 1.0, 2.0, primary_dir);
    
    double sync_time = sync_traj.getTotalTime();
    double async_time = async_traj.getTotalTime();
    
    std::cout << "[EuclideanTest] Sync time: " << sync_time << "s, Async time: " << async_time << "s" << std::endl;
    
    // Sample trajectory at several points to check path quality
    int num_samples = 10;
    double max_deviation_sync = 0.0, max_deviation_async = 0.0;
    
    for (int i = 1; i < num_samples; ++i) {
        double t_sync = (i / double(num_samples)) * sync_time;
        double t_async = (i / double(num_samples)) * async_time;
        
        Eigen::Vector2d pos_sync = sync_traj.getPosition2D(t_sync);
        Eigen::Vector2d pos_async = async_traj.getPosition2D(t_async);
        
        // Calculate deviation from straight line
        Eigen::Vector2d straight_line_pos = start + (end - start) * (i / double(num_samples));
        
        double dev_sync = (pos_sync - straight_line_pos).norm();
        double dev_async = (pos_async - straight_line_pos).norm();
        
        max_deviation_sync = std::max(max_deviation_sync, dev_sync);
        max_deviation_async = std::max(max_deviation_async, dev_async);
    }
    
    std::cout << "[EuclideanTest] Max deviation - Sync: " << max_deviation_sync 
              << "m, Async: " << max_deviation_async << "m" << std::endl;
    
    // Async trajectory should follow straighter path
    EXPECT_LT(max_deviation_async, max_deviation_sync);
}

// Test trajectory planner static methods
TEST_F(MTrajectoryPlannerTest, TrajectoryPlannerMethods) {
    M_RobotState robot_state;
    robot_state.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    robot_state.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    robot_state.constraints = constraints_;
    
    Eigen::Vector3d destination(2.0, 1.5, M_PI/4);
    
    // Test trajectory generation
    auto trajectory = M_TrajectoryPlanner::generatePositionTrajectory(robot_state, destination);
    
    ASSERT_NE(trajectory, nullptr);
    EXPECT_GT(trajectory->getTotalTime(), 0.0);
    
    // Check final destination
    Eigen::Vector3d final_pos = trajectory->getPosition(trajectory->getTotalTime());
    EXPECT_NEAR(final_pos.x(), 2.0, 1e-2);
    EXPECT_NEAR(final_pos.y(), 1.5, 1e-2);
    
    std::cout << "[PlannerTest] Generated trajectory time: " << trajectory->getTotalTime() << "s" << std::endl;
}

// Test velocity constraint enforcement
TEST_F(MTrajectoryPlannerTest, VelocityConstraints) {
    M_BangBangTrajectory2D traj;
    
    Eigen::Vector2d start(0.0, 0.0);
    Eigen::Vector2d end(5.0, 0.0);  // 5m straight line
    Eigen::Vector2d vel0(0.0, 0.0);
    
    double v_max = 0.8;  // Conservative velocity limit
    double a_max = 2.0;
    
    traj.generate(start, end, vel0, v_max, a_max);
    
    // Sample trajectory and check velocity constraints
    double dt = 0.01;  // 10ms sampling
    double max_velocity = 0.0;
    bool constraint_violated = false;
    
    for (double t = 0; t <= traj.getTotalTime(); t += dt) {
        Eigen::Vector2d vel = traj.getVelocity2D(t);
        double speed = vel.norm();
        max_velocity = std::max(max_velocity, speed);
        
        if (speed > v_max + 1e-3) {  // Small tolerance for numerical errors
            constraint_violated = true;
            std::cout << "[VelocityTest] Constraint violated at t=" << t 
                      << "s, speed=" << speed << "m/s" << std::endl;
        }
    }
    
    std::cout << "[VelocityTest] Max velocity reached: " << max_velocity 
              << "m/s (limit: " << v_max << "m/s)" << std::endl;
    
    EXPECT_FALSE(constraint_violated);
    EXPECT_LE(max_velocity, v_max + 1e-3);
}

// Performance benchmark test
TEST_F(MTrajectoryPlannerTest, PerformanceBenchmark) {
    const int num_trajectories = 1000;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_trajectories; ++i) {
        M_RobotState robot_state;
        robot_state.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        robot_state.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        robot_state.constraints = constraints_;
        
        // Random destination
        Eigen::Vector3d destination(
            (rand() % 1000) / 100.0 - 5.0,  // -5 to 5 meters
            (rand() % 1000) / 100.0 - 5.0,
            (rand() % 628) / 100.0 - 3.14   // -π to π radians
        );
        
        auto trajectory = M_TrajectoryPlanner::generatePositionTrajectory(robot_state, destination);
        ASSERT_NE(trajectory, nullptr);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    double avg_time = duration.count() / double(num_trajectories);
    std::cout << "[Performance] Generated " << num_trajectories 
              << " trajectories in " << duration.count() << "μs" << std::endl;
    std::cout << "[Performance] Average time per trajectory: " << avg_time << "μs" << std::endl;
    
    // Should be able to generate trajectories quickly (< 1ms each)
    EXPECT_LT(avg_time, 1000.0);
}

// Test path finder functionality
TEST_F(MTrajectoryPlannerTest, PathFinder) {
    M_PathFinderInput input;
    input.robot_state.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.robot_state.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.robot_state.constraints = constraints_;
    input.destination = Eigen::Vector3d(3.0, 4.0, M_PI/2);
    input.use_primary_direction = true;
    input.primary_direction = Eigen::Vector2d(3.0, 4.0).normalized();
    
    auto result = M_TrajectoryPlanner::findPath(input);
    
    EXPECT_TRUE(result.path_found);
    ASSERT_NE(result.trajectory, nullptr);
    EXPECT_GT(result.total_time, 0.0);
    
    std::cout << "[PathFinder] " << result.info << std::endl;
    std::cout << "[PathFinder] Total time: " << result.total_time << "s" << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "=== M_TrajectoryPlanner Comprehensive Test Suite ===" << std::endl;
    std::cout << "Testing Paper-based Bang-Bang trajectory implementation" << std::endl;
    std::cout << std::endl;
    
    return RUN_ALL_TESTS();
}