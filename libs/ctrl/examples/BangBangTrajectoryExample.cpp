#include "BangBangTrajectory.h"
#include <iostream>
#include <iomanip>

using namespace ctrl;

/**
 * @brief Example demonstrating BangBangTrajectory usage
 * 
 * This example shows how to:
 * 1. Create a trajectory generator with custom robot parameters
 * 2. Generate trajectories between waypoints
 * 3. Evaluate trajectories at specific times
 * 4. Integrate with existing RobotManager system
 */

void printTrajectoryInfo(const TrajectoryComplete& traj) {
    std::cout << "=== Trajectory Information ===" << std::endl;
    std::cout << "Total execution time: " << traj.total_execution_time << " seconds" << std::endl;
    
    std::cout << "\nX Trajectory (" << traj.translation.x_trajectory.segments.size() << " segments):" << std::endl;
    for (size_t i = 0; i < traj.translation.x_trajectory.segments.size(); ++i) {
        const auto& seg = traj.translation.x_trajectory.segments[i];
        std::cout << "  Segment " << i << ": ";
        switch (seg.case_type) {
            case BangBangCase::CASE_1: std::cout << "CASE_1 (reverse)"; break;
            case BangBangCase::CASE_2_1: std::cout << "CASE_2_1 (accel)"; break;
            case BangBangCase::CASE_2_2: std::cout << "CASE_2_2 (cruise)"; break;
            case BangBangCase::CASE_2_3: std::cout << "CASE_2_3 (decel)"; break;
            case BangBangCase::CASE_3: std::cout << "CASE_3 (overspeed)"; break;
        }
        std::cout << ", duration: " << std::fixed << std::setprecision(3) << seg.duration 
                  << "s, distance: " << seg.distance_traveled << "m" << std::endl;
    }
    
    std::cout << "\nY Trajectory (" << traj.translation.y_trajectory.segments.size() << " segments):" << std::endl;
    for (size_t i = 0; i < traj.translation.y_trajectory.segments.size(); ++i) {
        const auto& seg = traj.translation.y_trajectory.segments[i];
        std::cout << "  Segment " << i << ": ";
        switch (seg.case_type) {
            case BangBangCase::CASE_1: std::cout << "CASE_1 (reverse)"; break;
            case BangBangCase::CASE_2_1: std::cout << "CASE_2_1 (accel)"; break;
            case BangBangCase::CASE_2_2: std::cout << "CASE_2_2 (cruise)"; break;
            case BangBangCase::CASE_2_3: std::cout << "CASE_2_3 (decel)"; break;
            case BangBangCase::CASE_3: std::cout << "CASE_3 (overspeed)"; break;
        }
        std::cout << ", duration: " << std::fixed << std::setprecision(3) << seg.duration 
                  << "s, distance: " << seg.distance_traveled << "m" << std::endl;
    }
    
    std::cout << "\nSynchronization α: " << traj.translation.alpha_sync * 180.0 / M_PI << " degrees" << std::endl;
}

void demonstrateBasicUsage() {
    std::cout << "\n=== Basic Usage Example ===" << std::endl;
    
    // Create trajectory generator with SSL robot parameters
    RobotParams params;
    params.mass_kg = 2.3;
    params.inertia_z = 0.0085;
    params.friction_coeff = 0.8;
    params.wheel_force_max = 1.5;
    
    BangBangTrajectory trajectory_gen(params);
    
    // Define current and target states
    TrajectoryState current_state;
    current_state.position << 0.0, 0.0, 0.0;        // [x, y, θ] in meters/radians
    current_state.velocity << 0.5, -0.3, 0.0;       // [ẋ, ẏ, θ̇] in m/s, rad/s
    current_state.v_max = 2.0;                       // Max linear velocity
    current_state.a_max = 3.0;                       // Max linear acceleration
    current_state.theta_dot_max = 10.0;              // Max angular velocity
    current_state.theta_ddot_max = 20.0;             // Max angular acceleration
    
    TrajectoryState target_state;
    target_state.position << 1.5, 1.0, M_PI/4;      // Target: (1.5m, 1.0m, 45°)
    target_state.target_velocity << 0.0, 0.0, 0.0;  // Stop at target
    
    // Generate trajectory
    TrajectoryComplete trajectory = trajectory_gen.GenerateTrajectory(current_state, target_state);
    
    printTrajectoryInfo(trajectory);
    
    // Evaluate trajectory at different time points
    std::cout << "\n=== Trajectory Evaluation ===" << std::endl;
    std::cout << "Time(s)  | Position(x,y,θ)           | Velocity(ẋ,ẏ,θ̇)          | Acceleration(ẍ,ÿ,θ̈)" << std::endl;
    std::cout << "---------|---------------------------|---------------------------|-------------------------" << std::endl;
    
    for (double t = 0.0; t <= trajectory.total_execution_time; t += trajectory.total_execution_time / 10.0) {
        auto [pos, vel, acc] = trajectory_gen.EvaluateTrajectory(trajectory, t);
        std::cout << std::fixed << std::setprecision(2) << std::setw(8) << t << " | ";
        std::cout << "(" << std::setw(5) << pos[0] << "," << std::setw(5) << pos[1] << "," << std::setw(5) << pos[2] << ") | ";
        std::cout << "(" << std::setw(5) << vel[0] << "," << std::setw(5) << vel[1] << "," << std::setw(5) << vel[2] << ") | ";
        std::cout << "(" << std::setw(5) << acc[0] << "," << std::setw(5) << acc[1] << "," << std::setw(5) << acc[2] << ")" << std::endl;
    }
}

void demonstrateWaypointTrajectory() {
    std::cout << "\n=== Waypoint Trajectory Example ===" << std::endl;
    
    BangBangTrajectory trajectory_gen;
    
    // Define a sequence of waypoints
    std::vector<Eigen::Vector3d> waypoints = {
        Eigen::Vector3d(0.0, 0.0, 0.0),      // Start
        Eigen::Vector3d(1.0, 0.0, 0.0),      // Move right
        Eigen::Vector3d(1.0, 1.0, M_PI/2),   // Move up and turn
        Eigen::Vector3d(0.0, 1.0, M_PI),     // Move left and turn
        Eigen::Vector3d(0.0, 0.0, -M_PI/2)   // Return to start with different orientation
    };
    
    Eigen::Vector3d initial_velocity(0.2, 0.0, 0.0); // Start with some forward velocity
    
    auto trajectories = trajectory_gen.GenerateWaypointTrajectory(waypoints, initial_velocity);
    
    std::cout << "Generated " << trajectories.size() << " trajectory segments" << std::endl;
    
    double total_time = 0.0;
    for (size_t i = 0; i < trajectories.size(); ++i) {
        std::cout << "\nSegment " << i << " (waypoint " << i << " → " << (i+1) << "):" << std::endl;
        std::cout << "  Duration: " << trajectories[i].total_execution_time << " seconds" << std::endl;
        total_time += trajectories[i].total_execution_time;
    }
    
    std::cout << "\nTotal mission time: " << total_time << " seconds" << std::endl;
}

void demonstrateIntegrationWithRobotManager() {
    std::cout << "\n=== Integration with RobotManager Example ===" << std::endl;
    
    // This shows how BangBangTrajectory could be integrated with the existing system
    std::cout << "Integration points with existing system:" << std::endl;
    std::cout << "1. RobotManager::SetPath() - Replace current trajectory generation" << std::endl;
    std::cout << "2. TrajectoryManager - Use BangBangTrajectory for optimal paths" << std::endl;
    std::cout << "3. MotionController - Consume bang-bang acceleration commands" << std::endl;
    std::cout << "4. StateEstimator - Provide current robot state for trajectory generation" << std::endl;
    
    // Example pseudo-integration:
    BangBangTrajectory trajectory_gen;
    
    // Simulate getting current state from StateEstimator
    TrajectoryState current_state;
    current_state.position << 0.5, -0.3, 0.2;       // From pose_fWorld
    current_state.velocity << 0.1, 0.2, 0.0;        // From velocity estimation
    current_state.v_max = 2.0;                       // From SystemConfig
    current_state.a_max = 3.0;                       // From acceleration limits
    current_state.theta_dot_max = 10.0;
    current_state.theta_ddot_max = 20.0;
    
    // Target from path planning
    TrajectoryState target_state;
    target_state.position << 2.0, 1.5, -0.5;
    target_state.target_velocity = Eigen::Vector3d::Zero();
    
    TrajectoryComplete optimal_trajectory = trajectory_gen.GenerateTrajectory(current_state, target_state);
    
    std::cout << "\nGenerated optimal trajectory:" << std::endl;
    std::cout << "  Execution time: " << optimal_trajectory.total_execution_time << "s" << std::endl;
    std::cout << "  X segments: " << optimal_trajectory.translation.x_trajectory.segments.size() << std::endl;
    std::cout << "  Y segments: " << optimal_trajectory.translation.y_trajectory.segments.size() << std::endl;
    std::cout << "  Rotation segments: " << optimal_trajectory.rotation.segments.size() << std::endl;
    
    // Show how to extract velocity commands for RobotManager
    std::cout << "\nVelocity commands for first 1 second:" << std::endl;
    std::cout << "Time | Velocity Command [vx, vy, ω]" << std::endl;
    for (double t = 0.0; t <= std::min(1.0, optimal_trajectory.total_execution_time); t += 0.1) {
        auto [pos, vel, acc] = trajectory_gen.EvaluateTrajectory(optimal_trajectory, t);
        std::cout << std::fixed << std::setprecision(1) << t << "s | [" 
                  << std::setprecision(2) << vel[0] << ", " << vel[1] << ", " << vel[2] << "]" << std::endl;
    }
}

int main() {
    std::cout << "BangBangTrajectory Implementation Example" << std::endl;
    std::cout << "Based on Purwin & D'Andrea (2006) Algorithm" << std::endl;
    std::cout << "Custom wheel configuration: [-30°, 45°, 135°, -150°]" << std::endl;
    std::cout << "SSL Robot Parameters: mass=2.3kg, inertia=0.0085kg⋅m², μ=0.8, Fmax=1.5N" << std::endl;
    
    demonstrateBasicUsage();
    demonstrateWaypointTrajectory();
    demonstrateIntegrationWithRobotManager();
    
    std::cout << "\n=== Implementation Complete ===" << std::endl;
    std::cout << "Key features implemented:" << std::endl;
    std::cout << "✓ 5 bang-bang cases from Section 4" << std::endl;
    std::cout << "✓ X-Y velocity synchronization (Section 6)" << std::endl;
    std::cout << "✓ Theta trajectory generation (Section 7)" << std::endl;
    std::cout << "✓ Custom wheel configuration support" << std::endl;
    std::cout << "✓ SSL-specific robot parameters" << std::endl;
    std::cout << "✓ Waypoint trajectory generation" << std::endl;
    std::cout << "✓ Real-time trajectory evaluation" << std::endl;
    
    return 0;
}