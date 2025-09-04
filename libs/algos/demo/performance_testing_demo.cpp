#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <random>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"
#include "Kick.h"

using namespace std;

// Multi-robot configuration
constexpr int NUM_ROBOTS = 6;
constexpr bool ENABLE_GUI = true;
constexpr bool ENABLE_REALISTIC_NOISE = true;
constexpr bool ENABLE_REPLANNING = true;
constexpr double ROBOT_RADIUS = 0.102; // Robot radius in meters (half of 204mm diameter)
constexpr double SAFETY_MARGIN = 0.05;  // Additional 5cm safety margin

// Execution phases
enum class ExecutionPhase {
    PHASE_1_TWO_ROBOTS,    // First 2 robots run simultaneously 
    PHASE_2_ROBOT_3,       // Robot 2 runs alone
    PHASE_3_ROBOT_4,       // Robot 3 runs alone
    PHASE_4_ROBOT_5,       // Robot 4 runs alone
    PHASE_5_ROBOT_6,       // Robot 5 runs alone
    FINISHED
};

// Performance tracking structure
struct RobotPerformance {
    int robot_id;
    // Cross-track
    double total_error = 0.0;   // legacy: cross-track sum
    double max_error = 0.0;     // legacy: cross-track max
    // Along-track (arc-length difference)
    double total_along_abs = 0.0;
    double max_along_abs = 0.0;
    // Heading error (rad)
    double total_heading_abs = 0.0;
    double max_heading_abs = 0.0;
    int frame_count = 0;
    std::vector<double> error_history;
    bool trajectory_completed = false;
    double completion_time = 0.0;
    bool allowed_to_run = false;  // Controls which robots are active in each phase
};

// Collision detection function
bool CheckCollision(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
    double distance = (pos1.head<2>() - pos2.head<2>()).norm();
    double min_safe_distance = 2.0 * ROBOT_RADIUS + SAFETY_MARGIN;
    return distance < min_safe_distance;
}

// Cross-track error w.r.t. the current planner path (ignores time schedule).
// This matches the analysis style used in trajectory_analysis where the robot
// is compared to the closest point on the ideal geometric path.
double CrossTrackErrorToPlanner(ctrl::UniformBSplineTrajectoryPlanner& planner,
                                const Eigen::Vector3d& pose)
{
    if (planner.IsActive()) {
        // Coarse-to-fine sampling along the current spline parameter [0,1]
        const int coarse = 60;  // 1.7% resolution
        double best_u = 0.0;
        double best_d2 = std::numeric_limits<double>::max();
        for (int i = 0; i <= coarse; ++i) {
            double u = static_cast<double>(i) / coarse;
            Eigen::Vector3d p = planner.EvaluateBSplineAtParameter(u);
            double d2 = (p.head<2>() - pose.head<2>()).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; best_u = u; }
        }
        // Refine locally around best_u
        const int refine = 20;
        double u0 = std::max(0.0, best_u - 1.0 / coarse);
        double u1 = std::min(1.0, best_u + 1.0 / coarse);
        for (int i = 0; i <= refine; ++i) {
            double u = u0 + (u1 - u0) * (static_cast<double>(i) / refine);
            Eigen::Vector3d p = planner.EvaluateBSplineAtParameter(u);
            double d2 = (p.head<2>() - pose.head<2>()).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; }
        }
        return std::sqrt(best_d2);
    }
    if (planner.IsFinished()) {
        Eigen::Vector3d p = planner.EvaluateBSplineAtParameter(1.0);
        return (p.head<2>() - pose.head<2>()).norm();
    }
    return 0.0; // inactive and not finished
}

// Return closest spline parameter u in [0,1] to a given pose
double ClosestUToPlanner(ctrl::UniformBSplineTrajectoryPlanner& planner,
                         const Eigen::Vector3d& pose)
{
    if (!planner.IsActive() && !planner.IsFinished()) return 0.0;
    const int coarse = 60;
    double best_u = 0.0;
    double best_d2 = std::numeric_limits<double>::max();
    for (int i = 0; i <= coarse; ++i) {
        double u = static_cast<double>(i) / coarse;
        Eigen::Vector3d p = planner.EvaluateBSplineAtParameter(u);
        double d2 = (p.head<2>() - pose.head<2>()).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best_u = u; }
    }
    const int refine = 20;
    double u0 = std::max(0.0, best_u - 1.0 / coarse);
    double u1 = std::min(1.0, best_u + 1.0 / coarse);
    for (int i = 0; i <= refine; ++i) {
        double u = u0 + (u1 - u0) * (static_cast<double>(i) / refine);
        Eigen::Vector3d p = planner.EvaluateBSplineAtParameter(u);
        double d2 = (p.head<2>() - pose.head<2>()).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best_u = u; }
    }
    return best_u;
}

// Predefined test trajectories for each robot (completely separated areas)
std::vector<std::vector<Eigen::Vector3d>> GetTestTrajectories() {
    std::vector<std::vector<Eigen::Vector3d>> trajectories(NUM_ROBOTS);
    
    // PHASE 1: Two robots running simultaneously in separate areas
    
    // Robot 0: Left side square path (isolated area)
    trajectories[0] = {
        Eigen::Vector3d(-1.2, -0.8, 0),
        Eigen::Vector3d(-0.8, -0.8, 0),
        Eigen::Vector3d(-0.8, -0.4, 1.5708),
        Eigen::Vector3d(-1.2, -0.4, 3.14159),
        Eigen::Vector3d(-1.2, -0.8, -1.5708)
    };
    
    // Robot 1: Right side figure-8 path (isolated area)
    trajectories[1] = {
        Eigen::Vector3d(0.8, -0.6, 0),
        Eigen::Vector3d(1.0, -0.4, 0.785),
        Eigen::Vector3d(1.2, -0.6, 0),
        Eigen::Vector3d(1.0, -0.8, -0.785),
        Eigen::Vector3d(0.8, -0.6, 0)
    };
    
    // PHASE 2-5: Individual robots in separate areas
    
    // Robot 2: Top center triangular path
    trajectories[2] = {
        Eigen::Vector3d(-0.3, 0.6, 0),
        Eigen::Vector3d(0.3, 0.6, 0),
        Eigen::Vector3d(0.0, 1.0, 2.094),
        Eigen::Vector3d(-0.3, 0.6, -2.094)
    };
    
    // Robot 3: Bottom left circular arc
    trajectories[3] = {
        Eigen::Vector3d(-1.0, 0.2, 0),
        Eigen::Vector3d(-0.8, 0.4, 1.5708),
        Eigen::Vector3d(-1.0, 0.6, 3.14159),
        Eigen::Vector3d(-1.2, 0.4, -1.5708),
        Eigen::Vector3d(-1.0, 0.2, 0)
    };
    
    // Robot 4: Center area S-curve path
    trajectories[4] = {
        Eigen::Vector3d(-0.4, 0.0, 0),
        Eigen::Vector3d(-0.2, 0.2, -0.785),
        Eigen::Vector3d(0.0, 0.0, 0),
        Eigen::Vector3d(0.2, -0.2, 0.785),
        Eigen::Vector3d(0.4, 0.0, 0)
    };
    
    // Robot 5: Bottom right diamond path
    trajectories[5] = {
        Eigen::Vector3d(0.8, 0.2, 0),
        Eigen::Vector3d(1.0, 0.4, 0.785),
        Eigen::Vector3d(1.2, 0.2, -0.785),
        Eigen::Vector3d(1.0, 0.0, -2.356),
        Eigen::Vector3d(0.8, 0.2, 0.785)
    };
    
    return trajectories;
}

// Realistic SSL noise simulation
Eigen::Vector3d ApplyRealisticNoise(const Eigen::Vector3d& true_pose) {
    if (!ENABLE_REALISTIC_NOISE) return true_pose;
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<double> vision_noise(0.0, 0.005);  // 5mm std dev
    static std::normal_distribution<double> orientation_noise(0.0, 0.02);  // ~1.1° std dev
    static std::uniform_real_distribution<double> dropout_prob(0.0, 1.0);
    
    // Simulate occasional vision dropout (5% chance)
    if (dropout_prob(gen) < 0.05) {
        return true_pose; // Return old pose (simulating lost frame)
    }
    
    Eigen::Vector3d noisy_pose = true_pose;
    noisy_pose[0] += vision_noise(gen);
    noisy_pose[1] += vision_noise(gen);
    noisy_pose[2] += orientation_noise(gen);
    
    return noisy_pose;
}

// Calculate tracking error (legacy helper not used for cross-track now)
double CalculateTrackingError(const Eigen::Vector3d& current_pose, const Eigen::Vector3d& desired_pose) {
    return (current_pose.head<2>() - desired_pose.head<2>()).norm();
}

// Update performance metrics
void UpdatePerformanceMetrics(RobotPerformance& perf,
                              double cross_track_m,
                              double along_abs_m,
                              double heading_abs_rad,
                              double current_time) {
    perf.total_error += cross_track_m;
    perf.max_error = std::max(perf.max_error, cross_track_m);
    perf.total_along_abs += along_abs_m;
    perf.max_along_abs = std::max(perf.max_along_abs, along_abs_m);
    perf.total_heading_abs += heading_abs_rad;
    perf.max_heading_abs = std::max(perf.max_heading_abs, heading_abs_rad);
    perf.frame_count++;
    perf.error_history.push_back(cross_track_m);
}

// Print performance summary
void PrintPerformanceSummary(const std::vector<RobotPerformance>& performances) {
    std::cout << "\n========== MULTI-ROBOT PERFORMANCE SUMMARY ==========\n";
    
    double overall_mean_error = 0.0;
    double overall_max_error = 0.0;
    int completed_trajectories = 0;
    int active_robots = 0;
    
    for (const auto& perf : performances) {
        double mean_error = perf.frame_count > 0 ? (perf.total_error / perf.frame_count) : 0.0; // cross-track mean
        double mean_along = perf.frame_count > 0 ? (perf.total_along_abs / perf.frame_count) : 0.0;
        double mean_heading_deg = perf.frame_count > 0 ? (perf.total_heading_abs / perf.frame_count) * 180.0 / M_PI : 0.0;
        if (perf.frame_count > 0) {
            overall_mean_error += mean_error;
            active_robots++;
        }
        overall_max_error = std::max(overall_max_error, perf.max_error);
        
        if (perf.trajectory_completed) completed_trajectories++;
        
        std::cout << "Robot " << perf.robot_id << ":\n";
        std::cout << "  Cross-track Mean: " << (mean_error * 1000.0) << "mm\n";
        std::cout << "  Cross-track Max:  " << (perf.max_error * 1000.0) << "mm\n";
        std::cout << "  Along-track Mean: " << (mean_along * 1000.0) << "mm\n";
        std::cout << "  Along-track Max:  " << (perf.max_along_abs * 1000.0) << "mm\n";
        std::cout << "  Heading Mean:     " << mean_heading_deg << " deg\n";
        std::cout << "  Heading Max:      " << (perf.max_heading_abs * 180.0 / M_PI) << " deg\n";
        std::cout << "  Frames:     " << perf.frame_count << "\n";
        std::cout << "  Completed:  " << (perf.trajectory_completed ? "Yes" : (perf.frame_count>0?"No":"Not scheduled")) << "\n";
        if (perf.trajectory_completed) {
            std::cout << "  Time:       " << perf.completion_time << "s\n";
        }
        std::cout << "\n";
    }
    
    overall_mean_error = (active_robots > 0) ? (overall_mean_error / active_robots) : 0.0;
    
    std::cout << "OVERALL PERFORMANCE:\n";
    std::cout << "  Average Mean Error: " << (overall_mean_error * 1000.0) << "mm\n";
    std::cout << "  System Max Error:   " << (overall_max_error * 1000.0) << "mm\n";
    std::cout << "  Completion Rate:    " << completed_trajectories << "/" << active_robots 
              << " (" << (active_robots>0 ? (100.0 * completed_trajectories / active_robots) : 0.0) << "%)\n";
    std::cout << "====================================================\n";
}

int main(int argc, char* argv[]) {
    std::cout << "========== MULTI-ROBOT TRAJECTORY PERFORMANCE TESTING ==========\n";
    std::cout << "Configuration:\n";
    std::cout << "  Robots: " << NUM_ROBOTS << "\n";
    std::cout << "  GUI: " << (ENABLE_GUI ? "Enabled" : "Disabled") << "\n";
    std::cout << "  Realistic Noise: " << (ENABLE_REALISTIC_NOISE ? "Enabled" : "Disabled") << "\n";
    std::cout << "  Replanning: " << (ENABLE_REPLANNING ? "Enabled" : "Disabled") << "\n";
    std::cout << "=================================================================\n\n";
    
    // Initialize random seed
    srand(static_cast<unsigned int>(time(nullptr)));
    
    // Initialize simulation objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    
    vis::GLSimulation gl_simulation;
    if (ENABLE_GUI) {
        gl_simulation.InitGameObjects(soccer_objects);
    }
    
    // Initialize robot managers for each robot
    std::vector<rob::RobotManager> robot_managers(NUM_ROBOTS);
    std::vector<RobotPerformance> performances(NUM_ROBOTS);
    std::vector<std::vector<Eigen::Vector3d>> trajectories = GetTestTrajectories();
    
    // Configure each robot
    for (int i = 0; i < NUM_ROBOTS; i++) {
        performances[i].robot_id = i;
        
        // Start each robot exactly at the first waypoint of its own trajectory
        // so they do not have to traverse across the field to reach the path.
        // This makes the multi-robot test collision-free by design.
        Eigen::Vector3d start_pose = trajectories[i].front();
        
        robot_managers[i].InitializePose(start_pose);
        robot_managers[i].SetTrajectoryManagerType(rob::TrajectoryManagerType::UniformBSpline);
        
        // Configure trajectory planner
        auto& planner = robot_managers[i].GetUniformBSplinePlanner();
        planner.SetLimits(0.8, 0.5, 0.8, 0.5);
        planner.SetFeedbackGains(0.1, 0.05);
        planner.SetReplanningEnabled(ENABLE_REPLANNING);
        planner.SetVerbose(false);
        
        // Set trajectory for this robot
        robot_managers[i].SetUniformBSplinePath(trajectories[i]);
        
        std::cout << "Robot " << i << " initialized with " << trajectories[i].size() 
                  << " waypoints, starting at (" << start_pose.transpose() << ")\n";
    }
    
    std::cout << "\nStarting performance test...\n\n";
    
    // Main simulation loop with phase management
    const double dt = 1.0/60.0; // 60 FPS
    const double max_simulation_time = 60.0; // 60 seconds max
    double simulation_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    ExecutionPhase current_phase = ExecutionPhase::PHASE_1_TWO_ROBOTS;
    std::vector<double> trajectory_start_times(NUM_ROBOTS, 0.0);
    
    std::cout << "Starting PHASE 1: Two robots running simultaneously (Robot 0 and Robot 1)\n";
    
    while (simulation_time < max_simulation_time && current_phase != ExecutionPhase::FINISHED) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        
        // Run GUI simulation step first
        if (ENABLE_GUI) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
                std::cout << "GUI simulation finished\n";
                break;
            }
            // Process input for proper window handling
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        }
        
        // Run all robots simultaneously in collision-free regions
        std::vector<bool> robot_allowed(NUM_ROBOTS, true);
        
        bool any_robot_active = false;
        bool phase_completed = true;
        
        // Update each robot
        for (int i = 0; i < NUM_ROBOTS; i++) {
            auto& manager = robot_managers[i];
            auto& perf = performances[i];
            
            if (!robot_allowed[i] || perf.trajectory_completed) continue;
            
            // Initialize trajectory start time for this robot
            if (trajectory_start_times[i] == 0.0) {
                trajectory_start_times[i] = simulation_time;
            }
            
            // Desired pose for this frame (planner clock). When the planner is finished,
            // GetCurrentDesiredPosition() returns zero (inactive). Use the final waypoint instead
            // to avoid a large spurious error spike at completion.
            Eigen::Vector3d desired_pose;
            auto& planner_ref = manager.GetUniformBSplinePlanner();
            if (planner_ref.IsActive()) {
                desired_pose = planner_ref.GetCurrentDesiredPosition();
            } else if (planner_ref.IsFinished()) {
                desired_pose = planner_ref.EvaluateBSplineAtParameter(1.0);
            } else {
                desired_pose = manager.GetPoseInWorldFrame();
            }

            // Check for collisions with other active robots
            bool collision_detected = false;
            Eigen::Vector3d current_pose = manager.GetPoseInWorldFrame();
            
            for (int j = 0; j < NUM_ROBOTS; j++) {
                if (i != j && robot_allowed[j] && !performances[j].trajectory_completed) {
                    Eigen::Vector3d other_pose = robot_managers[j].GetPoseInWorldFrame();
                    if (CheckCollision(current_pose, other_pose)) {
                        collision_detected = true;
                        std::cout << "COLLISION DETECTED between Robot " << i << " and Robot " << j << "!\n";
                        break;
                    }
                }
            }
            
            // Only run robot if no collision detected
            if (!collision_detected) {
                // Run robot control and sense logic
                manager.ControlLogic();
                manager.SenseLogic();
            }

            // Measure cross-track, along-track, and heading errors
            {
                Eigen::Vector3d true_pose = manager.GetPoseInWorldFrame();
                auto& planner = manager.GetUniformBSplinePlanner();
                double cross_track = CrossTrackErrorToPlanner(planner, true_pose);
                // Along-track: difference in parameter mapped to arc length (approx via total length)
                double u_robot = ClosestUToPlanner(planner, true_pose);
                Eigen::Vector3d ref_pose = planner.IsActive() ? planner.GetCurrentDesiredPosition()
                                                              : planner.EvaluateBSplineAtParameter(1.0);
                double u_ref = ClosestUToPlanner(planner, ref_pose);
                double along_abs = std::abs(u_ref - u_robot) * planner.GetTotalArcLength();
                // Heading error at robot's closest point
                Eigen::Vector3d d1 = planner.GetTangentAt(u_robot);
                double heading_ref = std::atan2(d1[1], d1[0]);
                double heading_err = util::WrapAngle(heading_ref - true_pose[2]);
                double heading_abs = std::abs(heading_err);
                UpdatePerformanceMetrics(perf, cross_track, along_abs, heading_abs, simulation_time);
            }
            
            // Consider a robot completed when its planner reports finished,
            // or as a fallback when it exceeds a generous max time per trajectory.
            bool planner_finished = manager.GetUniformBSplinePlanner().IsFinished();
            bool trajectory_completed = planner_finished || (simulation_time - trajectory_start_times[i] > 20.0);
            if (trajectory_completed) {
                if (!perf.trajectory_completed) {
                    perf.trajectory_completed = true;
                    perf.completion_time = simulation_time;
                    std::cout << "Robot " << i << " completed trajectory at t=" 
                              << simulation_time << "s\n";
                }
            } else {
                any_robot_active = true;
                phase_completed = false; // This robot is still running
            }
        }
        
        // Check if we should advance to next phase
        if (phase_completed && current_phase != ExecutionPhase::FINISHED) {
            switch (current_phase) {
                case ExecutionPhase::PHASE_1_TWO_ROBOTS:
                    current_phase = ExecutionPhase::PHASE_2_ROBOT_3;
                    std::cout << "\nPhase 1 completed. Starting PHASE 2: Robot 2 running alone\n";
                    break;
                case ExecutionPhase::PHASE_2_ROBOT_3:
                    current_phase = ExecutionPhase::PHASE_3_ROBOT_4;
                    std::cout << "\nPhase 2 completed. Starting PHASE 3: Robot 3 running alone\n";
                    break;
                case ExecutionPhase::PHASE_3_ROBOT_4:
                    current_phase = ExecutionPhase::PHASE_4_ROBOT_5;
                    std::cout << "\nPhase 3 completed. Starting PHASE 4: Robot 4 running alone\n";
                    break;
                case ExecutionPhase::PHASE_4_ROBOT_5:
                    current_phase = ExecutionPhase::PHASE_5_ROBOT_6;
                    std::cout << "\nPhase 4 completed. Starting PHASE 5: Robot 5 running alone\n";
                    break;
                case ExecutionPhase::PHASE_5_ROBOT_6:
                    current_phase = ExecutionPhase::FINISHED;
                    std::cout << "\nAll phases completed!\n";
                    break;
                case ExecutionPhase::FINISHED:
                    break;
            }
        }
        
        // Update robot positions in soccer_objects for GUI
        if (ENABLE_GUI) {
            for (int i = 0; i < NUM_ROBOTS && i < soccer_objects.size(); i++) {
                Eigen::Vector3d pose = robot_managers[i].GetPoseInWorldFrame();
                soccer_objects[i].position = pose;
            }
        }
        
        // Exit if all robots completed their trajectories
        if (!any_robot_active) {
            std::cout << "\nAll robots completed their trajectories!\n";
            break;
        }
        
        simulation_time += dt;
        
        // Frame rate limiting
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
        auto target_duration = std::chrono::microseconds(static_cast<long>(dt * 1000000));
        
        if (loop_duration < target_duration) {
            std::this_thread::sleep_for(target_duration - loop_duration);
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "\nSimulation completed in " << (total_duration.count() / 1000.0) << " seconds\n";
    
    // Print performance summary
    PrintPerformanceSummary(performances);
    
    // GUI cleanup handled automatically by GLSimulation destructor
    
    return 0;
}
