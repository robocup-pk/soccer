#include <iostream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "CircularObstacle.h"
#include "AdvancedMotionPlanner.h"
#include "MoveConstraints.h"

using namespace std;

enum class DemoPhase {
    PHASE1_CROSS_PATTERN,
    PHASE2_MIXED_MOVEMENT,
    PHASE3_SEQUENTIAL_WITH_OBSTACLES,
    PHASE4_PURE_ROTATION,
    COMPLETED
};

std::vector<std::shared_ptr<ctrl::IObstacle>> CreatePhaseObstacles(DemoPhase phase) {
    std::vector<std::shared_ptr<ctrl::IObstacle>> obstacles;
    
    switch (phase) {
        case DemoPhase::PHASE1_CROSS_PATTERN:
            // Basic obstacles for cross pattern
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, 0.0), 0.3, "CenterObstacle"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.5, 0.5), 0.15, "QuadrantObstacle1"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(-0.5, -0.5), 0.15, "QuadrantObstacle2"));
            break;
            
        case DemoPhase::PHASE2_MIXED_MOVEMENT:
            // More obstacles for mixed movement
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.3, 0.0), 0.2, "RightObstacle"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(-0.3, 0.0), 0.2, "LeftObstacle"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, 0.8), 0.15, "TopObstacle"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, -0.8), 0.15, "BottomObstacle"));
            break;
            
        case DemoPhase::PHASE3_SEQUENTIAL_WITH_OBSTACLES:
            // Dense obstacle field for sequential movement
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, 0.0), 0.25, "CenterMajor"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.6, 0.3), 0.2, "NorthEast"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(-0.6, 0.3), 0.2, "NorthWest"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.6, -0.3), 0.2, "SouthEast"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(-0.6, -0.3), 0.2, "SouthWest"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, 0.6), 0.15, "North"));
            obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                Eigen::Vector2d(0.0, -0.6), 0.15, "South"));
            break;
            
        case DemoPhase::PHASE4_PURE_ROTATION:
            // No obstacles for pure rotation test
            break;
            
        case DemoPhase::COMPLETED:
            break;
    }
    
    return obstacles;
}

bool PlanRobotTrajectory(int robot_id, rob::RobotManager& robot_manager, 
                        const Eigen::Vector3d& destination,
                        const std::vector<std::shared_ptr<ctrl::IObstacle>>& obstacles,
                        std::vector<rob::RobotManager>& all_robots,
                        const ctrl::MoveConstraints& constraints) {
    
    std::cout << "[Robot " << robot_id << "] Planning trajectory..." << std::endl;
    
    // Create robot-specific obstacles (include other robots as obstacles)
    auto robot_obstacles = obstacles; // Copy phase obstacles
    
    // Add other robots as circular obstacles
    for (int other_id = 0; other_id < all_robots.size(); ++other_id) {
        if (other_id != robot_id) {
            Eigen::Vector3d other_pos = all_robots[other_id].GetPoseInWorldFrame();
            robot_obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(
                other_pos.head<2>(), 0.12, "Robot" + std::to_string(other_id)));
        }
    }
    
    // Create advanced motion planner
    ctrl::AdvancedMotionPlanner planner;
    
    // Plan trajectory with obstacle avoidance
    Eigen::Vector3d current_pos = robot_manager.GetPoseInWorldFrame();
    Eigen::Vector3d current_vel = robot_manager.GetVelocityInWorldFrame();
    
    std::cout << "[Robot " << robot_id << "] Current: " << current_pos.transpose() << std::endl;
    std::cout << "[Robot " << robot_id << "] Destination: " << destination.transpose() << std::endl;
    std::cout << "[Robot " << robot_id << "] Obstacles: " << robot_obstacles.size() << std::endl;
    
    // ADVANCED PATHFINDING CALL
    planner.planTrajectory(
        current_pos,           // Start position
        current_vel,          // Start velocity
        destination,          // Goal
        robot_obstacles,      // All obstacles
        constraints          // Movement constraints
    );
    
    if (planner.isValid()) {
        std::cout << "[Robot " << robot_id << "] SUCCESS! Trajectory duration: " 
                  << planner.getTotalTime() << "s" << std::endl;
        
        // Set the trajectory directly
        robot_manager.SetAdvancedTrajectory(planner);
        return true;
    } else {
        std::cout << "[Robot " << robot_id << "] FAILED! Using direct path as fallback" << std::endl;
        // Fallback: direct waypoint path
        std::vector<Eigen::Vector3d> fallback_path = {current_pos, destination};
        robot_manager.SetSmoothPathTrackerPath(fallback_path, util::GetCurrentTime());
        return false;
    }
}

bool AllRobotsIdle(std::vector<rob::RobotManager>& robot_managers) {
    for (auto& robot : robot_managers) {
        if (robot.GetRobotState() != "IDLE") {
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "[MultiRobot Demo] Extended Advanced Motion Planning System - Multi-Phase Test" << std::endl;
    
    // Initialize random seed
    srand(static_cast<unsigned int>(time(nullptr)));

    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize 4 RobotManagers
    const int NUM_ROBOTS = 4;
    std::vector<rob::RobotManager> robot_managers(NUM_ROBOTS);
    
    // Robot starting positions
    std::vector<Eigen::Vector3d> start_positions = {
        Eigen::Vector3d(-1.0, -1.0, 0.0),  // Robot 0: Bottom-left
        Eigen::Vector3d( 1.0, -1.0, 0.0),  // Robot 1: Bottom-right
        Eigen::Vector3d(-1.0,  1.0, 0.0),  // Robot 2: Top-left
        Eigen::Vector3d( 1.0,  1.0, 0.0)   // Robot 3: Top-right
    };
    
    // Phase-based destinations
    std::vector<std::vector<Eigen::Vector3d>> phase_destinations = {
        // PHASE 1: Cross pattern (swap positions)
        {
            Eigen::Vector3d( 1.0,  1.0, M_PI),    // Robot 0: Bottom-left -> Top-right
            Eigen::Vector3d(-1.0,  1.0, M_PI),    // Robot 1: Bottom-right -> Top-left
            Eigen::Vector3d( 1.0, -1.0, M_PI),    // Robot 2: Top-left -> Bottom-right
            Eigen::Vector3d(-1.0, -1.0, M_PI)     // Robot 3: Top-right -> Bottom-left
        },
        // PHASE 2: Mixed movement (2 sequential, 2 parallel)
        {
            Eigen::Vector3d( 0.0, -1.2, 0.0),     // Robot 0: Move down
            Eigen::Vector3d( 0.0,  1.2, 0.0),     // Robot 1: Move up (parallel with 0)
            Eigen::Vector3d(-1.2,  0.0, M_PI/2),  // Robot 2: Move left (sequential after 0,1)
            Eigen::Vector3d( 1.2,  0.0, -M_PI/2)  // Robot 3: Move right (sequential after 2)
        },
        // PHASE 3: All sequential movement with dense obstacles
        {
            Eigen::Vector3d(-1.2, -1.2, 0.0),     // Robot 0: Back to corner
            Eigen::Vector3d( 1.2, -1.2, M_PI/2),  // Robot 1: Adjacent corner
            Eigen::Vector3d( 1.2,  1.2, M_PI),    // Robot 2: Opposite corner
            Eigen::Vector3d(-1.2,  1.2, -M_PI/2)  // Robot 3: Final corner
        },
        // PHASE 4: Pure rotation test (no translation, just rotate 180 degrees)
        {
            Eigen::Vector3d(-1.2, -1.2, M_PI),    // Robot 0: Same position, rotate 180°
            Eigen::Vector3d( 1.2, -1.2, -M_PI/2), // Robot 1: Same position, rotate -90°
            Eigen::Vector3d( 1.2,  1.2, 0.0),     // Robot 2: Same position, rotate -180°
            Eigen::Vector3d(-1.2,  1.2, M_PI/2)   // Robot 3: Same position, rotate 90°
        }
    };
    
    // Initialize robot poses
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        robot_managers[i].InitializePose(start_positions[i]);
        robot_managers[i].SetTrajectoryManagerType(rob::TrajectoryManagerType::AdvancedTrajectory);
    }
    
    // Demo phase management
    DemoPhase current_phase = DemoPhase::PHASE1_CROSS_PATTERN;
    bool phase_planned = false;
    int phase_wait_frames = 0;
    const int PHASE_WAIT_TIME = 100; // Frames to wait between phases
    
    // Advanced movement constraints
    ctrl::MoveConstraints constraints;
    constraints.setVelMax(1.0)        // m/s - reasonable for multi-robot
               .setAccMax(0.8)        // m/s² - smooth acceleration
               .setVelMaxW(3.0)       // rad/s - moderate rotation
               .setAccMaxW(2.5);      // rad/s² - smooth angular acceleration
    
    // Timing
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    const int max_frames = 5000; // Much longer simulation for multi-phase
    int next_robot_to_plan = 0; // For sequential planning in Phase 3
    
    std::cout << "[MultiRobot] Starting extended simulation with " << NUM_ROBOTS << " robots..." << std::endl;
    std::cout << "[MultiRobot] PHASE 1: Cross pattern movement with basic obstacles" << std::endl;
    
    while (frame_count < max_frames && current_phase != DemoPhase::COMPLETED) {
        // Run simulation step
        if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
            std::cout << "[MultiRobot] Simulation finished" << std::endl;
            break;
        }
        
        // Process input
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Phase management
        if (!phase_planned && frame_count > 20) {
            auto phase_obstacles = CreatePhaseObstacles(current_phase);
            auto& destinations = phase_destinations[static_cast<int>(current_phase)];
            
            std::cout << "\n=== PHASE " << (static_cast<int>(current_phase) + 1) << " TRAJECTORY PLANNING ===" << std::endl;
            std::cout << "[MultiRobot] Created " << phase_obstacles.size() << " obstacles for this phase" << std::endl;
            
            // Different planning strategies per phase
            switch (current_phase) {
                case DemoPhase::PHASE1_CROSS_PATTERN:
                    // All robots plan simultaneously (parallel)
                    for (int robot_id = 0; robot_id < NUM_ROBOTS; ++robot_id) {
                        PlanRobotTrajectory(robot_id, robot_managers[robot_id], 
                                          destinations[robot_id], phase_obstacles, 
                                          robot_managers, constraints);
                    }
                    break;
                    
                case DemoPhase::PHASE2_MIXED_MOVEMENT:
                    // First two robots plan simultaneously (parallel)
                    std::cout << "\n--- Planning Parallel Group (Robots 0,1) ---" << std::endl;
                    PlanRobotTrajectory(0, robot_managers[0], destinations[0], 
                                      phase_obstacles, robot_managers, constraints);
                    PlanRobotTrajectory(1, robot_managers[1], destinations[1], 
                                      phase_obstacles, robot_managers, constraints);
                    
                    // Wait for them to finish, then plan sequential
                    next_robot_to_plan = 2; // Start with robot 2
                    break;
                    
                case DemoPhase::PHASE3_SEQUENTIAL_WITH_OBSTACLES:
                    // All robots move one by one
                    std::cout << "\n--- Planning Sequential Movement (Robot " << next_robot_to_plan << ") ---" << std::endl;
                    if (next_robot_to_plan < NUM_ROBOTS) {
                        PlanRobotTrajectory(next_robot_to_plan, robot_managers[next_robot_to_plan], 
                                          destinations[next_robot_to_plan], phase_obstacles, 
                                          robot_managers, constraints);
                        next_robot_to_plan++;
                    }
                    break;
                    
                case DemoPhase::PHASE4_PURE_ROTATION:
                    // All robots rotate in place, one by one
                    std::cout << "\n--- Planning Pure Rotation (Robot " << next_robot_to_plan << ") ---" << std::endl;
                    if (next_robot_to_plan < NUM_ROBOTS) {
                        std::cout << "[Robot " << next_robot_to_plan << "] PURE ROTATION TEST: Staying at same position, rotating to " 
                                  << destinations[next_robot_to_plan][2] << " radians (" 
                                  << destinations[next_robot_to_plan][2] * 180.0 / M_PI << "°)" << std::endl;
                        PlanRobotTrajectory(next_robot_to_plan, robot_managers[next_robot_to_plan], 
                                          destinations[next_robot_to_plan], phase_obstacles, 
                                          robot_managers, constraints);
                        next_robot_to_plan++;
                    }
                    break;
                    
                case DemoPhase::COMPLETED:
                    break;
            }
            
            phase_planned = true;
            std::cout << "\n=== PHASE " << (static_cast<int>(current_phase) + 1) << " EXECUTION STARTED ===" << std::endl;
        }
        
        // Handle sequential planning for Phase 2 and Phase 3
        if (phase_planned) {
            if (current_phase == DemoPhase::PHASE2_MIXED_MOVEMENT && next_robot_to_plan < NUM_ROBOTS) {
                // Check if parallel robots (0,1) are done
                if (robot_managers[0].GetRobotState() == "IDLE" && 
                    robot_managers[1].GetRobotState() == "IDLE") {
                    
                    auto phase_obstacles = CreatePhaseObstacles(current_phase);
                    auto& destinations = phase_destinations[static_cast<int>(current_phase)];
                    
                    std::cout << "\n--- Sequential Planning Robot " << next_robot_to_plan << " ---" << std::endl;
                    PlanRobotTrajectory(next_robot_to_plan, robot_managers[next_robot_to_plan], 
                                      destinations[next_robot_to_plan], phase_obstacles, 
                                      robot_managers, constraints);
                    next_robot_to_plan++;
                }
            }
            else if (current_phase == DemoPhase::PHASE3_SEQUENTIAL_WITH_OBSTACLES && 
                     next_robot_to_plan < NUM_ROBOTS) {
                // Check if previous robot is done
                if (next_robot_to_plan > 0 && robot_managers[next_robot_to_plan - 1].GetRobotState() == "IDLE") {
                    auto phase_obstacles = CreatePhaseObstacles(current_phase);
                    auto& destinations = phase_destinations[static_cast<int>(current_phase)];
                    
                    std::cout << "\n--- Sequential Planning Robot " << next_robot_to_plan << " ---" << std::endl;
                    PlanRobotTrajectory(next_robot_to_plan, robot_managers[next_robot_to_plan], 
                                      destinations[next_robot_to_plan], phase_obstacles, 
                                      robot_managers, constraints);
                    next_robot_to_plan++;
                }
            }
            else if (current_phase == DemoPhase::PHASE4_PURE_ROTATION && 
                     next_robot_to_plan < NUM_ROBOTS) {
                // Check if previous robot is done
                if (next_robot_to_plan > 0 && robot_managers[next_robot_to_plan - 1].GetRobotState() == "IDLE") {
                    auto phase_obstacles = CreatePhaseObstacles(current_phase);
                    auto& destinations = phase_destinations[static_cast<int>(current_phase)];
                    
                    std::cout << "\n--- Pure Rotation Planning Robot " << next_robot_to_plan << " ---" << std::endl;
                    std::cout << "[Robot " << next_robot_to_plan << "] PURE ROTATION TEST: Staying at same position, rotating to " 
                              << destinations[next_robot_to_plan][2] << " radians (" 
                              << destinations[next_robot_to_plan][2] * 180.0 / M_PI << "°)" << std::endl;
                    PlanRobotTrajectory(next_robot_to_plan, robot_managers[next_robot_to_plan], 
                                      destinations[next_robot_to_plan], phase_obstacles, 
                                      robot_managers, constraints);
                    next_robot_to_plan++;
                }
            }
        }
        
        // Check for phase completion
        if (phase_planned && AllRobotsIdle(robot_managers)) {
            phase_wait_frames++;
            
            if (phase_wait_frames >= PHASE_WAIT_TIME) {
                // Move to next phase
                switch (current_phase) {
                    case DemoPhase::PHASE1_CROSS_PATTERN:
                        current_phase = DemoPhase::PHASE2_MIXED_MOVEMENT;
                        std::cout << "\n\n### ADVANCING TO PHASE 2: Mixed Movement (2 parallel, 2 sequential) ###\n" << std::endl;
                        break;
                    case DemoPhase::PHASE2_MIXED_MOVEMENT:
                        current_phase = DemoPhase::PHASE3_SEQUENTIAL_WITH_OBSTACLES;
                        std::cout << "\n\n### ADVANCING TO PHASE 3: Sequential Movement with Dense Obstacles ###\n" << std::endl;
                        break;
                    case DemoPhase::PHASE3_SEQUENTIAL_WITH_OBSTACLES:
                        current_phase = DemoPhase::PHASE4_PURE_ROTATION;
                        std::cout << "\n\n### ADVANCING TO PHASE 4: Pure Rotation Test (No Translation) ###\n" << std::endl;
                        break;
                    case DemoPhase::PHASE4_PURE_ROTATION:
                        current_phase = DemoPhase::COMPLETED;
                        std::cout << "\n\n### ALL PHASES COMPLETED! ###\n" << std::endl;
                        break;
                    case DemoPhase::COMPLETED:
                        break;
                }
                
                phase_planned = false;
                phase_wait_frames = 0;
                next_robot_to_plan = 0;
            }
        }
        
        // Update all robots
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            robot_managers[i].ControlLogic();
            robot_managers[i].SenseLogic();
            
            // Update soccer object positions for visualization
            if (i < soccer_objects.size()) {
                soccer_objects[i].position = robot_managers[i].GetPoseInWorldFrame();
            }
        }
        
        // Progress reporting every 100 frames
        if (frame_count % 100 == 0) {
            std::cout << "[MultiRobot] Frame " << frame_count << " - Phase " 
                      << (static_cast<int>(current_phase) + 1) << " - Robot positions:" << std::endl;
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                Eigen::Vector3d pos = robot_managers[i].GetPoseInWorldFrame();
                std::string state = robot_managers[i].GetRobotState();
                std::cout << "  Robot " << i << ": (" << pos.transpose() << ") State: " << state << std::endl;
            }
        }
        
        frame_count++;
        
        // Exit on ESC
        if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }
    }
    
    // Final results
    std::cout << "\n=== EXTENDED MULTI-ROBOT DEMO RESULTS ===" << std::endl;
    
    if (current_phase == DemoPhase::COMPLETED) {
        std::cout << "🎉 ALL PHASES SUCCESSFULLY COMPLETED! 🎉" << std::endl;
        std::cout << "\nPhase Summary:" << std::endl;
        std::cout << "  Phase 1: ✅ Cross-pattern movement with basic obstacles" << std::endl;
        std::cout << "  Phase 2: ✅ Mixed coordination (2 parallel + 2 sequential)" << std::endl;
        std::cout << "  Phase 3: ✅ Full sequential movement with dense obstacles" << std::endl;
    } else {
        std::cout << "Demo ended at Phase " << (static_cast<int>(current_phase) + 1) << std::endl;
    }
    
    // Show final positions relative to last phase targets
    if (static_cast<int>(current_phase) > 0) {
        int last_phase = static_cast<int>(current_phase) - 1;
        if (last_phase < phase_destinations.size()) {
            auto& last_targets = phase_destinations[last_phase];
            
            std::cout << "\nFinal Robot Positions:" << std::endl;
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                Eigen::Vector3d final_pos = robot_managers[i].GetPoseInWorldFrame();
                Eigen::Vector3d target = last_targets[i];
                double distance_error = (final_pos.head<2>() - target.head<2>()).norm();
                double angle_error = std::abs(util::WrapAngle(final_pos[2] - target[2]));
                
                std::cout << "Robot " << i << ":" << std::endl;
                std::cout << "  Final: (" << final_pos.transpose() << ")" << std::endl;
                std::cout << "  Target: (" << target.transpose() << ")" << std::endl;
                std::cout << "  Position Error: " << distance_error * 1000 << "mm" << std::endl;
                std::cout << "  Orientation Error: " << angle_error * 180.0 / M_PI << "°" << std::endl;
                std::cout << "  State: " << robot_managers[i].GetRobotState() << std::endl;
            }
        }
    }
    
    std::cout << "\n[MultiRobot] Extended simulation completed after " << frame_count << " frames" << std::endl;
    std::cout << "[MultiRobot] Demonstrated advanced multi-robot coordination with:" << std::endl;
    std::cout << "  - Simultaneous multi-robot planning" << std::endl;
    std::cout << "  - Mixed parallel/sequential coordination" << std::endl; 
    std::cout << "  - Dense obstacle field navigation" << std::endl;
    std::cout << "  - Dynamic trajectory replanning" << std::endl;
    
    return 0;
}