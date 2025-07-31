#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "M_TrajectoryPlanner.h"
#include "M_TrajectorySegment.h"

int main() {
    std::cout << "=== Simple Trajectory Test ===" << std::endl;
    
    // Create 3 simple waypoints in a straight line
    std::vector<Eigen::Vector3d> waypoints = {
        Eigen::Vector3d(0.0, 0.0, 0.0),      // Start
        Eigen::Vector3d(-0.5, 0.0, 0.0),     // Middle
        Eigen::Vector3d(-1.0, 0.0, 0.0)      // End
    };
    
    // Setup constraints
    ctrl::M_MoveConstraints constraints;
    constraints.vel_max = 1.0;
    constraints.acc_max = 2.0;
    constraints.vel_max_w = 3.0;
    constraints.acc_max_w = 5.0;
    
    // Robot state
    ctrl::M_RobotState robot_state;
    robot_state.position = waypoints[0];
    robot_state.velocity = Eigen::Vector3d::Zero();
    robot_state.constraints = constraints;
    
    std::cout << "Creating multi-waypoint trajectory..." << std::endl;
    
    try {
        auto trajectory = ctrl::M_MultiWaypointPlanner::generateMultiWaypointTrajectory(
            robot_state, waypoints, constraints
        );
        
        std::cout << "SUCCESS: Multi-waypoint trajectory created" << std::endl;
        std::cout << "Total time: " << trajectory->getTotalTime() << "s" << std::endl;
        trajectory->print();
        
        // Test trajectory sampling
        std::cout << "\nTrajectory sampling:" << std::endl;
        for (double t = 0; t <= trajectory->getTotalTime(); t += 0.5) {
            Eigen::Vector3d pos = trajectory->getPosition(t);
            Eigen::Vector3d vel = trajectory->getVelocity(t);
            std::cout << "t=" << t << "s: pos=(" << pos.x() << ", " << pos.y() 
                      << "), vel=(" << vel.x() << ", " << vel.y() << ")" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "FAILED: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}