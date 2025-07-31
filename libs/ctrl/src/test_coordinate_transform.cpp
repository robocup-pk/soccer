#include <iostream>
#include <Eigen/Dense>
#include "BSplineTrajectoryManager.h"
#include "RobotModel.h"
#include "Utils.h"

int main() {
    std::cout << "=== Coordinate Transformation Test ===" << std::endl;
    
    // Test 1: Simple forward motion in world frame
    std::cout << "\nTest 1: Forward motion (world frame)" << std::endl;
    Eigen::Vector3d vel_world(1.0, 0.0, 0.0);  // 1 m/s forward in world X
    
    // Robot at different orientations
    std::vector<double> robot_angles = {0.0, M_PI/4, M_PI/2, M_PI, -M_PI/2};
    
    for (double angle : robot_angles) {
        Eigen::Vector3d pose(0, 0, angle);
        
        // Convert to body frame
        Eigen::Vector3d vel_body = util::RotateAboutZ(vel_world, -angle);
        
        std::cout << "  Robot angle: " << angle * 180/M_PI << " deg" << std::endl;
        std::cout << "    World velocity: " << vel_world.transpose() << std::endl;
        std::cout << "    Body velocity:  " << vel_body.transpose() << std::endl;
        
        // Convert to wheel speeds
        kin::RobotModel robot_model;
        Eigen::Vector4d wheel_speeds = robot_model.RobotVelocityToWheelSpeedsRpm(vel_body);
        std::cout << "    Wheel RPMs:     " << wheel_speeds.transpose() << std::endl;
        
        // Verify reverse transformation
        Eigen::Vector3d vel_body_check = robot_model.WheelSpeedsRpmToRobotVelocity(wheel_speeds);
        std::cout << "    Body vel check: " << vel_body_check.transpose() << std::endl;
        
        Eigen::Vector3d vel_world_check = util::RotateAboutZ(vel_body_check, angle);
        std::cout << "    World vel check:" << vel_world_check.transpose() << std::endl;
        std::cout << std::endl;
    }
    
    // Test 2: Pure rotation
    std::cout << "\nTest 2: Pure rotation" << std::endl;
    Eigen::Vector3d vel_rot(0.0, 0.0, 1.0);  // 1 rad/s rotation
    
    kin::RobotModel robot_model;
    Eigen::Vector4d wheel_speeds_rot = robot_model.RobotVelocityToWheelSpeedsRpm(vel_rot);
    std::cout << "  Body velocity:  " << vel_rot.transpose() << std::endl;
    std::cout << "  Wheel RPMs:     " << wheel_speeds_rot.transpose() << std::endl;
    
    // Test 3: Check robot configuration
    std::cout << "\nTest 3: Robot wheel configuration" << std::endl;
    kin::RobotDescription desc = kin::GetRobotDescription();
    
    for (int i = 0; i < desc.num_wheels; i++) {
        std::cout << "  Wheel " << i << ":" << std::endl;
        std::cout << "    Position: (" << desc.wheel_positions_m[i].first 
                  << ", " << desc.wheel_positions_m[i].second << ") m" << std::endl;
        std::cout << "    Angle: " << desc.wheel_angles_rad[i] * 180/M_PI << " deg" << std::endl;
    }
    
    return 0;
}