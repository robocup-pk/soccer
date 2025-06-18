#include "path_trajectory.h"
#include <iostream>
#include <cassert>

using namespace algos;

double calculateDistance(const Vector2D& p1, const Vector2D& p2);

void testRealRobotCollisionScenario() {
    std::cout << "=== Testing Real Robot Collision Scenario ===" << std::endl;
    
    PathTrajectory pathfinder;
    
    // Simulate real robot positions from your demo
    Vector2D robot0_pos(0, 130);      // robot0 start position
    Vector2D robot1_pos(0, -220);     // robot1 position (350 units away)
    Vector2D ball_pos(100, 50);       // ball position
    
    std::vector<Obstacle> obstacles;
    
    // Add robot1 as obstacle with same radius as IntelligentMovement
    obstacles.push_back(Obstacle(robot1_pos, 25.0, Vector2D(0, 0)));
    
    std::cout << "Robot0 (controlled): (" << robot0_pos.x << ", " << robot0_pos.y << ")" << std::endl;
    std::cout << "Robot1 (obstacle): (" << robot1_pos.x << ", " << robot1_pos.y << ") radius: 25.0" << std::endl;
    std::cout << "Ball (target): (" << ball_pos.x << ", " << ball_pos.y << ")" << std::endl;
    std::cout << "Distance between robots: " << calculateDistance(robot0_pos, robot1_pos) << std::endl;
    
    auto result = pathfinder.findRRTPath(robot0_pos, ball_pos, obstacles, 80.0, 1000);
    
    std::cout << "\nRRT Result:" << std::endl;
    std::cout << "  Success: " << (result.success ? "YES" : "NO") << std::endl;
    std::cout << "  Path Points: " << result.trajectory.size() << std::endl;
    std::cout << "  Total Distance: " << result.total_distance << std::endl;
    
    if (result.success && !result.trajectory.empty()) {
        std::cout << "\nPath verification:" << std::endl;
        for (size_t i = 0; i < result.trajectory.size(); ++i) {
            const auto& point = result.trajectory[i];
            double dist_to_robot1 = calculateDistance(point.position, robot1_pos);
            
            std::cout << "  Point " << i << ": (" << point.position.x << ", " << point.position.y 
                      << ") dist to robot1: " << dist_to_robot1;
            
            if (dist_to_robot1 < 25.0 + 8.0) { // obstacle radius + agent radius
                std::cout << " [TOO CLOSE!]";
            }
            std::cout << std::endl;
        }
        
        // Check final position
        double final_dist = calculateDistance(result.trajectory.back().position, ball_pos);
        std::cout << "\nFinal distance to ball: " << final_dist << std::endl;
        
        assert(result.success);
    }
}

void testSimpleObstacleInPath() {
    std::cout << "\n=== Testing Simple Obstacle in Direct Path ===" << std::endl;
    
    PathTrajectory pathfinder;
    
    Vector2D start(0, 0);
    Vector2D target(100, 0);
    std::vector<Obstacle> obstacles;
    
    // Place obstacle directly in path
    obstacles.push_back(Obstacle(Vector2D(50, 0), 20.0));
    
    std::cout << "Start: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Target: (" << target.x << ", " << target.y << ")" << std::endl;
    std::cout << "Obstacle: (50, 0) radius: 20.0" << std::endl;
    
    auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1000);
    
    std::cout << "\nResult: " << (result.success ? "SUCCESS" : "FAILED") << std::endl;
    
    if (result.success) {
        std::cout << "Path points: " << result.trajectory.size() << std::endl;
        std::cout << "Distance: " << result.total_distance << std::endl;
        
        // Verify no collisions
        bool collision_free = true;
        for (const auto& point : result.trajectory) {
            double dist = calculateDistance(point.position, obstacles[0].position);
            if (dist < obstacles[0].radius + 8.0) {
                collision_free = false;
                std::cout << "COLLISION at (" << point.position.x << ", " << point.position.y 
                          << ") distance: " << dist << std::endl;
            }
        }
        
        if (collision_free) {
            std::cout << "✓ Path is collision-free!" << std::endl;
        } else {
            std::cout << "✗ Path has collisions!" << std::endl;
        }
    }
}

void testCurrentDemoConfiguration() {
    std::cout << "\n=== Testing Current Demo Configuration ===" << std::endl;
    
    PathTrajectory pathfinder;
    
    // Current demo setup from GLWindow.cpp
    Vector2D robot0_pos(0, 130);     // robot0 starts at (0, 130)  
    Vector2D robot1_pos(0, -220);    // robot1 starts at (0, -220) = 130 + (-350)
    Vector2D ball_pos(-52.5, 0);     // ball initial position
    
    std::vector<Obstacle> obstacles;
    obstacles.push_back(Obstacle(robot1_pos, 25.0)); // robot1 as obstacle
    
    std::cout << "Demo configuration:" << std::endl;
    std::cout << "  robot0: (" << robot0_pos.x << ", " << robot0_pos.y << ")" << std::endl;
    std::cout << "  robot1: (" << robot1_pos.x << ", " << robot1_pos.y << ")" << std::endl;
    std::cout << "  ball: (" << ball_pos.x << ", " << ball_pos.y << ")" << std::endl;
    std::cout << "  Distance between robots: " << calculateDistance(robot0_pos, robot1_pos) << " units" << std::endl;
    
    auto result = pathfinder.findRRTPath(robot0_pos, ball_pos, obstacles, 80.0, 1000);
    
    std::cout << "\nPath planning result: " << (result.success ? "SUCCESS" : "FAILED") << std::endl;
    
    if (result.success) {
        std::cout << "  Points: " << result.trajectory.size() << std::endl;
        std::cout << "  Distance: " << result.total_distance << std::endl;
        std::cout << "  Start: (" << result.trajectory[0].position.x << ", " << result.trajectory[0].position.y << ")" << std::endl;
        std::cout << "  End: (" << result.trajectory.back().position.x << ", " << result.trajectory.back().position.y << ")" << std::endl;
    }
}

double calculateDistance(const Vector2D& p1, const Vector2D& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

int main() {
    srand(42); // Consistent results
    
    testSimpleObstacleInPath();
    testRealRobotCollisionScenario();
    testCurrentDemoConfiguration();
    
    std::cout << "\n=== Debug Tests Completed ===" << std::endl;
    return 0;
}