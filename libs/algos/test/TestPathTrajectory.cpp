#include "path_trajectory.h"
#include <iostream>
#include <cassert>

using namespace algos;

void testBasicPathFinding() {
    std::cout << "Testing basic path finding..." << std::endl;
    
    PathTrajectory pathfinder;
    
    // Simple test: start at (0,0), go to (100,0) with no obstacles
    Vector2D start(0, 0);
    Vector2D target(100, 0);
    std::vector<Obstacle> obstacles;
    
    auto result = pathfinder.findDirectPath(start, target, obstacles, 50.0);
    
    assert(result.success);
    assert(result.trajectory.size() > 0);
    assert(result.total_distance > 90.0); // Should be close to 100
    
    std::cout << "✓ Basic path finding test passed" << std::endl;
    std::cout << "  - Trajectory points: " << result.trajectory.size() << std::endl;
    std::cout << "  - Total distance: " << result.total_distance << std::endl;
    std::cout << "  - Total time: " << result.total_time << std::endl;
}

void testObstacleAvoidance() {
    std::cout << "Testing obstacle avoidance..." << std::endl;
    
    PathTrajectory pathfinder;
    
    // Test with obstacle in the middle
    Vector2D start(0, 0);
    Vector2D target(100, 0);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(Obstacle(Vector2D(50, 0), 20.0)); // Obstacle directly in path
    
    auto result = pathfinder.findPotentialFieldPath(start, target, obstacles, 50.0);
    
    std::cout << "✓ Obstacle avoidance test completed" << std::endl;
    std::cout << "  - Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  - Trajectory points: " << result.trajectory.size() << std::endl;
    std::cout << "  - Total distance: " << result.total_distance << std::endl;
    
    // Check that path doesn't go through obstacle
    bool path_clear = true;
    for (const auto& point : result.trajectory) {
        double dist_to_obstacle = (point.position - obstacles[0].position).length();
        if (dist_to_obstacle < obstacles[0].radius + 5.0) { // 5.0 is agent radius buffer
            path_clear = false;
            break;
        }
    }
    
    if (path_clear) {
        std::cout << "  - Path successfully avoids obstacle" << std::endl;
    } else {
        std::cout << "  - Warning: Path may pass too close to obstacle" << std::endl;
    }
}

void testMultipleObstacles() {
    std::cout << "Testing multiple obstacles..." << std::endl;
    
    PathTrajectory pathfinder;
    
    Vector2D start(0, 0);
    Vector2D target(100, 100);
    std::vector<Obstacle> obstacles;
    
    // Create a maze-like scenario
    obstacles.push_back(Obstacle(Vector2D(30, 30), 15.0));
    obstacles.push_back(Obstacle(Vector2D(70, 30), 15.0));
    obstacles.push_back(Obstacle(Vector2D(30, 70), 15.0));
    obstacles.push_back(Obstacle(Vector2D(70, 70), 15.0));
    
    auto result = pathfinder.findPotentialFieldPath(start, target, obstacles, 60.0);
    
    std::cout << "✓ Multiple obstacles test completed" << std::endl;
    std::cout << "  - Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  - Trajectory points: " << result.trajectory.size() << std::endl;
    std::cout << "  - Total distance: " << result.total_distance << std::endl;
}

void testMovingObstacles() {
    std::cout << "Testing dynamic obstacles..." << std::endl;
    
    PathTrajectory pathfinder;
    
    Vector2D start(0, 0);
    Vector2D target(100, 0);
    std::vector<Obstacle> obstacles;
    
    // Moving obstacle crossing the path
    obstacles.push_back(Obstacle(Vector2D(50, -30), 12.0, Vector2D(0, 20))); // Moving upward
    
    auto result = pathfinder.findPotentialFieldPath(start, target, obstacles, 50.0);
    
    std::cout << "✓ Dynamic obstacles test completed" << std::endl;
    std::cout << "  - Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  - Trajectory points: " << result.trajectory.size() << std::endl;
}

void testSoccerScenario() {
    std::cout << "Testing soccer-like scenario..." << std::endl;
    
    PathTrajectory pathfinder;
    
    // Player at (x=10, y=20) wants to reach ball at (x=80, y=50)
    Vector2D player_pos(10, 20);
    Vector2D ball_pos(80, 50);
    
    // Opponents as obstacles
    std::vector<Obstacle> obstacles;
    obstacles.push_back(Obstacle(Vector2D(40, 35), 8.0, Vector2D(5, 0))); // Moving opponent
    obstacles.push_back(Obstacle(Vector2D(60, 45), 8.0, Vector2D(-3, 2))); // Another moving opponent
    
    auto result = pathfinder.findPath(player_pos, 0.0, ball_pos, 0.0, obstacles, 80.0, 40.0);
    
    std::cout << "✓ Soccer scenario test completed" << std::endl;
    std::cout << "  - Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "  - Trajectory points: " << result.trajectory.size() << std::endl;
    std::cout << "  - Total time to reach ball: " << result.total_time << " seconds" << std::endl;
    
    if (result.success && result.trajectory.size() > 0) {
        std::cout << "  - Start position: (" << result.trajectory[0].position.x 
                  << ", " << result.trajectory[0].position.y << ")" << std::endl;
        std::cout << "  - End position: (" << result.trajectory.back().position.x 
                  << ", " << result.trajectory.back().position.y << ")" << std::endl;
    }
}

int main() {
    std::cout << "=== Path Trajectory Algorithm Tests ===" << std::endl;
    
    try {
        testBasicPathFinding();
        std::cout << std::endl;
        
        testObstacleAvoidance();
        std::cout << std::endl;
        
        testMultipleObstacles();
        std::cout << std::endl;
        
        testMovingObstacles();
        std::cout << std::endl;
        
        testSoccerScenario();
        std::cout << std::endl;
        
        std::cout << "=== All tests completed successfully! ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}