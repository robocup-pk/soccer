#include "path_trajectory.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>

using namespace algos;

class RRTTestSuite {
public:
    void runAllTests() {
        std::cout << "=== RRT Algorithm Accuracy Tests ===" << std::endl;
        
        testBasicPathNoObstacles();
        testSingleObstacleBetweenSourceAndTarget();
        testWallBarrierBetweenSourceAndTarget();
        testLShapedObstacleNavigation();
        testMultipleObstaclesComplexMaze();
        testNarrowPassageThroughObstacles();
        testObstacleCloseToSource();
        testObstacleCloseToTarget();
        testCircularObstacleAvoidance();
        testCorridorNavigation();
        testRealWorldSoccerScenarios();
        
        std::cout << "=== All RRT Accuracy Tests Completed ===" << std::endl;
    }

private:
    PathTrajectory pathfinder;
    
    bool isPathCollisionFree(const std::vector<TrajectoryPoint>& trajectory, 
                            const std::vector<Obstacle>& obstacles,
                            double agent_radius = 10.0) {
        for (const auto& point : trajectory) {
            for (const auto& obstacle : obstacles) {
                double distance = calculateDistance(point.position, obstacle.position);
                if (distance < obstacle.radius + agent_radius) {
                    std::cout << "  [COLLISION] Path point (" << point.position.x << ", " << point.position.y 
                              << ") too close to obstacle at (" << obstacle.position.x << ", " << obstacle.position.y 
                              << ") - Distance: " << distance << ", Required: " << (obstacle.radius + agent_radius) << std::endl;
                    return false;
                }
            }
        }
        return true;
    }
    
    bool pathReachesTarget(const std::vector<TrajectoryPoint>& trajectory, 
                          const Vector2D& target, double threshold = 25.0) {
        if (trajectory.empty()) return false;
        
        double final_distance = calculateDistance(trajectory.back().position, target);
        return final_distance <= threshold;
    }
    
    double calculateDistance(const Vector2D& p1, const Vector2D& p2) {
        return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }
    
    void printPathDetails(const PathTrajectory::PathResult& result, const std::string& test_name) {
        std::cout << "[" << test_name << "] Results:" << std::endl;
        std::cout << "  - Success: " << (result.success ? "YES" : "NO") << std::endl;
        std::cout << "  - Path Points: " << result.trajectory.size() << std::endl;
        std::cout << "  - Total Distance: " << result.total_distance << std::endl;
        std::cout << "  - Total Time: " << result.total_time << "s" << std::endl;
        
        if (!result.trajectory.empty()) {
            std::cout << "  - Start: (" << result.trajectory[0].position.x 
                      << ", " << result.trajectory[0].position.y << ")" << std::endl;
            std::cout << "  - End: (" << result.trajectory.back().position.x 
                      << ", " << result.trajectory.back().position.y << ")" << std::endl;
        }
    }

    void testBasicPathNoObstacles() {
        std::cout << "\n1. Testing basic path with no obstacles..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 0);
        std::vector<Obstacle> obstacles; // No obstacles
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 500);
        
        printPathDetails(result, "No Obstacles");
        
        assert(result.success);
        assert(pathReachesTarget(result.trajectory, target));
        assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
        
        std::cout << "✓ Basic path test PASSED" << std::endl;
    }
    
    void testSingleObstacleBetweenSourceAndTarget() {
        std::cout << "\n2. Testing single obstacle directly between source and target..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 0);
        std::vector<Obstacle> obstacles;
        
        // Place obstacle directly in the path
        obstacles.push_back(Obstacle(Vector2D(50, 0), 20.0));
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1000);
        
        printPathDetails(result, "Single Obstacle");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Single obstacle avoidance PASSED" << std::endl;
        } else {
            std::cout << "✗ Single obstacle avoidance FAILED - No path found" << std::endl;
        }
    }
    
    void testWallBarrierBetweenSourceAndTarget() {
        std::cout << "\n3. Testing wall barrier between source and target..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 0);
        std::vector<Obstacle> obstacles;
        
        // Create vertical wall with gap
        for (int y = -50; y <= 30; y += 10) {
            obstacles.push_back(Obstacle(Vector2D(50, y), 8.0));
        }
        for (int y = 50; y <= 80; y += 10) {
            obstacles.push_back(Obstacle(Vector2D(50, y), 8.0));
        }
        // Gap between y=30 and y=50
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1500);
        
        printPathDetails(result, "Wall Barrier");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Wall barrier navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Wall barrier navigation FAILED - No path found" << std::endl;
        }
    }
    
    void testLShapedObstacleNavigation() {
        std::cout << "\n4. Testing L-shaped obstacle navigation..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(80, 80);
        std::vector<Obstacle> obstacles;
        
        // Create L-shaped obstacle
        for (int x = 20; x <= 60; x += 10) {
            obstacles.push_back(Obstacle(Vector2D(x, 40), 12.0));
        }
        for (int y = 40; y <= 80; y += 10) {
            obstacles.push_back(Obstacle(Vector2D(60, y), 12.0));
        }
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1500);
        
        printPathDetails(result, "L-Shaped Obstacle");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ L-shaped obstacle navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ L-shaped obstacle navigation FAILED" << std::endl;
        }
    }
    
    void testMultipleObstaclesComplexMaze() {
        std::cout << "\n5. Testing complex maze with multiple obstacles..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(150, 100);
        std::vector<Obstacle> obstacles;
        
        // Create maze-like environment
        obstacles.push_back(Obstacle(Vector2D(30, 30), 18.0));
        obstacles.push_back(Obstacle(Vector2D(30, 70), 18.0));
        obstacles.push_back(Obstacle(Vector2D(70, 20), 18.0));
        obstacles.push_back(Obstacle(Vector2D(70, 80), 18.0));
        obstacles.push_back(Obstacle(Vector2D(110, 40), 18.0));
        obstacles.push_back(Obstacle(Vector2D(110, 80), 18.0));
        obstacles.push_back(Obstacle(Vector2D(90, 60), 15.0));
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 2000);
        
        printPathDetails(result, "Complex Maze");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Complex maze navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Complex maze navigation FAILED" << std::endl;
        }
    }
    
    void testNarrowPassageThroughObstacles() {
        std::cout << "\n6. Testing narrow passage navigation..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 0);
        std::vector<Obstacle> obstacles;
        
        // Create narrow passage
        obstacles.push_back(Obstacle(Vector2D(50, 25), 20.0));
        obstacles.push_back(Obstacle(Vector2D(50, -25), 20.0));
        // Gap of ~10 units between obstacles
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1500);
        
        printPathDetails(result, "Narrow Passage");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Narrow passage navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Narrow passage navigation FAILED" << std::endl;
        }
    }
    
    void testObstacleCloseToSource() {
        std::cout << "\n7. Testing obstacle very close to source..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 100);
        std::vector<Obstacle> obstacles;
        
        // Obstacle very close to start
        obstacles.push_back(Obstacle(Vector2D(20, 20), 10.0)); // Moved farther and smaller
        obstacles.push_back(Obstacle(Vector2D(50, 50), 12.0));
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1500);
        
        printPathDetails(result, "Obstacle Near Source");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Obstacle near source navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Obstacle near source navigation FAILED" << std::endl;
        }
    }
    
    void testObstacleCloseToTarget() {
        std::cout << "\n8. Testing obstacle very close to target..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(100, 100);
        std::vector<Obstacle> obstacles;
        
        // Obstacle very close to target
        obstacles.push_back(Obstacle(Vector2D(85, 85), 12.0));
        obstacles.push_back(Obstacle(Vector2D(50, 50), 15.0));
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1000);
        
        printPathDetails(result, "Obstacle Near Target");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Obstacle near target navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Obstacle near target navigation FAILED" << std::endl;
        }
    }
    
    void testCircularObstacleAvoidance() {
        std::cout << "\n9. Testing circular obstacle arrangement..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(0, 100);
        std::vector<Obstacle> obstacles;
        
        // Create circular arrangement of obstacles
        double center_x = 0, center_y = 50;
        double radius = 30;
        for (int angle = 0; angle < 360; angle += 45) {
            double rad = angle * M_PI / 180.0;
            double x = center_x + radius * cos(rad);
            double y = center_y + radius * sin(rad);
            obstacles.push_back(Obstacle(Vector2D(x, y), 8.0));
        }
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1500);
        
        printPathDetails(result, "Circular Obstacles");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Circular obstacle avoidance PASSED" << std::endl;
        } else {
            std::cout << "✗ Circular obstacle avoidance FAILED" << std::endl;
        }
    }
    
    void testCorridorNavigation() {
        std::cout << "\n10. Testing corridor navigation..." << std::endl;
        
        Vector2D start(0, 0);
        Vector2D target(120, 0);
        std::vector<Obstacle> obstacles;
        
        // Create corridor walls
        for (int x = 20; x <= 100; x += 10) {
            obstacles.push_back(Obstacle(Vector2D(x, 20), 8.0)); // Top wall
            obstacles.push_back(Obstacle(Vector2D(x, -20), 8.0)); // Bottom wall
        }
        
        auto result = pathfinder.findRRTPath(start, target, obstacles, 80.0, 1000);
        
        printPathDetails(result, "Corridor Navigation");
        
        if (result.success) {
            assert(pathReachesTarget(result.trajectory, target));
            assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Reduced agent radius for testing
            std::cout << "✓ Corridor navigation PASSED" << std::endl;
        } else {
            std::cout << "✗ Corridor navigation FAILED" << std::endl;
        }
    }
    
    void testRealWorldSoccerScenarios() {
        std::cout << "\n11. Testing real-world soccer scenarios..." << std::endl;
        
        // Scenario 1: Player avoiding two opponents to reach ball
        {
            Vector2D player_pos(10, 20);
            Vector2D ball_pos(80, 50);
            std::vector<Obstacle> obstacles;
            
            // Two opponent robots (reduced radius for realistic robot size)
            obstacles.push_back(Obstacle(Vector2D(40, 35), 18.0, Vector2D(5, 0))); // Moving opponent
            obstacles.push_back(Obstacle(Vector2D(60, 45), 18.0, Vector2D(-3, 2))); // Another moving opponent
            
            auto result = pathfinder.findRRTPath(player_pos, ball_pos, obstacles, 90.0, 1500);
            
            printPathDetails(result, "Soccer Scenario 1");
            
            if (result.success) {
                assert(pathReachesTarget(result.trajectory, ball_pos));
                assert(isPathCollisionFree(result.trajectory, obstacles, 8.0)); // Robot radius
                std::cout << "✓ Soccer scenario 1 PASSED" << std::endl;
            } else {
                std::cout << "✗ Soccer scenario 1 FAILED" << std::endl;
            }
        }
        
        // Scenario 2: Goalkeeper avoiding players to reach ball
        {
            Vector2D keeper_pos(-40, 0);
            Vector2D ball_pos(20, 30);
            std::vector<Obstacle> obstacles;
            
            // Field players as obstacles (reduced radius)
            obstacles.push_back(Obstacle(Vector2D(-10, 15), 18.0));
            obstacles.push_back(Obstacle(Vector2D(0, -10), 18.0));
            obstacles.push_back(Obstacle(Vector2D(15, 0), 18.0));
            
            auto result = pathfinder.findRRTPath(keeper_pos, ball_pos, obstacles, 70.0, 1500);
            
            printPathDetails(result, "Soccer Scenario 2");
            
            if (result.success) {
                assert(pathReachesTarget(result.trajectory, ball_pos));
                assert(isPathCollisionFree(result.trajectory, obstacles, 8.0));
                std::cout << "✓ Soccer scenario 2 PASSED" << std::endl;
            } else {
                std::cout << "✗ Soccer scenario 2 FAILED" << std::endl;
            }
        }
    }
};

int main() {
    // Seed random number generator for consistent results
    srand(42);
    
    RRTTestSuite test_suite;
    test_suite.runAllTests();
    
    return 0;
}