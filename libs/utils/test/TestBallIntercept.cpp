#include <gtest/gtest.h>
#include <cmath>
#include "ball_intercept.h"

using namespace algos;

class OptimalPathFindingTest : public ::testing::Test {
protected:
    void SetUp() override {
        player = SimplePlayer(0.0, 0.0, 0.0);
        target = Vector2D(10.0, 10.0);
        interceptor = std::make_unique<BallInterceptor>();
    }

    SimplePlayer player;
    Vector2D target;
    std::unique_ptr<BallInterceptor> interceptor;
};

TEST_F(OptimalPathFindingTest, DirectPath_NoObstacles) {
    std::vector<SimplePlayer> opponents;
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, target, opponents);
    
    EXPECT_DOUBLE_EQ(optimal_path.x, target.x);
    EXPECT_DOUBLE_EQ(optimal_path.y, target.y);
    
    std::cout << "Direct path (no obstacles): " 
              << "Target(" << target.x << ", " << target.y << ") -> "
              << "Optimal(" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
}

TEST_F(OptimalPathFindingTest, PathClear_DistantObstacles) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(20.0, 20.0, 0.0),  // Far from path
        SimplePlayer(-10.0, -5.0, 0.0)  // Behind player
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, target, opponents);
    
    EXPECT_DOUBLE_EQ(optimal_path.x, target.x);
    EXPECT_DOUBLE_EQ(optimal_path.y, target.y);
    
    std::cout << "Path with distant obstacles: " 
              << "Target(" << target.x << ", " << target.y << ") -> "
              << "Optimal(" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
}

TEST_F(OptimalPathFindingTest, ObstacleOnDirectPath_SingleOpponent) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(5.0, 5.0, 0.0)  // Directly on the path
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, target, opponents);
    
    EXPECT_NE(optimal_path.x, target.x);
    EXPECT_NE(optimal_path.y, target.y);
    
    double distance_from_obstacle = optimal_path.distance(opponents[0].position());
    EXPECT_GT(distance_from_obstacle, 0.5);  // Should avoid collision radius
    
    std::cout << "Path with obstacle on direct route:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Target: (" << target.x << ", " << target.y << ")" << std::endl;
    std::cout << "  Obstacle: (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    std::cout << "  Optimal: (" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
    std::cout << "  Distance from obstacle: " << distance_from_obstacle << std::endl;
}

TEST_F(OptimalPathFindingTest, MultipleObstacles_ClusteredTogether) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(4.0, 4.0, 0.0),
        SimplePlayer(5.0, 5.0, 0.0),
        SimplePlayer(6.0, 6.0, 0.0)
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, target, opponents);
    
    EXPECT_NE(optimal_path.x, target.x);
    EXPECT_NE(optimal_path.y, target.y);
    
    for (const auto& opponent : opponents) {
        double distance_from_opponent = optimal_path.distance(opponent.position());
        EXPECT_GT(distance_from_opponent, 0.5);
    }
    
    std::cout << "Path with multiple clustered obstacles:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Target: (" << target.x << ", " << target.y << ")" << std::endl;
    std::cout << "  Optimal: (" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
    for (size_t i = 0; i < opponents.size(); ++i) {
        std::cout << "  Obstacle " << i+1 << ": (" << opponents[i].x << ", " << opponents[i].y 
                  << ") - Distance: " << optimal_path.distance(opponents[i].position()) << std::endl;
    }
}

TEST_F(OptimalPathFindingTest, ObstacleVeryCloseToPlayer) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(1.0, 1.0, 0.0)  // Very close to player
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, target, opponents);
    
    double distance_from_obstacle = optimal_path.distance(opponents[0].position());
    EXPECT_GT(distance_from_obstacle, 1.5);  // Should create significant avoidance
    
    std::cout << "Path with very close obstacle:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Target: (" << target.x << ", " << target.y << ")" << std::endl;
    std::cout << "  Close Obstacle: (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    std::cout << "  Optimal: (" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
    std::cout << "  Avoidance distance: " << distance_from_obstacle << std::endl;
}

TEST_F(OptimalPathFindingTest, ObstacleAtTarget) {
    Vector2D close_target(5.0, 5.0);
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(5.0, 5.0, 0.0)  // Exactly at target
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, close_target, opponents);
    
    EXPECT_NE(optimal_path.x, close_target.x);
    EXPECT_NE(optimal_path.y, close_target.y);
    
    double distance_from_obstacle = optimal_path.distance(opponents[0].position());
    EXPECT_GT(distance_from_obstacle, 0.5);
    
    std::cout << "Path with obstacle at target:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Target: (" << close_target.x << ", " << close_target.y << ")" << std::endl;
    std::cout << "  Obstacle at target: (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    std::cout << "  Optimal: (" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
}

TEST_F(OptimalPathFindingTest, CalculateAvoidanceForce_SingleOpponent) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(2.0, 0.0, 0.0)  // To the right of player
    };
    Vector2D desired_direction(1.0, 0.0);  // Moving toward opponent
    
    Vector2D avoidance_force = interceptor->calculateAvoidanceForce(player, opponents, desired_direction);
    
    EXPECT_NEAR(avoidance_force.length(), 1.0, 1e-10);  // Should be normalized
    EXPECT_LT(avoidance_force.x, desired_direction.x);  // Should be deflected away from opponent
    
    std::cout << "Avoidance force calculation:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Opponent: (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    std::cout << "  Desired direction: (" << desired_direction.x << ", " << desired_direction.y << ")" << std::endl;
    std::cout << "  Avoidance force: (" << avoidance_force.x << ", " << avoidance_force.y << ")" << std::endl;
}

TEST_F(OptimalPathFindingTest, CalculateAvoidanceForce_MultipleOpponents) {
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(1.0, 1.0, 0.0),   // Upper right
        SimplePlayer(1.0, -1.0, 0.0)  // Lower right
    };
    Vector2D desired_direction(1.0, 0.0);  // Moving right between opponents
    
    Vector2D avoidance_force = interceptor->calculateAvoidanceForce(player, opponents, desired_direction);
    
    EXPECT_NEAR(avoidance_force.length(), 1.0, 1e-10);  // Should be normalized
    EXPECT_LT(avoidance_force.x, desired_direction.x);  // Should be deflected away
    EXPECT_NEAR(avoidance_force.y, 0.0, 0.1);  // Y forces should cancel out
    
    std::cout << "Multi-opponent avoidance force:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Opponent 1: (" << opponents[0].x << ", " << opponents[0].y << ")" << std::endl;
    std::cout << "  Opponent 2: (" << opponents[1].x << ", " << opponents[1].y << ")" << std::endl;
    std::cout << "  Desired direction: (" << desired_direction.x << ", " << desired_direction.y << ")" << std::endl;
    std::cout << "  Avoidance force: (" << avoidance_force.x << ", " << avoidance_force.y << ")" << std::endl;
}

TEST_F(OptimalPathFindingTest, PathClearance_ChecksIntermediatePoints) {
    // Create obstacle that would only be detected by intermediate point checking
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(5.0, 5.0, 0.0)  // Exactly in the middle of path
    };
    
    bool is_clear = interceptor->isPathClear(player.position(), target, opponents, 0);
    EXPECT_FALSE(is_clear);
    
    // Test with clear path
    std::vector<SimplePlayer> no_opponents;
    bool is_clear_no_obstacles = interceptor->isPathClear(player.position(), target, no_opponents, 0);
    EXPECT_TRUE(is_clear_no_obstacles);
    
    std::cout << "Path clearance check:" << std::endl;
    std::cout << "  Start: (" << player.position().x << ", " << player.position().y << ")" << std::endl;
    std::cout << "  End: (" << target.x << ", " << target.y << ")" << std::endl;
    std::cout << "  Obstacle in middle: " << (is_clear ? "No" : "Yes") << std::endl;
    std::cout << "  Clear path (no obstacles): " << (is_clear_no_obstacles ? "Yes" : "No") << std::endl;
}

TEST_F(OptimalPathFindingTest, LongDistance_PathOptimization) {
    Vector2D far_target(50.0, 30.0);
    std::vector<SimplePlayer> opponents = {
        SimplePlayer(10.0, 6.0, 0.0),
        SimplePlayer(20.0, 12.0, 0.0),
        SimplePlayer(30.0, 18.0, 0.0)
    };
    
    Vector2D optimal_path = interceptor->findOptimalPath(player, far_target, opponents);
    
    // Should create intermediate target
    double direct_distance = player.position().distance(far_target);
    double optimal_distance = player.position().distance(optimal_path);
    
    EXPECT_LT(optimal_distance, direct_distance);  // Should be closer intermediate target
    
    std::cout << "Long distance path optimization:" << std::endl;
    std::cout << "  Player: (" << player.x << ", " << player.y << ")" << std::endl;
    std::cout << "  Far target: (" << far_target.x << ", " << far_target.y << ")" << std::endl;
    std::cout << "  Optimal intermediate: (" << optimal_path.x << ", " << optimal_path.y << ")" << std::endl;
    std::cout << "  Direct distance: " << direct_distance << std::endl;
    std::cout << "  Intermediate distance: " << optimal_distance << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "Running Optimal Path Finding Tests..." << std::endl;
    std::cout << "====================================" << std::endl;
    
    int result = RUN_ALL_TESTS();
    
    if (result == 0) {
        std::cout << std::endl << "All optimal path finding tests passed! ✓" << std::endl;
        std::cout << "Path finding algorithm is working correctly with obstacle avoidance." << std::endl;
    } else {
        std::cout << std::endl << "Some tests failed! ✗" << std::endl;
        std::cout << "Please check the test output above for details." << std::endl;
    }
    
    return result;
}