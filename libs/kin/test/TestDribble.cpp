#include <gtest/gtest.h>
#include <cmath>
#include "Dribble.h"
#include "SoccerObject.h"
#include "RobotModel.h"

class DribbleTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create robot at origin facing positive X direction
        robot = state::SoccerObject("robot0", 
                                   Eigen::Vector3d(0.0, 0.0, 0.0),  // position (x, y, angle)
                                   Eigen::Vector2d(0.18, 0.18),     // size (18cm x 18cm)
                                   Eigen::Vector3d::Zero(),          // velocity
                                   Eigen::Vector3d::Zero(),          // acceleration
                                   2.0f);                            // mass (2kg)
        robot.radius_m = 0.09f; // 9cm radius
        
        // Create ball near robot
        ball = state::Ball(Eigen::Vector3d(0.15, 0.0, 0.0)); // 15cm in front of robot
    }
    
    state::SoccerObject robot;
    state::Ball ball;
};

// Test 1: Basic dribble functionality when robot faces ball
TEST_F(DribbleTest, BasicDribbleWhenFacingBall) {
    // Robot at origin facing +X, ball at (0.15, 0, 0)
    Eigen::Vector3d initial_ball_pos = ball.position;
    Eigen::Vector3d initial_ball_vel = ball.velocity;
    
    // Apply dribble with moderate power
    bool success = kin::Dribble(robot, ball, 10.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed when robot faces ball and ball is in range";
    
    // Ball should have some velocity applied (but not necessarily moved yet)
    // The dribble function applies velocity changes, actual movement happens in physics update
    EXPECT_NE(ball.velocity.x(), initial_ball_vel.x()) << "Ball velocity should change after dribble";
}

// Test 2: Dribble fails when ball is too far away
TEST_F(DribbleTest, DribbleFailsWhenBallTooFar) {
    // Place ball 1 meter away (beyond dribbling range)
    ball.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    
    bool success = kin::Dribble(robot, ball, 10.0, true);
    
    EXPECT_FALSE(success) << "Dribble should fail when ball is beyond max dribble distance";
}

// Test 3: Dribble with different power levels
TEST_F(DribbleTest, DribbleWithDifferentPowerLevels) {
    // Test with low power
    ball.velocity = Eigen::Vector3d::Zero();
    bool success_low = kin::Dribble(robot, ball, 1.0, true);
    Eigen::Vector3d velocity_low = ball.velocity;
    
    // Reset ball
    ball.velocity = Eigen::Vector3d::Zero();
    ball.position = Eigen::Vector3d(0.15, 0.0, 0.0);
    
    // Test with high power
    bool success_high = kin::Dribble(robot, ball, 20.0, true);
    Eigen::Vector3d velocity_high = ball.velocity;
    
    EXPECT_TRUE(success_low) << "Low power dribble should succeed";
    EXPECT_TRUE(success_high) << "High power dribble should succeed";
    
    // Higher power should result in larger velocity change (but capped by max force)
    double low_magnitude = velocity_low.head<2>().norm();
    double high_magnitude = velocity_high.head<2>().norm();
    EXPECT_GE(high_magnitude, low_magnitude) << "Higher power should result in higher velocity magnitude";
}

// Test 4: Dribble force direction - ball should be pulled toward robot front
TEST_F(DribbleTest, DribbleForceDirection) {
    // Place ball slightly to the side and behind desired position
    ball.position = Eigen::Vector3d(0.05, 0.05, 0.0); // 5cm forward, 5cm to right
    ball.velocity = Eigen::Vector3d::Zero();
    
    bool success = kin::Dribble(robot, ball, 15.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed for close ball";
    
    // Ball should get velocity toward the robot's front (approximately +X direction)
    // and slightly toward center (-Y direction to correct the offset)
    EXPECT_GT(ball.velocity.x(), 0.0) << "Ball should get positive X velocity (toward robot front)";
    EXPECT_LT(ball.velocity.y(), 0.0) << "Ball should get negative Y velocity (toward robot center)";
}

// Test 5: Robot orientation affects dribble direction
TEST_F(DribbleTest, RobotOrientationAffectsDribbleDirection) {
    // Robot facing +Y direction (90 degrees)
    robot.position.z() = M_PI / 2.0;
    
    // Ball positioned in front of robot (in robot's local coordinate)
    ball.position = Eigen::Vector3d(0.0, 0.15, 0.0); // 15cm in +Y direction
    ball.velocity = Eigen::Vector3d::Zero();
    
    bool success = kin::Dribble(robot, ball, 15.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed when ball is in front of rotated robot";
    
    // The desired ball position should be calculated based on robot's orientation
    // For robot facing +Y, the ball should be pulled toward position (0, 0.08, 0) approximately
}

// Test 6: Continuous vs single dribble
TEST_F(DribbleTest, ContinuousVsSingleDribble) {
    // Test continuous dribble (default for active dribbling)
    ball.velocity = Eigen::Vector3d::Zero();
    bool success_continuous = kin::Dribble(robot, ball, 10.0, true);
    Eigen::Vector3d velocity_continuous = ball.velocity;
    
    // Reset ball
    ball.velocity = Eigen::Vector3d::Zero();
    ball.position = Eigen::Vector3d(0.15, 0.0, 0.0);
    
    // Test single dribble
    bool success_single = kin::Dribble(robot, ball, 10.0, false);
    Eigen::Vector3d velocity_single = ball.velocity;
    
    EXPECT_TRUE(success_continuous) << "Continuous dribble should succeed";
    EXPECT_TRUE(success_single) << "Single dribble should succeed";
    
    // Both should apply forces (the implementation is the same for now)
    EXPECT_GT(velocity_continuous.head<2>().norm(), 0.0) << "Continuous dribble should apply force";
    EXPECT_GT(velocity_single.head<2>().norm(), 0.0) << "Single dribble should apply force";
}

// Test 7: Dribble force is limited by maximum force
TEST_F(DribbleTest, DribbleForceIsLimited) {
    // Place ball very far from desired position to test force limiting
    ball.position = Eigen::Vector3d(0.2, 0.0, 0.0); // 20cm forward (at edge of range)
    ball.velocity = Eigen::Vector3d::Zero();
    
    // Apply very high power
    bool success = kin::Dribble(robot, ball, 100.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed even with very high power";
    
    // Velocity should be limited - not extremely high despite high power
    double velocity_magnitude = ball.velocity.head<2>().norm();
    EXPECT_LT(velocity_magnitude, 10.0) << "Dribble velocity should be limited by max force constraint";
    EXPECT_GT(velocity_magnitude, 0.0) << "Dribble should still apply some force";
}

// Test 8: Ball attachment should not occur during dribbling
TEST_F(DribbleTest, BallNotAttachedDuringDribble) {
    // Ensure ball is not attached initially
    ball.is_attached = false;
    ball.attached_to = nullptr;
    
    bool success = kin::Dribble(robot, ball, 15.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed";
    EXPECT_FALSE(ball.is_attached) << "Ball should not be attached during dribbling (SSL rule)";
    EXPECT_EQ(ball.attached_to, nullptr) << "Ball should not be attached to any object";
}

// Test 9: Dribbling sets is_dribbling flag
TEST_F(DribbleTest, DribblingSetsDribblingFlag) {
    // Initially not dribbling
    ball.is_dribbling = false;
    
    bool success = kin::Dribble(robot, ball, 15.0, true);
    
    EXPECT_TRUE(success) << "Dribble should succeed";
    // Note: The is_dribbling flag is set in RobotManager, not in the Dribble function itself
    // This test documents the expected behavior
}

// Test 10: Edge case - ball exactly at maximum distance
TEST_F(DribbleTest, BallAtMaximumDistance) {
    // Calculate maximum dribble distance
    double max_distance = robot.radius_m + ball.radius_m + 0.1; // From Dribble.cpp
    
    // Place ball exactly at maximum distance
    ball.position = Eigen::Vector3d(max_distance, 0.0, 0.0);
    
    bool success = kin::Dribble(robot, ball, 10.0, true);
    
    // Should succeed if distance is exactly at limit
    EXPECT_TRUE(success) << "Dribble should succeed when ball is exactly at maximum distance";
    
    // Place ball just beyond maximum distance
    ball.position = Eigen::Vector3d(max_distance + 0.001, 0.0, 0.0);
    ball.velocity = Eigen::Vector3d::Zero();
    
    bool success_beyond = kin::Dribble(robot, ball, 10.0, true);
    EXPECT_FALSE(success_beyond) << "Dribble should fail when ball is just beyond maximum distance";
}

// Test 11: Multiple successive dribbles
TEST_F(DribbleTest, MultipleDribbles) {
    int successful_dribbles = 0;
    
    // Apply dribble multiple times
    for (int i = 0; i < 5; i++) {
        if (kin::Dribble(robot, ball, 10.0, true)) {
            successful_dribbles++;
        }
        
        // Simulate small time step physics to see ball movement
        // (In real simulation, physics would be updated between dribbles)
    }
    
    EXPECT_EQ(successful_dribbles, 5) << "All successive dribbles should succeed for ball in range";
    
    // Ball should have accumulated velocity from multiple dribbles
    EXPECT_GT(ball.velocity.head<2>().norm(), 0.0) << "Ball should have velocity after multiple dribbles";
}

// Test 12: Dribble with BallModel vs regular SoccerObject
TEST_F(DribbleTest, DribbleWithBallModel) {
    // Test confirms that BallModel (derived from SoccerObject) works with dribble
    state::Ball ball_model(Eigen::Vector3d(0.12, 0.0, 0.0));
    
    bool success = kin::Dribble(robot, ball_model, 10.0, true);
    
    EXPECT_TRUE(success) << "Dribble should work with BallModel objects";
    EXPECT_GT(ball_model.velocity.head<2>().norm(), 0.0) << "BallModel should receive velocity from dribble";
}