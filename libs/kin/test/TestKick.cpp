#include <gtest/gtest.h>
#include <cmath>
#include "Kick.h"
#include "SoccerObject.h"
#include "BallModel.h"
#include "RobotModel.h"

class KickTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create robot with its center at (0,0,0) facing positive X direction
        robot = state::SoccerObject("robot0", 
                                   Eigen::Vector3d(-0.09, -0.09, 0.0),  // position (x, y, angle) so center is at (0,0)
                                   Eigen::Vector2d(0.18, 0.18),     // size (18cm x 18cm)
                                   Eigen::Vector3d::Zero(),          // velocity
                                   Eigen::Vector3d::Zero(),          // acceleration
                                   2.0f);                            // mass (2kg)
        robot.radius_m = 0.09f; // 9cm radius
        
        // Create ball in kicking position (close to robot front), so its center is at (0.12, 0, 0)
        ball = state::Ball(Eigen::Vector3d(0.12 - 0.0215, 0.0 - 0.0215, 0.0)); // Adjust for ball center
    }
    
    state::SoccerObject robot;
    state::Ball ball;
};

// Test 1: Basic kick functionality when robot faces ball
TEST_F(KickTest, BasicKickWhenFacingBall) {
    Eigen::Vector3d initial_velocity = ball.velocity;
    
    // Apply kick with default power
    bool success = kin::Kick(robot, ball);
    
    EXPECT_TRUE(success) << "Kick should succeed when robot faces ball and ball is in range";
    
    // Ball should have significant velocity in forward direction (+X)
    EXPECT_GT(ball.velocity.x(), initial_velocity.x()) << "Ball should gain forward velocity from kick";
    EXPECT_GT(ball.velocity.x(), 2.0) << "Ball velocity should be substantial (> 2 m/s)";
    
    // Ball should stay in 2D plane (z-velocity should be 0 for SSL)
    EXPECT_EQ(ball.velocity.z(), 0.0) << "Ball should have no vertical velocity (SSL 2D)";
}

// Test 2: Kick fails when ball is too far away
TEST_F(KickTest, KickFailsWhenBallTooFar) {
    // Place ball beyond maximum kick distance (25cm for SSL)
    // Ball center at (0.3, 0, 0)
    ball.position = Eigen::Vector3d(0.3 - ball.radius_m, -ball.radius_m, 0.0); 
    
    bool success = kin::Kick(robot, ball);
    
    EXPECT_FALSE(success) << "Kick should fail when ball is beyond max kick distance";
    
    // Ball velocity should remain unchanged
    EXPECT_EQ(ball.velocity.norm(), 0.0) << "Ball velocity should not change when kick fails";
}

// Test 3: Kick fails when robot is not facing ball
TEST_F(KickTest, KickFailsWhenNotFacingBall) {
    // Robot facing +X, but ball is to the side (+Y)
    // Ball center at (0, 0.12, 0)
    ball.position = Eigen::Vector3d(-ball.radius_m, 0.12 - ball.radius_m, 0.0); 
    
    bool success = kin::Kick(robot, ball, 3.0, false); // Don't force kick
    
    EXPECT_FALSE(success) << "Kick should fail when robot is not facing ball";
    
    // Ball velocity should remain unchanged
    EXPECT_EQ(ball.velocity.norm(), 0.0) << "Ball velocity should not change when kick fails";
}

// Test 4: Force kick bypasses orientation check
TEST_F(KickTest, ForceKickBypassesOrientationCheck) {
    // Robot facing +X, but ball is to the side (+Y)
    // Ball center at (0, 0.12, 0)
    ball.position = Eigen::Vector3d(-ball.radius_m, 0.12 - ball.radius_m, 0.0); 
    
    bool success = kin::Kick(robot, ball, 3.0, true); // Force kick = true
    
    EXPECT_TRUE(success) << "Force kick should succeed even when robot not facing ball";
    
    // Ball should gain velocity
    EXPECT_GT(ball.velocity.norm(), 0.0) << "Ball should gain velocity from force kick";
}

// Test 5: Different kick power levels
TEST_F(KickTest, DifferentKickPowerLevels) {
    // Test with low power
    state::Ball ball1(Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0));
    bool success_low = kin::Kick(robot, ball1, 1.0);
    double velocity_low = ball1.velocity.norm();
    
    // Test with medium power
    state::Ball ball2(Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0));
    bool success_medium = kin::Kick(robot, ball2, 3.0);
    double velocity_medium = ball2.velocity.norm();
    
    // Test with high power
    state::Ball ball3(Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0));
    bool success_high = kin::Kick(robot, ball3, 5.0);
    double velocity_high = ball3.velocity.norm();
    
    EXPECT_TRUE(success_low) << "Low power kick should succeed";
    EXPECT_TRUE(success_medium) << "Medium power kick should succeed";
    EXPECT_TRUE(success_high) << "High power kick should succeed";
    
    // Higher power should result in higher velocity
    EXPECT_LT(velocity_low, velocity_medium) << "Medium power should be higher than low power";
    EXPECT_LT(velocity_medium, velocity_high) << "High power should be higher than medium power";
    
    // Verify actual velocity values are reasonable
    EXPECT_NEAR(velocity_low, 1.0, 0.2) << "Low power kick should result in ~1 m/s";
    EXPECT_NEAR(velocity_medium, 3.0, 0.2) << "Medium power kick should result in ~3 m/s";
    EXPECT_NEAR(velocity_high, 5.0, 0.2) << "High power kick should result in ~5 m/s";
}

// Test 6: SSL maximum power constraint
TEST_F(KickTest, SSLMaximumPowerConstraint) {
    // Try to kick with power exceeding SSL maximum (6 m/s)
    bool success = kin::Kick(robot, ball, 10.0); // Request 10 m/s
    
    EXPECT_TRUE(success) << "Kick should succeed but be capped at max power";
    
    // Velocity should be capped at SSL maximum
    EXPECT_LE(ball.velocity.norm(), kin::ssl::MAX_KICK_POWER + 0.1) 
        << "Ball velocity should be capped at SSL maximum";
    EXPECT_GE(ball.velocity.norm(), kin::ssl::MAX_KICK_POWER - 0.1) 
        << "Ball velocity should reach SSL maximum";
}

// Test 7: Kick direction based on robot orientation
TEST_F(KickTest, KickDirectionBasedOnRobotOrientation) {
    // Test kick in different directions based on robot orientation
    
    // Robot facing +X (0 radians)
    robot.position.z() = 0.0;
    ball.position = Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0);
    state::Ball ball_x = ball;
    bool success_x = kin::Kick(robot, ball_x, 3.0);
    
    // Robot facing +Y (π/2 radians)
    robot.position.z() = M_PI / 2.0;
    ball.position = Eigen::Vector3d(-ball.radius_m, 0.12 - ball.radius_m, 0.0);
    state::Ball ball_y = ball;
    bool success_y = kin::Kick(robot, ball_y, 3.0);
    
    // Robot facing -X (π radians)
    robot.position.z() = M_PI;
    ball.position = Eigen::Vector3d(-0.12 - ball.radius_m, -ball.radius_m, 0.0);
    state::Ball ball_neg_x = ball;
    bool success_neg_x = kin::Kick(robot, ball_neg_x, 3.0);
    
    EXPECT_TRUE(success_x) << "Kick in +X direction should succeed";
    EXPECT_TRUE(success_y) << "Kick in +Y direction should succeed";
    EXPECT_TRUE(success_neg_x) << "Kick in -X direction should succeed";
    
    // Check kick directions
    EXPECT_GT(ball_x.velocity.x(), 2.5) << "Ball should move in +X direction";
    EXPECT_NEAR(ball_x.velocity.y(), 0.0, 0.1) << "Ball should not move in Y direction";
    
    EXPECT_GT(ball_y.velocity.y(), 2.5) << "Ball should move in +Y direction";
    EXPECT_NEAR(ball_y.velocity.x(), 0.0, 0.1) << "Ball should not move in X direction";
    
    EXPECT_LT(ball_neg_x.velocity.x(), -2.5) << "Ball should move in -X direction";
    EXPECT_NEAR(ball_neg_x.velocity.y(), 0.0, 0.1) << "Ball should not move in Y direction";
}

// Test 8: SSL default power values
TEST_F(KickTest, SSLDefaultPowerValues) {
    // Test with default kick power (no parameter)
    state::Ball ball_default = ball;
    bool success_default = kin::Kick(robot, ball_default);
    
    // Test with explicit default power
    state::Ball ball_explicit = ball;
    ball_explicit.position = Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0);
    bool success_explicit = kin::Kick(robot, ball_explicit, kin::ssl::DEFAULT_KICK_POWER);
    
    EXPECT_TRUE(success_default) << "Default kick should succeed";
    EXPECT_TRUE(success_explicit) << "Explicit default kick should succeed";
    
    // Both should result in similar velocities
    EXPECT_NEAR(ball_default.velocity.norm(), ball_explicit.velocity.norm(), 0.1)
        << "Default and explicit default power should give similar results";
    
    // Should be around 3 m/s (DEFAULT_KICK_POWER)
    EXPECT_NEAR(ball_default.velocity.norm(), 3.0, 0.2) 
        << "Default kick power should be ~3 m/s";
}

// Test 9: Ball at edge of kick range
TEST_F(KickTest, BallAtEdgeOfKickRange) {
    // Place ball exactly at maximum kick distance
    double max_kick_distance_from_center = kin::ssl::MAX_KICK_DISTANCE; 
    ball.position = Eigen::Vector3d(max_kick_distance_from_center - ball.radius_m, -ball.radius_m, 0.0);
    
    bool success_at_edge = kin::Kick(robot, ball, 3.0);
    EXPECT_TRUE(success_at_edge) << "Kick should succeed when ball is exactly at max distance";
    
    // Place ball just beyond maximum kick distance
    state::Ball ball_beyond = ball;
    ball_beyond.position = Eigen::Vector3d(max_kick_distance_from_center - ball.radius_m + 0.001, -ball.radius_m, 0.0);
    
    bool success_beyond = kin::Kick(robot, ball_beyond, 3.0);
    EXPECT_FALSE(success_beyond) << "Kick should fail when ball is just beyond max distance";
}

// Test 10: Multiple kicks (should not be possible in rapid succession)
TEST_F(KickTest, MultipleKicks) {
    // First kick should succeed
    bool first_kick = kin::Kick(robot, ball, 3.0);
    EXPECT_TRUE(first_kick) << "First kick should succeed";
    
    Eigen::Vector3d velocity_after_first = ball.velocity;
    
    // Reset ball position but keep velocity
    ball.position = Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0);
    
    // Second immediate kick might have different behavior
    // (in real implementation, there might be a cooldown)
    bool second_kick = kin::Kick(robot, ball, 3.0);
    
    // Both kicks should succeed in this test setup
    EXPECT_TRUE(second_kick) << "Second kick should also succeed in test environment";
}

// Test 11: Kick with different ball positions relative to robot
TEST_F(KickTest, KickWithDifferentBallPositions) {
    // Ball directly in front
    ball.position = Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0);
    state::Ball ball_front = ball;
    bool success_front = kin::Kick(robot, ball_front, 3.0);
    
    // Ball slightly to the right but still in front sector
    ball.position = Eigen::Vector3d(0.10 - ball.radius_m, 0.02 - ball.radius_m, 0.0); // Adjusted Y position to be clearly in front sector
    state::Ball ball_right = ball;
    bool success_right = kin::Kick(robot, ball_right, 3.0);
    
    // Ball slightly to the left but still in front sector  
    ball.position = Eigen::Vector3d(0.10 - ball.radius_m, -0.02 - ball.radius_m, 0.0); // Adjusted Y position to be clearly in front sector
    state::Ball ball_left = ball;
    bool success_left = kin::Kick(robot, ball_left, 3.0);
    
    EXPECT_TRUE(success_front) << "Kick should succeed for ball directly in front";
    EXPECT_TRUE(success_right) << "Kick should succeed for ball slightly to right";
    EXPECT_TRUE(success_left) << "Kick should succeed for ball slightly to left";
    
    // All kicks should result in substantial forward velocity
    EXPECT_GT(ball_front.velocity.x(), 2.5) << "Front kick should have good forward velocity";
    EXPECT_GT(ball_right.velocity.x(), 2.0) << "Right kick should have forward velocity";
    EXPECT_GT(ball_left.velocity.x(), 2.0) << "Left kick should have forward velocity";
}

// Test 12: Pass power vs kick power
TEST_F(KickTest, PassPowerVsKickPower) {
    // Test with pass power (lighter kick)
    state::Ball ball_pass = ball;
    bool success_pass = kin::Kick(robot, ball_pass, kin::ssl::DEFAULT_PASS_POWER);
    
    // Test with kick power (stronger kick)
    state::Ball ball_kick = ball;
    ball_kick.position = Eigen::Vector3d(0.12 - ball.radius_m, -ball.radius_m, 0.0);
    bool success_kick = kin::Kick(robot, ball_kick, kin::ssl::DEFAULT_KICK_POWER);
    
    EXPECT_TRUE(success_pass) << "Pass should succeed";
    EXPECT_TRUE(success_kick) << "Kick should succeed";
    
    // Pass should have lower velocity than kick
    EXPECT_LT(ball_pass.velocity.norm(), ball_kick.velocity.norm()) 
        << "Pass should have lower velocity than kick";
    
    // Check specific values
    EXPECT_NEAR(ball_pass.velocity.norm(), 2.0, 0.2) << "Pass should be ~2 m/s";
    EXPECT_NEAR(ball_kick.velocity.norm(), 3.0, 0.2) << "Kick should be ~3 m/s";
}

// Test 13: Ball physics after kick (integration test)
TEST_F(KickTest, BallPhysicsAfterKick) {
    // Apply kick
    bool success = kin::Kick(robot, ball, 4.0);
    EXPECT_TRUE(success) << "Kick should succeed";
    
    // Store initial velocity after kick
    Eigen::Vector3d initial_velocity = ball.velocity;
    Eigen::Vector3d initial_position = ball.position;
    
    // Simulate physics for one time step
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double dt = 0.1; // 100ms
    ball.UpdatePhysics(ball.position, ball.velocity, acceleration, dt);
    
    // Ball should have moved in kick direction
    EXPECT_GT(ball.position.x(), initial_position.x()) << "Ball should move forward after kick";
    
    // Velocity should decrease due to friction (ball physics)
    EXPECT_LT(ball.velocity.norm(), initial_velocity.norm()) 
        << "Ball velocity should decrease due to friction";
    
    // But ball should still be moving
    EXPECT_GT(ball.velocity.norm(), 0.5) << "Ball should still be moving after physics update";
}

// Test 14: SSL compliance - no attachment during kick
TEST_F(KickTest, NoAttachmentDuringKick) {
    // Ensure ball is not attached initially
    ball.is_attached = false;
    ball.attached_to = nullptr;
    
    bool success = kin::Kick(robot, ball, 3.0);
    
    EXPECT_TRUE(success) << "Kick should succeed";
    EXPECT_FALSE(ball.is_attached) << "Ball should not be attached during kick (SSL rule)";
    EXPECT_EQ(ball.attached_to, nullptr) << "Ball should not be attached to any object";
}

// Test 15: Zero power kick
TEST_F(KickTest, ZeroPowerKick) {
    Eigen::Vector3d initial_velocity = ball.velocity;
    
    bool success = kin::Kick(robot, ball, 0.0);
    
    // Zero power kick might succeed but not change velocity
    if (success) {
        EXPECT_EQ(ball.velocity.norm(), initial_velocity.norm()) 
            << "Zero power kick should not change ball velocity";
    }
    
    // Or it might fail depending on implementation
    // Either behavior is acceptable for zero power
}