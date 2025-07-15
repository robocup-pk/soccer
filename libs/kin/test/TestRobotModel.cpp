#include <gtest/gtest.h>
#include <cmath>

#include <Eigen/Dense>

#include "RobotModel.h"

class RobotModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    kin::RobotDescription robot_desc;
    // Square configuration with wheels at corners
    robot_desc.wheel_positions_m = {
        {0.15, 0.15},    // wheel 1: front-left
        {-0.15, 0.15},   // wheel 2: rear-left
        {-0.15, -0.15},  // wheel 3: rear-right
        {0.15, -0.15}    // wheel 4: front-right
    };

    // Wheel angles (perpendicular to radial direction for typical omniwheel setup)
    robot_desc.wheel_angles_rad = {
        -M_PI / 4,     // -45° (wheel 1)
        M_PI / 4,      // 45° (wheel 2)
        3 * M_PI / 4,  // 135° (wheel 3)
        -3 * M_PI / 4  // -135° (wheel 4)
    };

    robot_model = std::make_unique<kin::RobotModel>(robot_desc);

    // Tolerance for floating point comparisons
    tolerance = 1e-6;
  }

  // Helper function to check if two vectors are approximately equal
  bool VectorsEqual(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, double tol = 1e-6) {
    if (v1.size() != v2.size()) return false;
    return (v1 - v2).norm() < tol;
  }

  // Helper function to check if a value is approximately zero
  bool IsZero(double value, double tol = 1e-6) { return std::abs(value) < tol; }

  kin::RobotDescription robot_desc;
  std::unique_ptr<kin::RobotModel> robot_model;
  double tolerance;
};

// Test 1: Robot goes straight forward (positive X direction)
TEST_F(RobotModelTest, StraightForward) {
  // For this wheel configuration, equal wheel speeds should create rotation
  // To go straight forward, we need to solve for the required wheel speeds
  Eigen::Vector3d desired_velocity(0.1, 0.0, 0.0);  // 0.1 m/s forward, no lateral, no rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Front left should equal rear left
  EXPECT_NEAR(wheel_speeds(0), wheel_speeds(1), tolerance);
  // Front right should equal rear right
  EXPECT_NEAR(wheel_speeds(2), wheel_speeds(3), tolerance);
  EXPECT_NEAR(wheel_speeds(0), -wheel_speeds(2), tolerance);
  EXPECT_NEAR(std::abs(wheel_speeds(0)), std::abs(wheel_speeds(2)), tolerance);

  // Verify forward kinematics gives us back the desired velocity
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_NEAR(actual_velocity(0), 0.1, tolerance);     // vx = 0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 2: Robot goes left (positive Y direction)
TEST_F(RobotModelTest, StraightLeft) {
  Eigen::Vector3d desired_velocity(0.0, 0.1, 0.0);  // 0.1 m/s left, no forward, no rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Front left be opposite to rear left
  EXPECT_NEAR(wheel_speeds(0), -wheel_speeds(1), tolerance);
  // Same with FR and RR
  EXPECT_NEAR(wheel_speeds(2), -wheel_speeds(3), tolerance);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_TRUE(IsZero(actual_velocity(0), tolerance));  // vx = 0
  EXPECT_NEAR(actual_velocity(1), 0.1, tolerance);     // vy = 0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 3: Robot goes right (negative Y direction)
TEST_F(RobotModelTest, StraightRight) {
  Eigen::Vector3d desired_velocity(0.0, -0.1, 0.0);  // 0.1 m/s right, no forward, no rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_TRUE(IsZero(actual_velocity(0), tolerance));  // vx = 0
  EXPECT_NEAR(actual_velocity(1), -0.1, tolerance);    // vy = -0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 4: Robot goes backward (negative X direction)
TEST_F(RobotModelTest, StraightBackward) {
  Eigen::Vector3d desired_velocity(-0.1, 0.0, 0.0);  // 0.1 m/s backward, no lateral, no
                                                     // rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_NEAR(actual_velocity(0), -0.1, tolerance);    // vx = -0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 5: Robot turns counter-clockwise (positive angular velocity)
TEST_F(RobotModelTest, TurnCounterClockwise) {
  Eigen::Vector3d desired_velocity(0.0, 0.0, 0.5);  // No translation, 0.5 rad/s CCW rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_TRUE(IsZero(actual_velocity(0), tolerance));  // vx = 0
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_NEAR(actual_velocity(2), 0.5, tolerance);     // ω = 0.5 rad/s
}

// Test 6: Robot turns clockwise (negative angular velocity)
TEST_F(RobotModelTest, TurnClockwise) {
  Eigen::Vector3d desired_velocity(0.0, 0.0, -0.5);  // No translation, 0.5 rad/s CW rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_TRUE(IsZero(actual_velocity(0), tolerance));  // vx = 0
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_NEAR(actual_velocity(2), -0.5, tolerance);    // ω = -0.5 rad/s
}

// Test 7: Combined motion - diagonal movement
TEST_F(RobotModelTest, DiagonalMotion) {
  Eigen::Vector3d desired_velocity(0.1, 0.1, 0.0);  // Forward + left, no rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_NEAR(actual_velocity(0), 0.1, tolerance);     // vx = 0.1 m/s
  EXPECT_NEAR(actual_velocity(1), 0.1, tolerance);     // vy = 0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 8: Combined motion - forward while turning
TEST_F(RobotModelTest, ForwardWhileTurning) {
  Eigen::Vector3d desired_velocity(0.1, 0.0, 0.3);  // Forward + CCW rotation

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_NEAR(actual_velocity(0), 0.1, tolerance);     // vx = 0.1 m/s
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_NEAR(actual_velocity(2), 0.3, tolerance);     // ω = 0.3 rad/s
}

// Test 9: Zero motion
TEST_F(RobotModelTest, ZeroMotion) {
  Eigen::Vector3d desired_velocity(0.0, 0.0, 0.0);  // No motion

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(desired_velocity);

  // All wheel speeds should be zero
  for (int i = 0; i < wheel_speeds.size(); ++i) {
    EXPECT_TRUE(IsZero(wheel_speeds(i), tolerance));
  }

  // Verify forward kinematics
  Eigen::Vector3d actual_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  EXPECT_TRUE(IsZero(actual_velocity(0), tolerance));  // vx = 0
  EXPECT_TRUE(IsZero(actual_velocity(1), tolerance));  // vy = 0
  EXPECT_TRUE(IsZero(actual_velocity(2), tolerance));  // ω = 0
}

// Test 10: Jacobian properties
TEST_F(RobotModelTest, JacobianProperties) {
  Eigen::MatrixXd J = robot_model->InverseMapping();
  Eigen::MatrixXd J_plus = robot_model->ForwardMapping();

  // Check dimensions
  EXPECT_EQ(J.rows(), 4);       // 4 wheels
  EXPECT_EQ(J.cols(), 3);       // 3 DOF (x, y, theta)
  EXPECT_EQ(J_plus.rows(), 3);  // 3 DOF
  EXPECT_EQ(J_plus.cols(), 4);  // 4 wheels

  // Test that J+ is indeed the pseudoinverse of J
  // For overdetermined systems: J+ * J = I (identity)
  Eigen::Matrix3d should_be_identity = J_plus * J;
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(should_be_identity(i, j), identity(i, j), tolerance);
    }
  }
}

// Test 11: Consistency check - round trip
TEST_F(RobotModelTest, RoundTripConsistency) {
  // Start with robot velocity, convert to wheel speeds, then back to robot velocity
  Eigen::Vector3d original_velocity(0.05, 0.03, 0.2);

  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(original_velocity);
  Eigen::Vector3d recovered_velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  // Should get back the original velocity
  EXPECT_NEAR(recovered_velocity(0), original_velocity(0), tolerance);
  EXPECT_NEAR(recovered_velocity(1), original_velocity(1), tolerance);
  EXPECT_NEAR(recovered_velocity(2), original_velocity(2), tolerance);
}

// Test 12: Units consistency
TEST_F(RobotModelTest, UnitsConsistency) {
  // Test that wheel radius is properly embedded in Jacobians
  Eigen::Vector3d robot_vel(1.0, 0.0, 0.0);  // 1 m/s forward
  Eigen::VectorXd wheel_speeds = robot_model->RobotVelocityToWheelSpeedsRps(robot_vel);

  // For 1 m/s robot velocity and 0.05m wheel radius,
  // wheel surface speed should be 1 m/s, so angular speed = 1/0.05 = 20 rad/s
  // (This will depend on the specific wheel configuration, but we can check magnitude)

  // At least verify that wheel speeds are non-zero and reasonable
  bool has_nonzero_speed = false;
  for (int i = 0; i < wheel_speeds.size(); ++i) {
    if (std::abs(wheel_speeds(i)) > tolerance) {
      has_nonzero_speed = true;
      // Wheel speeds should be reasonable (not extremely large or small)
      EXPECT_LT(std::abs(wheel_speeds(i)), 1000.0);  // Less than 1000 rad/s
      EXPECT_GT(std::abs(wheel_speeds(i)), 0.1);     // Greater than 0.1 rad/s
    }
  }
  EXPECT_TRUE(has_nonzero_speed);
}

// Test 13: Specific wheel speed patterns
TEST_F(RobotModelTest, SpecificWheelSpeedPatterns) {
  // Test with equal wheel speeds (should produce rotation for this configuration)
  std::vector<double> equal_speeds = {1.0, 1.0, 1.0, 1.0};
  Eigen::Vector3d velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(equal_speeds);

  // For the wheel configuration we have (perpendicular to radial),
  // equal wheel speeds should produce primarily rotational motion
  // The exact values depend on the geometry, but we can check that rotation is non-zero
  EXPECT_FALSE(IsZero(velocity(2), tolerance));  // Should have rotational component

  // Go straight
  std::vector<double> opposite_speeds = {1.0, 1.0, -1.0, -1.0};
  velocity = robot_model->WheelSpeedsRadpsToRobotVelocity(opposite_speeds);
  EXPECT_GE(velocity(0), 0);
  EXPECT_NEAR(0, velocity(1), tolerance);  // shouldn't go sideways
  EXPECT_NEAR(0, velocity(2), tolerance);  // shouldn't turn
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}