#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sstream>
#include "Waypoint.h"

class WaypointTest : public ::testing::Test {
 protected:
  void SetUp() override {
    wp1 = state::Waypoint(1.0, 2.0, 0.5);
    wp2 = state::Waypoint(3.0, 4.0, 1.0);
  }

  state::Waypoint wp1, wp2;
};

TEST_F(WaypointTest, TestWaypointCreation) {
  state::Waypoint wp(1.0, 2.0, 0.5);
  EXPECT_NEAR(wp.x, 1.0, 1e-6);
  EXPECT_NEAR(wp.y, 2.0, 1e-6);
  EXPECT_NEAR(wp.angle, 0.5, 1e-6);
}

TEST_F(WaypointTest, TestCreationWithVector) {
  Eigen::Vector3d vec(1.0, 2.0, 0.5);
  state::Waypoint wp(vec);
  EXPECT_NEAR(wp.x, 1.0, 1e-6);
  EXPECT_NEAR(wp.y, 2.0, 1e-6);
  EXPECT_NEAR(wp.angle, 0.5, 1e-6);
}

TEST_F(WaypointTest, TestSubtractionOperator) {
  state::Waypoint diff = wp2 - wp1;
  EXPECT_NEAR(diff.x, 2.0, 1e-6);
  EXPECT_NEAR(diff.y, 2.0, 1e-6);
  EXPECT_NEAR(diff.angle, 0.5, 1e-6);
}

TEST_F(WaypointTest, TestAdditionOperator) {
  state::Waypoint wp_same_angle(1.0, 2.0, 0.5);
  state::Waypoint sum = wp1 + wp_same_angle;
  EXPECT_NEAR(sum.x, 2.0, 1e-6);
  EXPECT_NEAR(sum.y, 4.0, 1e-6);
  EXPECT_NEAR(sum.angle, 0.5, 1e-6);
}

TEST_F(WaypointTest, TestMultiplicationOperator) {
  state::Waypoint product = wp1 * 2.0;
  EXPECT_NEAR(product.x, 2.0, 1e-6);
  EXPECT_NEAR(product.y, 4.0, 1e-6);
  EXPECT_NEAR(product.angle, 0.5, 1e-6);
}

TEST_F(WaypointTest, TestEqualityOperator) {
  state::Waypoint wp_copy(1.0, 2.0, 0.5);
  EXPECT_TRUE(wp1 == wp_copy);

  state::Waypoint wp_different(1.1, 2.0, 0.5);
  EXPECT_FALSE(wp1 == wp_different);
}

TEST_F(WaypointTest, TestLessThanOperator) {
  state::Waypoint wp_smaller(0.5, 1.5, 0.0);
  EXPECT_TRUE(wp_smaller < wp1);

  state::Waypoint wp_larger(1.5, 2.5, 0.0);
  EXPECT_TRUE(wp1 < wp_larger);
}

TEST_F(WaypointTest, TestGreaterThanOperator) {
  state::Waypoint wp_larger(1.5, 2.5, 0.0);
  EXPECT_TRUE(wp_larger > wp1);

  state::Waypoint wp_smaller(0.5, 1.5, 0.0);
  EXPECT_TRUE(wp1 > wp_smaller);
}

TEST_F(WaypointTest, TestNorm) {
  double norm = wp1.Norm();
  double expected = std::sqrt(1.0 * 1.0 + 2.0 * 2.0);
  EXPECT_NEAR(norm, expected, 1e-6);
}

TEST_F(WaypointTest, TestNormalize) {
  state::Waypoint wp(3.0, 4.0, 0.0);
  wp.Normalize();
  EXPECT_NEAR(wp.Norm(), 1.0, 1e-6);
  EXPECT_NEAR(wp.x, 0.6, 1e-3);
  EXPECT_NEAR(wp.y, 0.8, 1e-3);
}

TEST_F(WaypointTest, TestNearPosition) {
  state::Waypoint wp_close(1.01, 2.01, 0.5);
  EXPECT_TRUE(state::NearPosition(wp1, wp_close, 0.1));

  state::Waypoint wp_far(2.0, 3.0, 0.5);
  EXPECT_FALSE(state::NearPosition(wp1, wp_far, 0.1));
}

TEST_F(WaypointTest, TestNearPose) {
  state::Waypoint wp_close(1.01, 2.01, 0.51);
  EXPECT_TRUE(state::NearPose(wp1, wp_close, 0.1));

  state::Waypoint wp_far_angle(1.01, 2.01, 1.0);
  EXPECT_TRUE(state::NearPose(wp1, wp_far_angle, 0.1));
}

TEST_F(WaypointTest, TestPathOperations) {
  state::Path path = {wp1, wp2};
  EXPECT_EQ(path.size(), 2);
}

TEST_F(WaypointTest, TestWaypointStreamOperator) {
  // Test basic waypoint output
  std::stringstream ss1;
  ss1 << wp1;
  EXPECT_EQ(ss1.str(), "(1, 2)");

  // Test with different values
  state::Waypoint wp_zero(0, 0, 0);
  std::stringstream ss2;
  ss2 << wp_zero;
  EXPECT_EQ(ss2.str(), "(0, 0)");

  // Test with negative values
  state::Waypoint wp_neg(-1.5, -2.5, 0);
  std::stringstream ss3;
  ss3 << wp_neg;
  EXPECT_EQ(ss3.str(), "(-1.5, -2.5)");
}

TEST_F(WaypointTest, TestPathStreamOperator) {
  // Test empty path
  state::Path empty_path;
  std::stringstream ss1;
  ss1 << empty_path;
  EXPECT_EQ(ss1.str(), "[]");

  // Test single waypoint path
  state::Path single_path = {wp1};
  std::stringstream ss2;
  ss2 << single_path;
  EXPECT_EQ(ss2.str(), "[(1, 2)]");

  // Test multiple waypoint path
  state::Path multi_path = {wp1, wp2};
  std::stringstream ss3;
  ss3 << multi_path;
  EXPECT_EQ(ss3.str(), "[(1, 2), (3, 4)]");

  // Test longer path
  state::Waypoint wp3(5, 6, 1);
  state::Path long_path = {wp1, wp2, wp3};
  std::stringstream ss4;
  ss4 << long_path;
  EXPECT_EQ(ss4.str(), "[(1, 2), (3, 4), (5, 6)]");
}