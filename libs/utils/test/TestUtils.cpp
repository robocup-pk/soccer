#include <gtest/gtest.h>

#include "Utils.h"
#include "Vectors.h"

TEST(UtilsTest, TestReadFile) {
  std::string file = util::ReadFile("../resources/DemoFile.txt");
  EXPECT_FALSE(file.empty());

  std::string expected =
      "This is a demo file\n"
      "This is second line\n"
      "Some numbers like 1, 2, 3\n";

  EXPECT_EQ(file, expected);
}

TEST(UtilsTest, TestGetExecutableDir) {
  std::string dir = util::GetExecutableDir();
  EXPECT_FALSE(dir.empty());
  // The directory should contain some path information
  EXPECT_TRUE(dir.find("/") != std::string::npos || dir.find("\\") != std::string::npos);
}

TEST(UtilsTest, TestRotateAboutZ) {
  Eigen::Vector3d pose(1.0, 0.0, 0.5);

  // Test zero rotation
  Eigen::Vector3d result = util::RotateAboutZ(pose, 0.0);
  EXPECT_NEAR(result[0], 1.0, 1e-6);
  EXPECT_NEAR(result[1], 0.0, 1e-6);
  EXPECT_NEAR(result[2], 0.5, 1e-6);

  // Test 90 degree rotation
  result = util::RotateAboutZ(pose, M_PI / 2);
  EXPECT_NEAR(result[0], 0.0, 1e-6);
  EXPECT_NEAR(result[1], 1.0, 1e-6);
  EXPECT_NEAR(result[2], 0.5, 1e-6);

  // Test 180 degree rotation
  result = util::RotateAboutZ(pose, M_PI);
  EXPECT_NEAR(result[0], -1.0, 1e-6);
  EXPECT_NEAR(result[1], 0.0, 1e-6);
  EXPECT_NEAR(result[2], 0.5, 1e-6);

  // Test with different pose
  Eigen::Vector3d pose2(0.0, 1.0, 0.0);
  result = util::RotateAboutZ(pose2, M_PI / 2);
  EXPECT_NEAR(result[0], -1.0, 1e-6);
  EXPECT_NEAR(result[1], 0.0, 1e-6);
  EXPECT_NEAR(result[2], 0.0, 1e-6);
}

TEST(UtilsTest, TestWrapAngle) {
  // Test angles within range
  EXPECT_NEAR(util::WrapAngle(0.0), 0.0, 1e-6);
  EXPECT_NEAR(util::WrapAngle(M_PI / 2), M_PI / 2, 1e-6);
  EXPECT_NEAR(util::WrapAngle(M_PI), M_PI, 1e-6);
  EXPECT_NEAR(util::WrapAngle(-M_PI / 2), -M_PI / 2, 1e-6);

  // Test angles that need wrapping
  EXPECT_NEAR(util::WrapAngle(2 * M_PI), 0.0, 1e-6);
  EXPECT_NEAR(util::WrapAngle(3 * M_PI), M_PI, 1e-6);
  EXPECT_NEAR(util::WrapAngle(-2 * M_PI), 0.0, 1e-6);

  // The function maps -π to +π (they are equivalent)
  EXPECT_NEAR(util::WrapAngle(-M_PI), M_PI, 1e-6);
  EXPECT_NEAR(util::WrapAngle(-3 * M_PI), M_PI, 1e-6);

  // Test angles slightly outside range
  EXPECT_NEAR(util::WrapAngle(M_PI + 0.1), -M_PI + 0.1, 1e-6);
  EXPECT_NEAR(util::WrapAngle(-M_PI - 0.1), M_PI - 0.1, 1e-6);

  // Test edge cases
  EXPECT_NEAR(util::WrapAngle(M_PI), M_PI, 1e-6);
}
TEST(UtilsTest, TestGetCurrentTime) {
  double time1 = util::GetCurrentTime();
  double time2 = util::GetCurrentTime();

  EXPECT_GE(time2, time1);
  EXPECT_TRUE(time1 >= 0.0);
  EXPECT_TRUE(time2 >= 0.0);

  util::WaitMs(10);
  double time3 = util::GetCurrentTime();
  EXPECT_GT(time3, time2);
}

TEST(UtilsTest, TestWaitMs) {
  auto start = std::chrono::high_resolution_clock::now();
  util::WaitMs(50);
  auto end = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  EXPECT_GE(duration.count(), 0);
}

TEST(UtilsTest, TestCalculateDt) {
  // First call should return some dt
  double dt1 = util::CalculateDt();
  EXPECT_TRUE(dt1 >= 0.0);

  // Wait and call again
  util::WaitMs(10);
  double dt2 = util::CalculateDt();
  EXPECT_TRUE(dt2 > 0.0);
  EXPECT_TRUE(dt2 < 1.0);  // Should be reasonable

  // Call again quickly
  double dt3 = util::CalculateDt();
  EXPECT_TRUE(dt3 >= 0.0);
  EXPECT_TRUE(dt3 < dt2);  // Should be smaller
}

TEST(UtilsTest, TestPixelsPerMm) {
  float ppm = util::PixelsPerMm();
  EXPECT_TRUE(ppm > 0.0f);
  // Should be a reasonable value based on px_per_m
  EXPECT_TRUE(ppm < 1.0f);
}

TEST(UtilsTest, TestMmToPixels) {
  float mm_value = 100.0f;
  float pixels = util::MmToPixels(mm_value);
  EXPECT_TRUE(pixels > 0.0f);

  // Test with zero
  EXPECT_EQ(util::MmToPixels(0.0f), 0.0f);

  // Test proportionality
  float pixels1 = util::MmToPixels(10.0f);
  float pixels2 = util::MmToPixels(20.0f);
  EXPECT_EQ(pixels2 / pixels1, 2.0f);
}

TEST(UtilsTest, TestComputeAnglefromGyroData) {
  // Test with constant angular velocity
  double angular_velocity = 1.0;  // 1 rad/s
  util::WaitMs(10);
  double angle1 = util::ComputeAnglefromGyroData(angular_velocity);

  util::WaitMs(10);
  double angle2 = util::ComputeAnglefromGyroData(angular_velocity);

  EXPECT_GT(angle2, angle1);
  EXPECT_TRUE(angle1 >= 0.0);
  EXPECT_TRUE(angle2 >= 0.0);

  // Test with zero velocity
  util::WaitMs(10);
  double angle3 = util::ComputeAnglefromGyroData(0.0);
  util::WaitMs(10);
  double angle4 = util::ComputeAnglefromGyroData(0.0);

  EXPECT_NEAR(angle4, angle3, 1e-3);  // Should not change much

  // Test with negative velocity
  util::WaitMs(10);
  double angle5 = util::ComputeAnglefromGyroData(-0.5);
  EXPECT_LT(angle5, angle4);
}

TEST(UtilsTest, TestEigenGlmVectorMultiplication) {
  using namespace util;

  // Test multiplication: Eigen::Vector2f * glm::vec2 -> glm::vec2
  Eigen::Vector2f eigen_vec(2.0f, 3.0f);
  glm::vec2 glm_vec(4.0f, 5.0f);

  glm::vec2 result = eigen_vec * glm_vec;
  EXPECT_NEAR(result.x, 8.0f, 1e-6f);   // 2.0 * 4.0
  EXPECT_NEAR(result.y, 15.0f, 1e-6f);  // 3.0 * 5.0

  // Test with zero
  Eigen::Vector2f eigen_zero(0.0f, 0.0f);
  glm::vec2 result_zero = eigen_zero * glm_vec;
  EXPECT_NEAR(result_zero.x, 0.0f, 1e-6f);
  EXPECT_NEAR(result_zero.y, 0.0f, 1e-6f);

  // Test with negative values
  Eigen::Vector2f eigen_neg(-1.0f, -2.0f);
  glm::vec2 glm_neg(-3.0f, -4.0f);
  glm::vec2 result_neg = eigen_neg * glm_neg;
  EXPECT_NEAR(result_neg.x, 3.0f, 1e-6f);  // (-1.0) * (-3.0)
  EXPECT_NEAR(result_neg.y, 8.0f, 1e-6f);  // (-2.0) * (-4.0)

  // Test with different values
  Eigen::Vector2f eigen_test(1.5f, -2.5f);
  glm::vec2 glm_test(2.0f, 3.0f);
  glm::vec2 result_test = eigen_test * glm_test;
  EXPECT_NEAR(result_test.x, 3.0f, 1e-6f);   // 1.5 * 2.0
  EXPECT_NEAR(result_test.y, -7.5f, 1e-6f);  // -2.5 * 3.0
}