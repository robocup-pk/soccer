#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>
#include "HardwareManager.h"

class HardwareManagerTest : public ::testing::Test {
 protected:
  void SetUp() override { hw_manager = std::make_unique<hw::HardwareManager>(); }

  void TearDown() override { hw_manager.reset(); }

  std::unique_ptr<hw::HardwareManager> hw_manager;
};

TEST_F(HardwareManagerTest, TestSetBodyVelocity) {
  Eigen::Vector3d velocity(0.5, 0.3, 0.1);
  EXPECT_NO_FATAL_FAILURE(hw_manager->SetBodyVelocity(velocity));
}

TEST_F(HardwareManagerTest, TestNewMotorsRpms) {
  // In MODEL mode, should return data
  auto rpms = hw_manager->NewMotorsRpms();
  EXPECT_TRUE(rpms.has_value());
  if (rpms.has_value()) {
    EXPECT_EQ(rpms.value().size(), 4);
  }
}

TEST_F(HardwareManagerTest, TestNewGyroAngularVelocity) {
  auto gyro = hw_manager->NewGyroAngularVelocity();
  EXPECT_TRUE(gyro.has_value());
}

TEST_F(HardwareManagerTest, TestNewCameraData) {
  auto camera_data = hw_manager->NewCameraData();
  EXPECT_FALSE(camera_data.has_value());

  // Set camera data
  Eigen::Vector3d pose(1.0, 2.0, 0.5);
  hw_manager->NewCameraData(pose);
  camera_data = hw_manager->NewCameraData();
  EXPECT_TRUE(camera_data.has_value());
}

TEST_F(HardwareManagerTest, TestGyroCalibration) { EXPECT_TRUE(hw_manager->IsGyroCalibrated()); }

TEST_F(HardwareManagerTest, TestSetWheelSpeedsRpm) {
  Eigen::Vector4d wheel_speeds(100, 100, 100, 100);
  EXPECT_NO_FATAL_FAILURE(hw_manager->SetWheelSpeedsRpm(wheel_speeds));
}