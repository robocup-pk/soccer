#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>
#include <libserial/SerialPort.h>
#include "SensorDriver.h"

class SensorDriverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto null_serial = std::make_shared<LibSerial::SerialPort>();
    sensor_driver = std::make_unique<hw::SensorDriver>(null_serial);
  }

  void TearDown() override { sensor_driver.reset(); }

  std::unique_ptr<hw::SensorDriver> sensor_driver;
};

TEST_F(SensorDriverTest, TestSetAndGetAngularVelocity) {
  double w = 1.5;
  sensor_driver->SetAngularVelocityRadps(w);
  double retrieved = sensor_driver->GetAngularVelocityRadps();
  EXPECT_NEAR(retrieved, w, 1e-6);
}

TEST_F(SensorDriverTest, TestSendWheelSpeedRpm) {
  Eigen::Vector4d speeds(50, 50, 50, 50);
  EXPECT_NO_FATAL_FAILURE(sensor_driver->SendWheelSpeedRpm(speeds));
}

TEST_F(SensorDriverTest, TestGetSensorsData) {
  auto [rpms, gyro] = sensor_driver->GetSensorsData();
  EXPECT_EQ(rpms.size(), 4);
  EXPECT_GE(gyro, 0.0);
}

TEST_F(SensorDriverTest, TestGyroCalibration) { EXPECT_TRUE(sensor_driver->IsGyroCalibrated()); }

TEST_F(SensorDriverTest, TestNewDataAvailable) {
  EXPECT_TRUE(sensor_driver->NewDataAvailable());  // Initially true
  sensor_driver->new_data_available = false;
  EXPECT_FALSE(sensor_driver->NewDataAvailable());
}

TEST_F(SensorDriverTest, TestVerifyRpms) {
  std::vector<int> valid_rpms = {100, 100, 100, 100};
  EXPECT_TRUE(sensor_driver->VerifyRpms(valid_rpms));

  std::vector<int> invalid_rpms = {100, 100, 100, 500};  // Invalid RPM
  EXPECT_FALSE(sensor_driver->VerifyRpms(invalid_rpms));
}