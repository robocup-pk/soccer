#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <memory>

#include "RobotModel.h"

namespace est {
class StateEstimator {
 public:
  StateEstimator();

  void Predict(const Eigen::Vector4d& wheel_speeds_rpm, double gyro_w_radps);
  void Update(const Eigen::Vector3d& pose_meas);
  
  void NewMotorsData(const Eigen::Vector4d& wheel_speeds_rpm);
  void NewGyroData(double w_radps);
  void NewCameraData(const Eigen::Vector3d& pose_meas);

  void InitializePose(Eigen::Vector3d pose_fWorld_init);

  Eigen::Vector3d GetPose();
  Eigen::Vector3d GetPoseInit();
  void SetPose(Eigen::Vector3d pose);
  
  // Getters for Kalman filter internals (for debugging/analysis)
  Eigen::Matrix3d GetStateCovariance() const { return state_cov; }
  Eigen::Matrix3d GetProcessCovariance() const { return process_cov; }
  Eigen::Matrix3d GetMeasurementCovariance() const { return meas_cov; }
  
  // Store last Kalman gain and innovation for analysis
  Eigen::Matrix3d GetLastKalmanGain() const { return last_kalman_gain; }
  Eigen::Vector3d GetLastInnovation() const { return last_innovation; }
  Eigen::Matrix3d GetLastInnovationCovariance() const { return last_innovation_cov; }

  ~StateEstimator();

  bool initialized_pose;

  // Noise Parameters (Tunable)
  // =====================================================================================
  // Initial state covariance (P)
  // Represents the initial uncertainty in the robot's pose.
  // =====================================================================================
  double init_sigma_m;      // Initial uncertainty in position (m)
  double init_sigma_rad;    // Initial uncertainty in orientation (rad)

  // =====================================================================================
  // Process noise covariance (Q)
  // Represents the uncertainty in the robot's motion model.
  // =====================================================================================
  double process_sigma_m;   // Uncertainty in position due to motion (m/s)
  double process_sigma_rad; // Uncertainty in orientation due to motion (rad/s)

  // =====================================================================================
  // Measurement noise covariance (R)
  // Represents the uncertainty in the camera measurements.
  // =====================================================================================
  double meas_sigma_m;      // Uncertainty in camera position measurement (m)
  double meas_sigma_rad;    // Uncertainty in camera orientation measurement (rad)

 private:
  // Encoders
  bool initialized_encoder;
  double t_last_encoder;

  // Gyro
  bool initialized_gyro;
  double angle_random_walk_per_rt_t;
  double t_last_gyro;

  // Pose
  Eigen::Vector3d pose_init;
  Eigen::Vector3d pose_est;

  // Noise
  Eigen::Matrix3d state_cov;
  Eigen::Matrix3d process_cov;
  Eigen::Matrix3d meas_cov;
  
  // Store last update values for analysis
  Eigen::Matrix3d last_kalman_gain;
  Eigen::Vector3d last_innovation;
  Eigen::Matrix3d last_innovation_cov;

  std::shared_ptr<kin::RobotModel> robot_model;
};
}  // namespace est

#endif  // ESTIMATOR_H