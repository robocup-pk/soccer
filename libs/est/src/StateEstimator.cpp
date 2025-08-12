#include <cmath>
#include <iostream>

#include "StateEstimator.h"
#include "HwConfig.h"
#include "Utils.h"

#ifndef M_PI  // Need for Windows
#define M_PI 3.14159265
#endif

est::StateEstimator::StateEstimator() {
  std::cout << "[est::StateEstimator::StateEstimator]" << std::endl;
  robot_model = std::make_shared<kin::RobotModel>();
  angle_random_walk_per_rt_t = 0.001;
  initialized_pose = false;
  pose_init << 0, 0, 0;
  pose_est << 0, 0, 0;
  initialized_gyro = false;
  initialized_encoder = false;
  
  // Initialize analysis variables
  last_kalman_gain = Eigen::Matrix3d::Zero();
  last_innovation = Eigen::Vector3d::Zero();
  last_innovation_cov = Eigen::Matrix3d::Zero();
  // P_init
  init_sigma_m = 1;
  init_sigma_rad = 2 * M_PI;

  // Q
  process_sigma_m = 0.1;// Old value = 0.01
  process_sigma_rad = 0.2 * M_PI;

  // R
  meas_sigma_m = 0.01;
  meas_sigma_rad = 0.1 * M_PI;

  state_cov << std::pow(init_sigma_m, 2), 0, 0, 0, std::pow(init_sigma_m, 2), 0, 0, 0,
      std::pow(init_sigma_rad, 2);

  process_cov << std::pow(process_sigma_m, 2), 0, 0, 0, std::pow(process_sigma_m, 2), 0, 0, 0,
      std::pow(process_sigma_rad, 2);

  meas_cov << std::pow(meas_sigma_m, 2), 0, 0, 0, std::pow(meas_sigma_m, 2), 0, 0, 0,
      std::pow(meas_sigma_rad, 2);
}

void est::StateEstimator::NewMotorsData(const Eigen::Vector4d& wheel_speeds_rpm) {
  if (!initialized_pose) {
    std::cout << "[est::StateEstimator::NewEncoderData] Pose uninitialized!" << std::endl;
    return;
  }

  if (!initialized_encoder) {
    initialized_encoder = true;
    t_last_encoder = util::GetCurrentTime();
    return;
  }

  double dt = util::GetCurrentTime() - t_last_encoder;
  if (dt <= 0) {
    std::cout << "[est::StateEstimator::NewMotorsData] dt <= 0. Some serious problem!"
              << std::endl;
    return;
  }
  t_last_encoder = util::GetCurrentTime();

  // Calculate body velocity from wheel speeds [Vb = J * Vw]
  Eigen::Vector3d velocity_fBody = robot_model->WheelSpeedsRpmToRobotVelocity(wheel_speeds_rpm);
  Eigen::Vector3d velocity_fWorld = util::RotateAboutZ(velocity_fBody, pose_est[2]);

  // State Transfer Function
  double dx = -dt * (velocity_fBody[0] * std::sin(pose_est[2]) +
                     velocity_fBody[1] * std::cos(pose_est[2]));
  double dy =
      dt * (velocity_fBody[0] * std::cos(pose_est[2]) - velocity_fBody[1] * std::sin(pose_est[2]));
  Eigen::Matrix3d phi;
  phi << 1, 0, dx, 0, 1, dy, 0, 0, 1;

  // Predict Pose
  pose_est[0] += velocity_fWorld[0] * dt;
  pose_est[1] += velocity_fWorld[1] * dt;
  // pose_est[2] = util::WrapAngle(pose_est[2] + velocity_fWorld[2] * dt);

  // Predict Covariance
  state_cov = phi * state_cov * phi.transpose() + process_cov;
}

void est::StateEstimator::NewGyroData(double w_radps) {
  if (!initialized_pose) {
    std::cout << "[est::StateEstimator::NewGyroData] Pose uninitialized!" << std::endl;
    return;
  }

  if (!initialized_gyro) {
    initialized_gyro = true;
    t_last_gyro = util::GetCurrentTime();
    return;
  }

  // Predict Pose
  double dt = util::GetCurrentTime() - t_last_gyro;
  t_last_gyro = util::GetCurrentTime();
  pose_est[2] = util::WrapAngle(pose_est[2] + w_radps * dt);

  // Predict Covariance
  double q = angle_random_walk_per_rt_t * std::sqrt(dt);
  state_cov(2, 2) += q;
}

void est::StateEstimator::NewCameraData(const Eigen::Vector3d& pose_meas) {
  // Camera data provides pose measurements, so call Update
  Update(pose_meas);
}

void est::StateEstimator::Predict(const Eigen::Vector4d& wheel_speeds_rpm, double gyro_w_radps) {
  if (!initialized_pose) {
    std::cout << "[est::StateEstimator::Predict] Pose uninitialized!" << std::endl;
    return;
  }

  if (!initialized_encoder) {
    initialized_encoder = true;
    t_last_encoder = util::GetCurrentTime();
    return;
  }

  double dt = util::GetCurrentTime() - t_last_encoder;
  if (dt <= 0) {
    std::cout << "[est::StateEstimator::Predict] dt <= 0. Some serious problem!"
              << std::endl;
    return;
  }
  t_last_encoder = util::GetCurrentTime();

  // Calculate body velocity from wheel speeds
  Eigen::Vector3d velocity_fBody = robot_model->WheelSpeedsRpmToRobotVelocity(wheel_speeds_rpm);

  // Use gyro data for angular velocity if available
  if (initialized_gyro) {
    velocity_fBody[2] = gyro_w_radps;
  }

  // State transition model
  double c_theta = std::cos(pose_est[2]);
  double s_theta = std::sin(pose_est[2]);
  double v_x = velocity_fBody[0];
  double v_y = velocity_fBody[1];

  pose_est[0] += (v_x * c_theta - v_y * s_theta) * dt;
  pose_est[1] += (v_x * s_theta + v_y * c_theta) * dt;
  pose_est[2] += velocity_fBody[2] * dt;
  pose_est[2] = util::WrapAngle(pose_est[2]);

  // Jacobian of state transition function
  Eigen::Matrix3d F;
  F << 1, 0, (-v_x * s_theta - v_y * c_theta) * dt,
       0, 1, ( v_x * c_theta - v_y * s_theta) * dt,
       0, 0, 1;

  // Predict Covariance
  state_cov = F * state_cov * F.transpose() + process_cov;
}

void est::StateEstimator::Update(const Eigen::Vector3d& pose_meas) {
  if (!initialized_pose) {
    initialized_pose = true;
    pose_est = pose_meas;
    pose_init = pose_meas;
    std::cout << "[est::StateEstimator::Update] Initialized pose: " << pose_init.transpose()
              << std::endl;
    return;
  }

  Eigen::Vector3d innovation = pose_meas - pose_est;
  innovation[2] = util::WrapAngle(innovation[2]); // Normalize angle innovation
  Eigen::Matrix3d innovation_cov = state_cov + meas_cov;
  Eigen::Matrix3d kalman_gain = state_cov * innovation_cov.inverse();
  
  // Store for analysis
  last_innovation = innovation;
  last_innovation_cov = innovation_cov;
  last_kalman_gain = kalman_gain;

  // Pose Update
  pose_est += kalman_gain * innovation;
  pose_est[2] = util::WrapAngle(pose_est[2]);

  // Covariance Update: Joseph form
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  state_cov = (identity - kalman_gain) * state_cov * (identity - kalman_gain).transpose() +
              kalman_gain * meas_cov * kalman_gain.transpose();

  // Symmetrize covariance to ensure numerical stability
  state_cov = 0.5 * (state_cov + state_cov.transpose());
}

Eigen::Vector3d est::StateEstimator::GetPose() { return pose_est; }

Eigen::Vector3d est::StateEstimator::GetPoseInit() { return pose_init; }

void est::StateEstimator::SetPose(Eigen::Vector3d pose) { this->pose_est = pose; }

est::StateEstimator::~StateEstimator() { robot_model = nullptr; }

void est::StateEstimator::InitializePose(Eigen::Vector3d pose_fWorld_init) {
  pose_init = pose_fWorld_init;
  initialized_pose = true;
}