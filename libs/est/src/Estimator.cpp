#include <cmath>

#include "Estimator.h"
#include "HwConfig.h"
#include "Utils.h"

#ifndef M_PI  // Need for Windows
#define M_PI 3.14159265
#endif

est::Estimator::Estimator() {
  initialized_encoder = false;
  // P_init
  init_sigma_m = 1;
  init_sigma_rad = 2 * M_PI;
  // Q
  process_sigma_m = 0.001;
  process_sigma_rad = 0.1 * M_PI;

  // R
  meas_sigma_m = 0.001;
  meas_sigma_rad = 0.1 * M_PI;

  state_cov << std::pow(init_sigma_m, 2), 0, 0, 0, std::pow(init_sigma_m, 2), 0, 0, 0,
      std::pow(init_sigma_rad, 2);

  process_cov << std::pow(process_sigma_m, 2), 0, 0, 0, std::pow(process_sigma_m, 2), 0, 0, 0,
      std::pow(process_sigma_rad, 2);

  meas_cov << std::pow(meas_sigma_m, 2), 0, 0, 0, std::pow(meas_sigma_m, 2), 0, 0, 0,
      std::pow(meas_sigma_rad, 2);

  // Create a typical omniwheel robot configuration
  robot_desc.wheel_radius_m = 0.05;  // 5cm wheels

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
}

void est::Estimator::NewEncoderData(Eigen::Vector4d ticks) {
  if (!initialized_pose) return;

  if (!initialized_encoder) {
    initialized_encoder = true;
    Eigen::Vector4d last_ticks = ticks;
    t_last_encoder = util::GetCurrentTime();
    return;
  }

  // Calculate wheel speeds from encoder ticks
  double dt = util::GetCurrentTime() - t_last_encoder;
  Eigen::Vector4d wheel_speeds;
  for (int i = 0; i < robot_desc.num_wheels; i++) {
    double d_ticks = ticks[i] - last_ticks[i];
    double num_rev = d_ticks / hw::Config::ticks_per_rev;
    double dst_per_rev = 2 * M_PI * robot_desc.wheel_radius_m;
    double tot_dst = dst_per_rev * num_rev;
    wheel_speeds[i] = tot_dst / dt;
  }

  // Calculate body velocity from wheel speeds [Vb = J * Vw]
  Eigen::Vector3d velocity_fBody = robot_model->WheelSpeedsToRobotVelocity(
      std::vector<double>(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size()));

  // Transform body velocities to world frame
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
  pose_est[2] = util::WrapAngle(pose_est[2] + velocity_fWorld[2] * dt);

  // Predict Covariance
  state_cov += phi * state_cov * phi.transpose() + process_cov;

  t_last_encoder = util::GetCurrentTime();
}

void est::Estimator::NewGyroData(double w_radps) {
  if (!initialized_pose) return;

  if (!initialized_gyro) {
    initializd_gyro = true;
    t_last_gyro = util::GetCurrentTime();
    return;
  }

  // Predict Pose
  double dt = util::GetCurrentTime() - t_last_gyro;
  pose_est[2] = util::WrapAngle(pose_est[2] + w_radps * dt);

  // Predict Covariance
  double q = angle_random_walk_per_rt_t * std::sqrt(dt);
  state_cov(2, 2) += q;

  t_last_gyro = util::GetCurrentTime();
}

void est::Estimator::NewCameraData(Eigen::Vector3d pose_meas) {
  if (!initialized_pose) {
    initialized_pose = true;
    pose_est = pose_meas;
    return;
  }

  Eigen::Vector3d innovation = pose_meas - pose_est;
  Eigen::Matrix3d jacobian_h = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d innovation_cov = jacobian_h * state_cov * jacobian_h.transpose() + meas_cov;
  Eigen::Matrix3d kalman_gain = state_cov * jacobian_h.transpose() * innovation_cov.inverse();

  // Pose Update
  pose_est += kalman_gain * innovation;

  // Update P
  state_cov -= kalman_gain * jacobian_h * state_cov;
}