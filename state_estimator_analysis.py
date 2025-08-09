#!/usr/bin/env python3
"""
State Estimator Analysis
Compares camera measurements with state estimation predictions
Processes data from trajectory logs and simulates camera measurements
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple, Optional
import pandas as pd
from scipy import stats

@dataclass
class Measurement:
    """Single measurement data"""
    timestamp: float
    camera_pose: Optional[np.ndarray]  # [x, y, theta]
    encoder_data: Optional[np.ndarray]  # wheel speeds
    gyro_data: Optional[float]         # angular velocity
    
class StateEstimator:
    """Python implementation of the C++ StateEstimator class"""
    
    def __init__(self):
        # State
        self.pose_est = np.zeros(3)  # [x, y, theta]
        self.initialized_pose = False
        self.pose_init = np.zeros(3)
        
        # Covariance matrices
        self.state_cov = np.eye(3) * 0.01  # P matrix
        self.process_cov = np.diag([0.001, 0.001, 0.01])  # Q matrix (process noise)
        self.camera_cov = np.diag([0.005, 0.005, 0.02])   # R matrix (camera measurement noise)
        
        # Timing
        self.t_last_encoder = None
        self.t_last_gyro = None
        self.initialized_encoder = False
        self.initialized_gyro = False
        
        # Robot model parameters (from C++ code)
        self.wheel_radius = 0.03  # meters
        self.robot_radius = 0.09  # meters
        
    def initialize_pose(self, pose_init: np.ndarray):
        """Initialize the pose"""
        self.pose_init = pose_init.copy()
        self.pose_est = pose_init.copy()
        self.initialized_pose = True
        
    def new_camera_data(self, pose_meas: np.ndarray, timestamp: float):
        """Process new camera measurement using Kalman filter update"""
        if not self.initialized_pose:
            self.initialize_pose(pose_meas)
            return
            
        # Kalman filter measurement update
        # Innovation (measurement residual)
        y = pose_meas - self.pose_est
        
        # Normalize angle difference
        y[2] = self._normalize_angle(y[2])
        
        # Innovation covariance
        S = self.state_cov + self.camera_cov
        
        # Kalman gain
        K = self.state_cov @ np.linalg.inv(S)
        
        # State update
        self.pose_est = self.pose_est + K @ y
        
        # Normalize angle
        self.pose_est[2] = self._normalize_angle(self.pose_est[2])
        
        # Covariance update
        self.state_cov = (np.eye(3) - K) @ self.state_cov
        
    def new_encoder_data(self, wheel_speeds_rpm: np.ndarray, timestamp: float):
        """Process encoder data (dead reckoning)"""
        if not self.initialized_pose:
            return
            
        if not self.initialized_encoder:
            self.t_last_encoder = timestamp
            self.initialized_encoder = True
            return
            
        dt = timestamp - self.t_last_encoder
        if dt <= 0:
            return
            
        # Convert RPM to rad/s
        wheel_speeds_radps = wheel_speeds_rpm * 2 * np.pi / 60
        
        # Mecanum wheel kinematics (simplified)
        # vx = (w1 + w2 + w3 + w4) * r / 4
        # vy = (-w1 + w2 + w3 - w4) * r / 4  
        # wz = (-w1 + w2 - w3 + w4) * r / (4 * R)
        
        vx_body = (wheel_speeds_radps[0] + wheel_speeds_radps[1] + 
                   wheel_speeds_radps[2] + wheel_speeds_radps[3]) * self.wheel_radius / 4
        vy_body = (-wheel_speeds_radps[0] + wheel_speeds_radps[1] + 
                   wheel_speeds_radps[2] - wheel_speeds_radps[3]) * self.wheel_radius / 4
        wz = (-wheel_speeds_radps[0] + wheel_speeds_radps[1] - 
              wheel_speeds_radps[2] + wheel_speeds_radps[3]) * self.wheel_radius / (4 * self.robot_radius)
        
        # Transform to world frame
        theta = self.pose_est[2]
        vx_world = vx_body * np.cos(theta) - vy_body * np.sin(theta)
        vy_world = vx_body * np.sin(theta) + vy_body * np.cos(theta)
        
        # Predict new pose
        self.pose_est[0] += vx_world * dt
        self.pose_est[1] += vy_world * dt
        self.pose_est[2] += wz * dt
        
        # Normalize angle
        self.pose_est[2] = self._normalize_angle(self.pose_est[2])
        
        # Update covariance (predict step)
        self.state_cov += self.process_cov * dt
        
        self.t_last_encoder = timestamp
        
    def new_gyro_data(self, w_radps: float, timestamp: float):
        """Process gyro data"""
        if not self.initialized_pose:
            return
            
        if not self.initialized_gyro:
            self.t_last_gyro = timestamp
            self.initialized_gyro = True
            return
            
        dt = timestamp - self.t_last_gyro
        if dt <= 0:
            return
            
        # Update orientation
        self.pose_est[2] += w_radps * dt
        self.pose_est[2] = self._normalize_angle(self.pose_est[2])
        
        # Update covariance
        self.state_cov[2, 2] += self.process_cov[2, 2] * dt
        
        self.t_last_gyro = timestamp
        
    def get_pose(self) -> np.ndarray:
        """Get current estimated pose"""
        return self.pose_est.copy()
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
        

def simulate_trajectory_with_camera(num_trajectories: int = 1) -> List[Measurement]:
    """
    Simulate a trajectory with camera measurements
    Based on the trajectory from trajectory_log.txt
    """
    measurements = []
    
    # Read trajectory data
    trajectory_data = []
    with open('trajectory_log.txt', 'r') as f:
        lines = f.readlines()
        data_started = False
        for line in lines:
            if line.startswith('# DATA_START'):
                data_started = True
                continue
            if data_started and not line.startswith('#'):
                parts = line.strip().split()
                if len(parts) >= 7:
                    trajectory_data.append([float(p) for p in parts])
    
    # Convert to numpy array
    trajectory_data = np.array(trajectory_data)
    
    # Process specified number of trajectories
    total_points = len(trajectory_data)
    points_per_trajectory = total_points // num_trajectories
    
    # Camera measurement frequency (every N samples)
    camera_frequency = 10  # Camera measurement every 10 encoder measurements
    
    # Add noise parameters
    camera_noise_std = np.array([0.005, 0.005, 0.02])  # meters, meters, radians
    
    for traj_idx in range(num_trajectories):
        start_idx = traj_idx * points_per_trajectory
        end_idx = min((traj_idx + 1) * points_per_trajectory, total_points)
        
        for i in range(start_idx, end_idx):
            timestamp = trajectory_data[i, 0]
            true_pose = trajectory_data[i, 1:4]  # x, y, theta
            velocities = trajectory_data[i, 4:7]  # vx, vy, omega
            
            # Create measurement
            measurement = Measurement(
                timestamp=timestamp,
                camera_pose=None,
                encoder_data=None,
                gyro_data=None
            )
            
            # Add camera measurement at regular intervals with noise
            if i % camera_frequency == 0:
                camera_pose = true_pose + np.random.normal(0, camera_noise_std)
                measurement.camera_pose = camera_pose
            
            # Simulate encoder data (wheel speeds from velocities)
            # Inverse kinematics: from body velocities to wheel speeds
            vx, vy, wz = velocities
            R = 0.09  # robot radius
            r = 0.03  # wheel radius
            
            # Mecanum wheel inverse kinematics
            w1 = (vx - vy - R * wz) / r
            w2 = (vx + vy + R * wz) / r
            w3 = (vx + vy - R * wz) / r
            w4 = (vx - vy + R * wz) / r
            
            # Convert to RPM
            wheel_speeds_rpm = np.array([w1, w2, w3, w4]) * 60 / (2 * np.pi)
            measurement.encoder_data = wheel_speeds_rpm
            
            # Gyro data
            measurement.gyro_data = wz
            
            measurements.append(measurement)
    
    return measurements


def analyze_estimation_error(measurements: List[Measurement]) -> dict:
    """
    Run state estimator with camera-only updates and analyze error
    """
    estimator = StateEstimator()
    
    # Storage for results
    timestamps = []
    camera_poses = []
    estimated_poses = []
    errors = []
    
    for measurement in measurements:
        # Process only camera measurements
        if measurement.camera_pose is not None:
            estimator.new_camera_data(measurement.camera_pose, measurement.timestamp)
            
            # Store data
            timestamps.append(measurement.timestamp)
            camera_poses.append(measurement.camera_pose)
            estimated_poses.append(estimator.get_pose())
            
            # Calculate error
            error = measurement.camera_pose - estimator.get_pose()
            error[2] = estimator._normalize_angle(error[2])  # Normalize angle error
            errors.append(error)
    
    # Convert to numpy arrays
    timestamps = np.array(timestamps)
    camera_poses = np.array(camera_poses)
    estimated_poses = np.array(estimated_poses)
    errors = np.array(errors)
    
    # Calculate statistics
    stats_dict = {
        'timestamps': timestamps,
        'camera_poses': camera_poses,
        'estimated_poses': estimated_poses,
        'errors': errors,
        'x_error_mean': np.mean(errors[:, 0]),
        'x_error_std': np.std(errors[:, 0]),
        'y_error_mean': np.mean(errors[:, 1]),
        'y_error_std': np.std(errors[:, 1]),
        'theta_error_mean': np.mean(errors[:, 2]),
        'theta_error_std': np.std(errors[:, 2]),
        'x_error_rmse': np.sqrt(np.mean(errors[:, 0]**2)),
        'y_error_rmse': np.sqrt(np.mean(errors[:, 1]**2)),
        'theta_error_rmse': np.sqrt(np.mean(errors[:, 2]**2))
    }
    
    return stats_dict


def plot_results(stats: dict):
    """
    Create comprehensive plot showing errors and statistics
    """
    fig = plt.figure(figsize=(15, 10))
    
    # Create subplots
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    # Plot 1: X Error over time
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(stats['timestamps'], stats['errors'][:, 0] * 1000, 'b-', alpha=0.7)
    ax1.axhline(y=stats['x_error_mean'] * 1000, color='r', linestyle='--', label=f"Mean: {stats['x_error_mean']*1000:.2f} mm")
    ax1.fill_between(stats['timestamps'], 
                     (stats['x_error_mean'] - stats['x_error_std']) * 1000,
                     (stats['x_error_mean'] + stats['x_error_std']) * 1000,
                     alpha=0.3, color='r')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('X Error (mm)')
    ax1.set_title('X Position Error')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Y Error over time
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(stats['timestamps'], stats['errors'][:, 1] * 1000, 'g-', alpha=0.7)
    ax2.axhline(y=stats['y_error_mean'] * 1000, color='r', linestyle='--', label=f"Mean: {stats['y_error_mean']*1000:.2f} mm")
    ax2.fill_between(stats['timestamps'],
                     (stats['y_error_mean'] - stats['y_error_std']) * 1000,
                     (stats['y_error_mean'] + stats['y_error_std']) * 1000,
                     alpha=0.3, color='r')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y Error (mm)')
    ax2.set_title('Y Position Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Theta Error over time
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(stats['timestamps'], np.degrees(stats['errors'][:, 2]), 'r-', alpha=0.7)
    ax3.axhline(y=np.degrees(stats['theta_error_mean']), color='k', linestyle='--', 
                label=f"Mean: {np.degrees(stats['theta_error_mean']):.2f}°")
    ax3.fill_between(stats['timestamps'],
                     np.degrees(stats['theta_error_mean'] - stats['theta_error_std']),
                     np.degrees(stats['theta_error_mean'] + stats['theta_error_std']),
                     alpha=0.3, color='k')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Theta Error (degrees)')
    ax3.set_title('Orientation Error')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Trajectory comparison
    ax4 = fig.add_subplot(gs[1, :])
    ax4.plot(stats['camera_poses'][:, 0], stats['camera_poses'][:, 1], 'b-', label='Camera Measurements', alpha=0.7)
    ax4.plot(stats['estimated_poses'][:, 0], stats['estimated_poses'][:, 1], 'r--', label='State Estimation', alpha=0.7)
    ax4.scatter(stats['camera_poses'][0, 0], stats['camera_poses'][0, 1], c='g', s=100, marker='o', label='Start')
    ax4.scatter(stats['camera_poses'][-1, 0], stats['camera_poses'][-1, 1], c='r', s=100, marker='x', label='End')
    ax4.set_xlabel('X Position (m)')
    ax4.set_ylabel('Y Position (m)')
    ax4.set_title('Trajectory Comparison: Camera vs State Estimation')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    # Plot 5: Error histograms
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.hist(stats['errors'][:, 0] * 1000, bins=30, alpha=0.7, color='b', edgecolor='black')
    ax5.set_xlabel('X Error (mm)')
    ax5.set_ylabel('Frequency')
    ax5.set_title(f'X Error Distribution\nRMSE: {stats["x_error_rmse"]*1000:.2f} mm')
    ax5.grid(True, alpha=0.3)
    
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.hist(stats['errors'][:, 1] * 1000, bins=30, alpha=0.7, color='g', edgecolor='black')
    ax6.set_xlabel('Y Error (mm)')
    ax6.set_ylabel('Frequency')
    ax6.set_title(f'Y Error Distribution\nRMSE: {stats["y_error_rmse"]*1000:.2f} mm')
    ax6.grid(True, alpha=0.3)
    
    ax7 = fig.add_subplot(gs[2, 2])
    ax7.hist(np.degrees(stats['errors'][:, 2]), bins=30, alpha=0.7, color='r', edgecolor='black')
    ax7.set_xlabel('Theta Error (degrees)')
    ax7.set_ylabel('Frequency')
    ax7.set_title(f'Theta Error Distribution\nRMSE: {np.degrees(stats["theta_error_rmse"]):.2f}°')
    ax7.grid(True, alpha=0.3)
    
    plt.suptitle('State Estimator Analysis: Camera-Only Updates', fontsize=16, fontweight='bold')
    
    # Add statistics text box
    stats_text = f"""Statistics Summary:
X Error: μ={stats['x_error_mean']*1000:.2f}mm, σ={stats['x_error_std']*1000:.2f}mm, RMSE={stats['x_error_rmse']*1000:.2f}mm
Y Error: μ={stats['y_error_mean']*1000:.2f}mm, σ={stats['y_error_std']*1000:.2f}mm, RMSE={stats['y_error_rmse']*1000:.2f}mm
θ Error: μ={np.degrees(stats['theta_error_mean']):.2f}°, σ={np.degrees(stats['theta_error_std']):.2f}°, RMSE={np.degrees(stats['theta_error_rmse']):.2f}°"""
    
    fig.text(0.02, 0.02, stats_text, fontsize=10, 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.3))
    
    plt.tight_layout()
    plt.savefig('state_estimator_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    return fig


def main():
    """Main analysis function"""
    print("State Estimator Analysis - Camera Only Updates")
    print("=" * 50)
    
    # Number of trajectories to analyze (configurable)
    NUM_TRAJECTORIES = 1  # Change this to analyze more trajectories
    
    print(f"Analyzing {NUM_TRAJECTORIES} trajectory(ies)...")
    
    # Generate measurements with simulated camera data
    measurements = simulate_trajectory_with_camera(NUM_TRAJECTORIES)
    print(f"Generated {len(measurements)} measurements")
    
    # Count camera measurements
    camera_count = sum(1 for m in measurements if m.camera_pose is not None)
    print(f"Camera measurements: {camera_count}")
    
    # Run analysis
    stats = analyze_estimation_error(measurements)
    
    # Print results
    print("\nError Statistics:")
    print("-" * 30)
    print(f"X Error:")
    print(f"  Mean: {stats['x_error_mean']*1000:.3f} mm")
    print(f"  Std:  {stats['x_error_std']*1000:.3f} mm")
    print(f"  RMSE: {stats['x_error_rmse']*1000:.3f} mm")
    
    print(f"\nY Error:")
    print(f"  Mean: {stats['y_error_mean']*1000:.3f} mm")
    print(f"  Std:  {stats['y_error_std']*1000:.3f} mm")
    print(f"  RMSE: {stats['y_error_rmse']*1000:.3f} mm")
    
    print(f"\nTheta Error:")
    print(f"  Mean: {np.degrees(stats['theta_error_mean']):.3f} degrees")
    print(f"  Std:  {np.degrees(stats['theta_error_std']):.3f} degrees")
    print(f"  RMSE: {np.degrees(stats['theta_error_rmse']):.3f} degrees")
    
    # Plot results
    print("\nGenerating plots...")
    plot_results(stats)
    print("Plots saved to 'state_estimator_analysis.png'")
    
    return stats


if __name__ == "__main__":
    stats = main()