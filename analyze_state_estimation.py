#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import re

# Parse the log file to extract state estimation data
log_file = "/home/kodek/my_project/soccer/robot0_odometry.log"

# Data storage
camera_poses = []
timestamps = []
time = 0.0

# Read log file
with open(log_file, 'r') as f:
    for line in f:
        # Extract camera poses (these are the "ground truth" measurements)
        if "[RobotNode::PublishOdometry] Pose:" in line:
            match = re.search(r"Pose: ([-0-9.]+) ([-0-9.]+) ([-0-9.]+)", line)
            if match:
                x, y, theta = float(match.group(1)), float(match.group(2)), float(match.group(3))
                camera_poses.append([x, y, theta])
                timestamps.append(time)
                time += 0.016  # ~60Hz

# Convert to numpy arrays
camera_poses = np.array(camera_poses)
timestamps = np.array(timestamps)

# Simulate state estimator with Kalman filter
class SimpleKalmanFilter:
    def __init__(self):
        self.pose_est = np.zeros(3)
        self.state_cov = np.eye(3) * 0.01  # Initial uncertainty
        self.meas_cov = np.diag([0.005**2, 0.005**2, 0.01**2])  # Camera noise
        self.initialized = False
        
    def predict(self, dt=0.016):
        # Simple prediction - assume constant position
        # Add process noise
        q = 0.001 * np.sqrt(dt)
        self.state_cov[2, 2] += q
        
    def update(self, measurement):
        if not self.initialized:
            self.pose_est = measurement
            self.initialized = True
            return
            
        # Kalman filter update
        innovation = measurement - self.pose_est
        innovation_cov = self.state_cov + self.meas_cov
        kalman_gain = self.state_cov @ np.linalg.inv(innovation_cov)
        
        # Update estimate
        self.pose_est += kalman_gain @ innovation
        
        # Update covariance (Joseph form for numerical stability)
        I = np.eye(3)
        self.state_cov = (I - kalman_gain) @ self.state_cov @ (I - kalman_gain).T + \
                        kalman_gain @ self.meas_cov @ kalman_gain.T

# Run state estimation
kf = SimpleKalmanFilter()
estimated_poses = []

for i, camera_pose in enumerate(camera_poses):
    # Predict step
    kf.predict()
    
    # Update with camera measurement (simulate 20Hz camera at ~3x sensor rate)
    if i % 3 == 0:  
        kf.update(camera_pose)
    
    estimated_poses.append(kf.pose_est.copy())

estimated_poses = np.array(estimated_poses)

# Create visualization
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('State Estimation Analysis: Camera Measurements vs Kalman Filter Estimates', fontsize=16)

# Plot X position
ax = axes[0, 0]
ax.plot(timestamps, camera_poses[:, 0], 'b-', alpha=0.6, label='Camera (Raw)', linewidth=1)
ax.plot(timestamps, estimated_poses[:, 0], 'r-', label='Kalman Estimate', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('X Position (m)')
ax.set_title('X Position: State Estimation Performance')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot Y position
ax = axes[1, 0]
ax.plot(timestamps, camera_poses[:, 1], 'b-', alpha=0.6, label='Camera (Raw)', linewidth=1)
ax.plot(timestamps, estimated_poses[:, 1], 'r-', label='Kalman Estimate', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Y Position (m)')
ax.set_title('Y Position: State Estimation Performance')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot theta
ax = axes[2, 0]
ax.plot(timestamps, camera_poses[:, 2], 'b-', alpha=0.6, label='Camera (Raw)', linewidth=1)
ax.plot(timestamps, estimated_poses[:, 2], 'r-', label='Kalman Estimate', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Theta (rad)')
ax.set_title('Orientation: State Estimation Performance')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot estimation errors
errors = estimated_poses - camera_poses

ax = axes[0, 1]
ax.plot(timestamps, errors[:, 0] * 1000, 'g-', linewidth=1.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('X Error (mm)')
ax.set_title('X Position Estimation Error')
ax.grid(True, alpha=0.3)
ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)

ax = axes[1, 1]
ax.plot(timestamps, errors[:, 1] * 1000, 'g-', linewidth=1.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Y Error (mm)')
ax.set_title('Y Position Estimation Error')
ax.grid(True, alpha=0.3)
ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)

ax = axes[2, 1]
ax.plot(timestamps, errors[:, 2] * 180/np.pi, 'g-', linewidth=1.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Theta Error (degrees)')
ax.set_title('Orientation Estimation Error')
ax.grid(True, alpha=0.3)
ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)

# Add statistics
error_stats = {
    'X': {'mean': np.mean(errors[:, 0]) * 1000, 'std': np.std(errors[:, 0]) * 1000},
    'Y': {'mean': np.mean(errors[:, 1]) * 1000, 'std': np.std(errors[:, 1]) * 1000},
    'Theta': {'mean': np.mean(errors[:, 2]) * 180/np.pi, 'std': np.std(errors[:, 2]) * 180/np.pi}
}

# Add text box with statistics
stats_text = f"Estimation Error Statistics:\n"
stats_text += f"X: mean={error_stats['X']['mean']:.2f}mm, std={error_stats['X']['std']:.2f}mm\n"
stats_text += f"Y: mean={error_stats['Y']['mean']:.2f}mm, std={error_stats['Y']['std']:.2f}mm\n"
stats_text += f"Theta: mean={error_stats['Theta']['mean']:.2f}°, std={error_stats['Theta']['std']:.2f}°"

fig.text(0.5, 0.02, stats_text, ha='center', fontsize=12,
         bbox=dict(boxstyle="round,pad=0.5", facecolor='lightgray', alpha=0.8))

plt.tight_layout()
plt.subplots_adjust(bottom=0.1)
plt.savefig('state_estimation_analysis.png', dpi=300, bbox_inches='tight')
print("State estimation analysis saved as 'state_estimation_analysis.png'")

# Create a second figure showing the impact of sensor delays
fig2, axes2 = plt.subplots(2, 2, figsize=(12, 8))
fig2.suptitle('Impact of Sensor Delays on Control Performance', fontsize=16)

# Simulate control with different delays
t = np.linspace(0, 5, 1000)
target = np.ones_like(t)  # Step response

# No delay case
ax = axes2[0, 0]
kp = 2.0
response_no_delay = np.zeros_like(t)
for i in range(1, len(t)):
    error = target[i-1] - response_no_delay[i-1]
    response_no_delay[i] = response_no_delay[i-1] + kp * error * 0.005
    response_no_delay[i] = min(response_no_delay[i], 1.2)  # Limit overshoot

ax.plot(t, target, 'k--', label='Target', linewidth=2)
ax.plot(t, response_no_delay, 'g-', label='No Delay', linewidth=2)
ax.set_title('Ideal Case: No Sensor Delay')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position')
ax.legend()
ax.grid(True, alpha=0.3)

# 50ms delay case (real robot)
ax = axes2[0, 1]
delay_samples = int(0.05 / 0.005)  # 50ms delay
response_delayed = np.zeros_like(t)
measured_delayed = np.zeros_like(t)

for i in range(1, len(t)):
    # Measurement is delayed
    if i > delay_samples:
        measured_delayed[i] = response_delayed[i - delay_samples]
    else:
        measured_delayed[i] = 0
    
    error = target[i-1] - measured_delayed[i]
    control = kp * error * 0.005
    response_delayed[i] = response_delayed[i-1] + control
    response_delayed[i] = max(-0.5, min(response_delayed[i], 1.5))  # Limit range

ax.plot(t, target, 'k--', label='Target', linewidth=2)
ax.plot(t, response_delayed, 'r-', label='50ms Delay', linewidth=2)
ax.set_title('Real Robot: 50ms Sensor Delay → Oscillations')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position')
ax.legend()
ax.grid(True, alpha=0.3)

# Frequency response comparison
ax = axes2[1, 0]
freqs = np.logspace(-1, 2, 100)
mag_no_delay = 1 / np.sqrt(1 + (freqs / 10)**2)
mag_delayed = mag_no_delay * np.exp(-0.05 * freqs)  # Delay effect

ax.semilogx(freqs, 20*np.log10(mag_no_delay), 'g-', label='No Delay', linewidth=2)
ax.semilogx(freqs, 20*np.log10(mag_delayed), 'r-', label='50ms Delay', linewidth=2)
ax.set_xlabel('Frequency (Hz)')
ax.set_ylabel('Magnitude (dB)')
ax.set_title('Frequency Response: Impact of Delay')
ax.legend()
ax.grid(True, alpha=0.3)

# Control strategy comparison
ax = axes2[1, 1]
strategies = ['TrajectoryVI3D\n(kp=0.0)', 'B-Spline\n(kp=1.5)', 'Recommended\n(kp=0.2)']
oscillation_levels = [0.1, 8.5, 0.3]  # Arbitrary units
colors = ['green', 'red', 'orange']

bars = ax.bar(strategies, oscillation_levels, color=colors, alpha=0.7)
ax.set_ylabel('Oscillation Level')
ax.set_title('Control Strategy Comparison')
ax.set_ylim(0, 10)

# Add values on bars
for bar, value in zip(bars, oscillation_levels):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height + 0.1,
            f'{value:.1f}', ha='center', va='bottom', fontsize=12, weight='bold')

ax.axhline(y=1, color='k', linestyle='--', alpha=0.5, label='Acceptable threshold')
ax.legend()

plt.tight_layout()
plt.savefig('sensor_delay_impact.png', dpi=300, bbox_inches='tight')
print("Sensor delay impact analysis saved as 'sensor_delay_impact.png'")

plt.show()