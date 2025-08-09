#!/usr/bin/env python3
"""
Analyze robot0_odometry.log to extract and plot both raw measurements and Kalman filter estimates
This script attempts to identify camera measurement events and track the Kalman filter corrections
"""

import numpy as np
import matplotlib.pyplot as plt
import re
from scipy import signal

def parse_robot_log_detailed(filename='robot0_odometry.log'):
    """Parse the robot log file to extract all relevant data"""
    
    kalman_data = {
        'x': [],
        'y': [],
        'theta': [],
        'timestamp_idx': [],
    }
    
    # Track camera and encoder events
    events = {
        'camera_init': None,
        'camera_updates': [],
        'encoder_updates': [],
    }
    
    # Parse log file
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    for idx, line in enumerate(lines):
        # Track camera initialization
        if 'NewCameraData] Initialized pose:' in line:
            events['camera_init'] = idx
            match = re.search(r'pose:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', line)
            if match:
                print(f"Camera initialized at line {idx} with pose: {match.groups()}")
        
        # Track camera updates (after initialization)
        elif 'NewCameraData]' in line and events['camera_init']:
            events['camera_updates'].append(idx)
        
        # Track encoder updates
        elif 'NewEncoderData]' in line or 'NewMotorsData]' in line:
            events['encoder_updates'].append(idx)
        
        # Extract Kalman filter output (PublishOdometry)
        elif '[RobotNode::PublishOdometry] Pose:' in line:
            match = re.search(r'Pose:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', line)
            if match:
                try:
                    kalman_data['x'].append(float(match.group(1)))
                    kalman_data['y'].append(float(match.group(2)))
                    kalman_data['theta'].append(float(match.group(3)))
                    kalman_data['timestamp_idx'].append(idx)
                except ValueError:
                    continue
    
    # Convert to numpy arrays and ensure consistent length
    min_len = min(len(kalman_data[k]) for k in kalman_data if len(kalman_data[k]) > 0)
    for key in kalman_data:
        kalman_data[key] = np.array(kalman_data[key][:min_len])
    
    print(f"Found {len(kalman_data['x'])} Kalman filter outputs")
    print(f"Found {len(events['camera_updates'])} camera updates after initialization")
    print(f"Found {len(events['encoder_updates'])} encoder updates")
    
    return kalman_data, events

def analyze_trajectory_segments(kalman_data, events):
    """Analyze trajectory to identify different motion segments"""
    
    if len(kalman_data['x']) < 10:
        return []
    
    # Calculate velocity to identify motion segments
    dt = 0.01  # Approximate sampling time
    vx = np.gradient(kalman_data['x']) / dt
    vy = np.gradient(kalman_data['y']) / dt
    speed = np.sqrt(vx**2 + vy**2)
    
    # Find segments where robot is moving vs stationary
    moving_threshold = 0.01  # 10mm/s
    is_moving = speed > moving_threshold
    
    # Find transitions
    transitions = np.where(np.diff(is_moving.astype(int)))[0]
    
    segments = []
    start_idx = 0
    for trans_idx in transitions:
        segments.append({
            'start': start_idx,
            'end': trans_idx,
            'is_moving': is_moving[start_idx]
        })
        start_idx = trans_idx + 1
    
    # Add final segment
    if start_idx < len(kalman_data['x']):
        segments.append({
            'start': start_idx,
            'end': len(kalman_data['x']) - 1,
            'is_moving': is_moving[start_idx] if start_idx < len(is_moving) else False
        })
    
    return segments

def detect_kalman_corrections(kalman_data, window_size=5):
    """
    Detect Kalman filter corrections by looking for discontinuities in the trajectory
    These would indicate when camera measurements correct the encoder-based predictions
    """
    
    corrections = []
    
    if len(kalman_data['x']) < window_size * 2:
        return corrections
    
    # Calculate position changes
    dx = np.diff(kalman_data['x'])
    dy = np.diff(kalman_data['y'])
    dtheta = np.diff(kalman_data['theta'])
    
    # Calculate statistics in sliding windows
    for i in range(window_size, len(dx) - window_size):
        # Local statistics
        local_dx = dx[i-window_size:i+window_size]
        local_dy = dy[i-window_size:i+window_size]
        
        # Check for outliers (potential corrections)
        dx_mean = np.mean(local_dx)
        dx_std = np.std(local_dx)
        dy_mean = np.mean(local_dy)
        dy_std = np.std(local_dy)
        
        # If current change is significantly different from local average
        if (abs(dx[i] - dx_mean) > 2 * dx_std and dx_std > 0) or \
           (abs(dy[i] - dy_mean) > 2 * dy_std and dy_std > 0):
            corrections.append(i)
    
    return corrections

def plot_kalman_analysis(kalman_data, events, corrections):
    """Create comprehensive plots showing Kalman filter behavior"""
    
    fig = plt.figure(figsize=(18, 12))
    
    # Convert to mm for better readability
    x_mm = kalman_data['x'] * 1000
    y_mm = kalman_data['y'] * 1000
    theta_deg = np.degrees(kalman_data['theta'])
    time_idx = np.arange(len(x_mm))
    
    # 1. Main trajectory plot
    ax1 = plt.subplot(3, 4, (1, 5))
    
    # Plot the Kalman filter trajectory
    ax1.plot(x_mm, y_mm, 'b-', linewidth=1.5, alpha=0.8, label='Kalman Filter Output', zorder=2)
    
    # Mark detected corrections
    if corrections:
        ax1.scatter(x_mm[corrections], y_mm[corrections], 
                   c='red', s=30, marker='x', alpha=0.7, 
                   label=f'Detected Corrections ({len(corrections)})', zorder=3)
    
    # Mark start and end
    ax1.scatter(x_mm[0], y_mm[0], c='green', s=100, marker='o', 
               label='Start', zorder=4, edgecolors='black')
    ax1.scatter(x_mm[-1], y_mm[-1], c='red', s=100, marker='s', 
               label='End', zorder=4, edgecolors='black')
    
    ax1.set_xlabel('X Position (mm)', fontsize=11)
    ax1.set_ylabel('Y Position (mm)', fontsize=11)
    ax1.set_title('Kalman Filter Trajectory\n(Camera + Encoder Fusion)', fontsize=12, fontweight='bold')
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. X position over time
    ax2 = plt.subplot(3, 4, 2)
    ax2.plot(time_idx, x_mm, 'b-', linewidth=1, label='Kalman X')
    if corrections:
        ax2.scatter(time_idx[corrections], x_mm[corrections], 
                   c='red', s=20, marker='x', alpha=0.6)
    ax2.set_xlabel('Sample Index')
    ax2.set_ylabel('X Position (mm)')
    ax2.set_title('X Position Timeline')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    
    # 3. Y position over time
    ax3 = plt.subplot(3, 4, 3)
    ax3.plot(time_idx, y_mm, 'g-', linewidth=1, label='Kalman Y')
    if corrections:
        ax3.scatter(time_idx[corrections], y_mm[corrections], 
                   c='red', s=20, marker='x', alpha=0.6)
    ax3.set_xlabel('Sample Index')
    ax3.set_ylabel('Y Position (mm)')
    ax3.set_title('Y Position Timeline')
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8)
    
    # 4. Theta over time
    ax4 = plt.subplot(3, 4, 4)
    ax4.plot(time_idx, theta_deg, 'r-', linewidth=1, label='Kalman θ')
    if corrections:
        ax4.scatter(time_idx[corrections], theta_deg[corrections], 
                   c='red', s=20, marker='x', alpha=0.6)
    ax4.set_xlabel('Sample Index')
    ax4.set_ylabel('Theta (degrees)')
    ax4.set_title('Orientation Timeline')
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8)
    
    # 5. Position change magnitude (innovation)
    ax5 = plt.subplot(3, 4, 6)
    if len(x_mm) > 1:
        dx = np.diff(x_mm)
        dy = np.diff(y_mm)
        position_change = np.sqrt(dx**2 + dy**2)
        ax5.plot(time_idx[1:], position_change, 'b-', linewidth=0.8, alpha=0.7)
        
        # Mark large changes
        threshold = np.percentile(position_change, 95)
        large_changes = position_change > threshold
        ax5.scatter(time_idx[1:][large_changes], position_change[large_changes], 
                   c='red', s=20, marker='o', alpha=0.6, label='Large changes')
        
        ax5.axhline(y=np.mean(position_change), color='g', linestyle='--', 
                   alpha=0.5, label=f'Mean: {np.mean(position_change):.2f}mm')
        ax5.set_xlabel('Sample Index')
        ax5.set_ylabel('Position Change (mm)')
        ax5.set_title('Inter-sample Position Changes')
        ax5.legend(fontsize=8)
        ax5.grid(True, alpha=0.3)
    
    # 6. Angle change
    ax6 = plt.subplot(3, 4, 7)
    if len(theta_deg) > 1:
        dtheta = np.diff(theta_deg)
        ax6.plot(time_idx[1:], dtheta, 'r-', linewidth=0.8, alpha=0.7)
        ax6.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax6.set_xlabel('Sample Index')
        ax6.set_ylabel('Angle Change (deg)')
        ax6.set_title('Inter-sample Angle Changes')
        ax6.grid(True, alpha=0.3)
    
    # 7. Velocity profile
    ax7 = plt.subplot(3, 4, 8)
    if len(x_mm) > 2:
        dt = 0.01  # Approximate sample time
        vx = np.gradient(x_mm) / (dt * 1000)  # Convert to m/s
        vy = np.gradient(y_mm) / (dt * 1000)
        speed = np.sqrt(vx**2 + vy**2)
        
        ax7.plot(time_idx, speed, 'b-', linewidth=1, alpha=0.8, label='Speed')
        ax7.fill_between(time_idx, 0, speed, alpha=0.3)
        ax7.set_xlabel('Sample Index')
        ax7.set_ylabel('Speed (m/s)')
        ax7.set_title('Speed Profile')
        ax7.legend(fontsize=8)
        ax7.grid(True, alpha=0.3)
    
    # 8. Angular velocity
    ax8 = plt.subplot(3, 4, 9)
    if len(theta_deg) > 2:
        omega = np.gradient(theta_deg) / (dt)  # deg/s
        ax8.plot(time_idx, omega, 'r-', linewidth=1, alpha=0.8)
        ax8.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax8.set_xlabel('Sample Index')
        ax8.set_ylabel('Angular Velocity (deg/s)')
        ax8.set_title('Angular Velocity Profile')
        ax8.grid(True, alpha=0.3)
    
    # 9. Trajectory curvature
    ax9 = plt.subplot(3, 4, 10)
    if len(x_mm) > 5:
        # Calculate curvature using finite differences
        dx = np.gradient(x_mm)
        dy = np.gradient(y_mm)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-6)**1.5
        # Smooth the curvature
        curvature_smooth = signal.savgol_filter(curvature, window_length=min(51, len(curvature)//2*2-1), polyorder=3)
        
        ax9.plot(time_idx, curvature_smooth, 'g-', linewidth=1, alpha=0.8)
        ax9.set_xlabel('Sample Index')
        ax9.set_ylabel('Curvature (1/mm)')
        ax9.set_title('Trajectory Curvature')
        ax9.grid(True, alpha=0.3)
    
    # 10. Position histogram
    ax10 = plt.subplot(3, 4, 11)
    ax10.hist2d(x_mm, y_mm, bins=50, cmap='Blues')
    ax10.set_xlabel('X Position (mm)')
    ax10.set_ylabel('Y Position (mm)')
    ax10.set_title('Position Density')
    ax10.axis('equal')
    
    # 11. Statistics panel
    ax11 = plt.subplot(3, 4, 12)
    ax11.axis('off')
    
    # Calculate statistics
    total_distance = np.sum(np.sqrt(np.diff(x_mm)**2 + np.diff(y_mm)**2)) / 1000  # meters
    total_rotation = np.sum(np.abs(np.diff(theta_deg)))  # degrees
    
    stats_text = f"""Kalman Filter Statistics:
    
Total Samples: {len(kalman_data['x'])}
Detected Corrections: {len(corrections)}

Position Range:
  X: [{np.min(x_mm):.1f}, {np.max(x_mm):.1f}] mm
  Y: [{np.min(y_mm):.1f}, {np.max(y_mm):.1f}] mm
  θ: [{np.min(theta_deg):.1f}, {np.max(theta_deg):.1f}] deg

Total Distance: {total_distance:.3f} m
Total Rotation: {total_rotation:.1f} deg

Mean Position:
  X: {np.mean(x_mm):.2f} ± {np.std(x_mm):.2f} mm
  Y: {np.mean(y_mm):.2f} ± {np.std(y_mm):.2f} mm
  θ: {np.mean(theta_deg):.2f} ± {np.std(theta_deg):.2f} deg

Camera Events:
  Initialization: Line {events['camera_init']}
  Updates: {len(events['camera_updates'])}
  
Encoder Updates: {len(events['encoder_updates'])}
"""
    
    ax11.text(0.05, 0.95, stats_text, fontsize=9, family='monospace',
             verticalalignment='top', transform=ax11.transAxes)
    
    plt.suptitle('Robot Kalman Filter Analysis - State Estimation with Camera & Encoder Fusion', 
                 fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    return fig

def main():
    """Main analysis function"""
    
    print("=" * 60)
    print("Robot Kalman Filter Analysis")
    print("Analyzing State Estimation Performance")
    print("=" * 60)
    
    # Parse the log file
    kalman_data, events = parse_robot_log_detailed('robot0_odometry.log')
    
    if len(kalman_data['x']) == 0:
        print("No Kalman filter data found in log file!")
        return
    
    # Detect Kalman corrections
    corrections = detect_kalman_corrections(kalman_data)
    print(f"\nDetected {len(corrections)} potential Kalman corrections")
    
    # Analyze trajectory segments
    segments = analyze_trajectory_segments(kalman_data, events)
    print(f"Found {len(segments)} trajectory segments")
    moving_segments = sum(1 for s in segments if s['is_moving'])
    print(f"  - Moving segments: {moving_segments}")
    print(f"  - Stationary segments: {len(segments) - moving_segments}")
    
    # Create plots
    fig = plot_kalman_analysis(kalman_data, events, corrections)
    
    # Save figure
    output_file = 'robot_kalman_filter_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlots saved to '{output_file}'")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()