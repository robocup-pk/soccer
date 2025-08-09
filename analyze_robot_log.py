#!/usr/bin/env python3
"""
Analyze robot0_odometry.log to extract and plot odometry data
Looking for patterns that might indicate camera updates vs pure encoder estimates
"""

import numpy as np
import matplotlib.pyplot as plt
import re

def parse_robot_log(filename='robot0_odometry.log'):
    """Parse the robot log file to extract odometry data"""
    
    data = {
        'x': [],
        'y': [],
        'theta': [],
        'timestamp_idx': [],  # Use index as pseudo-timestamp
    }
    
    # Track when camera was initialized
    camera_init_idx = None
    
    with open(filename, 'r') as f:
        idx = 0
        for line in f:
            # Check for camera initialization
            if 'NewCameraData] Initialized pose:' in line:
                camera_init_idx = idx
                print(f"Camera initialized at line index {idx}")
            
            # Extract odometry data
            if '[RobotNode::PublishOdometry] Pose:' in line:
                # Parse the pose values - handle potential concatenated values
                # First try standard format
                match = re.search(r'Pose:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', line)
                if match:
                    try:
                        x_val = float(match.group(1))
                        y_val = float(match.group(2))
                        theta_val = float(match.group(3))
                        data['x'].append(x_val)
                        data['y'].append(y_val)
                        data['theta'].append(theta_val)
                        data['timestamp_idx'].append(idx)
                    except ValueError:
                        # Handle concatenated values - try to split them
                        pose_part = line.split('Pose:')[1].strip()
                        values = pose_part.split()
                        if len(values) >= 3:
                            try:
                                # Check if second value is concatenated
                                if len(values) == 2:
                                    # Might be all concatenated
                                    continue
                                data['x'].append(float(values[0]))
                                # Try to split concatenated y and theta
                                if '.' in values[1] and values[1].count('.') > 1:
                                    # Concatenated values - skip this line
                                    continue
                                data['y'].append(float(values[1]))
                                data['theta'].append(float(values[2]))
                                data['timestamp_idx'].append(idx)
                            except:
                                # Skip malformed lines
                                continue
            idx += 1
    
    # Convert to numpy arrays and ensure consistent length
    min_length = min(len(data['x']), len(data['y']), len(data['theta']), len(data['timestamp_idx']))
    for key in data:
        data[key] = np.array(data[key][:min_length])
    
    print(f"Data lengths - X: {len(data['x'])}, Y: {len(data['y'])}, Theta: {len(data['theta'])}")
    
    return data, camera_init_idx

def detect_camera_updates(data, threshold_position=0.005, threshold_angle=0.1):
    """
    Detect potential camera updates by looking for sudden jumps in position
    These would indicate Kalman filter corrections from camera measurements
    """
    
    camera_updates = []
    
    if len(data['x']) < 2:
        return camera_updates
    
    # Calculate differences between consecutive poses
    dx = np.diff(data['x'])
    dy = np.diff(data['y'])
    dtheta = np.diff(data['theta'])
    
    # Make sure all arrays have the same length
    min_len = min(len(dx), len(dy), len(dtheta))
    dx = dx[:min_len]
    dy = dy[:min_len]
    dtheta = dtheta[:min_len]
    
    # Calculate position change magnitude
    position_change = np.sqrt(dx**2 + dy**2)
    
    # Look for sudden large changes that might indicate camera corrections
    for i in range(1, len(position_change)):
        # Check if there's a sudden jump in position or angle
        if (position_change[i] > threshold_position or 
            abs(dtheta[i]) > threshold_angle):
            # Also check if the change pattern is different from neighbors
            if i > 1 and i < len(position_change) - 1:
                avg_neighbor_change = (position_change[i-1] + position_change[i+1]) / 2
                if position_change[i] > 2 * avg_neighbor_change:
                    camera_updates.append(i)
    
    return camera_updates

def calculate_velocities(data, dt=0.01):
    """Calculate velocities from position data"""
    
    velocities = {
        'vx': np.zeros(len(data['x'])),
        'vy': np.zeros(len(data['y'])),
        'omega': np.zeros(len(data['theta']))
    }
    
    if len(data['x']) > 1:
        # Use central differences where possible
        velocities['vx'][1:-1] = (data['x'][2:] - data['x'][:-2]) / (2 * dt)
        velocities['vy'][1:-1] = (data['y'][2:] - data['y'][:-2]) / (2 * dt)
        velocities['omega'][1:-1] = (data['theta'][2:] - data['theta'][:-2]) / (2 * dt)
        
        # Use forward/backward differences at boundaries
        velocities['vx'][0] = (data['x'][1] - data['x'][0]) / dt
        velocities['vy'][0] = (data['y'][1] - data['y'][0]) / dt
        velocities['omega'][0] = (data['theta'][1] - data['theta'][0]) / dt
        
        velocities['vx'][-1] = (data['x'][-1] - data['x'][-2]) / dt
        velocities['vy'][-1] = (data['y'][-1] - data['y'][-2]) / dt
        velocities['omega'][-1] = (data['theta'][-1] - data['theta'][-2]) / dt
    
    return velocities

def plot_analysis(data, camera_updates, camera_init_idx):
    """Create comprehensive analysis plots"""
    
    fig = plt.figure(figsize=(16, 12))
    
    # Convert to mm for better readability
    x_mm = data['x'] * 1000
    y_mm = data['y'] * 1000
    theta_deg = np.degrees(data['theta'])
    
    # Time axis (using index as proxy for time)
    time = np.arange(len(data['x']))
    
    # 1. XY Trajectory
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(x_mm, y_mm, 'b-', linewidth=1, alpha=0.7, label='Trajectory')
    
    # Mark potential camera updates
    if camera_updates:
        ax1.scatter(x_mm[camera_updates], y_mm[camera_updates], 
                   c='red', s=50, marker='o', alpha=0.7, label='Possible Camera Updates')
    
    # Mark start and end
    ax1.scatter(x_mm[0], y_mm[0], c='green', s=100, marker='o', label='Start')
    ax1.scatter(x_mm[-1], y_mm[-1], c='red', s=100, marker='s', label='End')
    
    ax1.set_xlabel('X Position (mm)')
    ax1.set_ylabel('Y Position (mm)')
    ax1.set_title('Robot Trajectory (Kalman Filter Output)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. X Position over time
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(time, x_mm, 'b-', linewidth=1)
    if camera_updates:
        ax2.scatter(time[camera_updates], x_mm[camera_updates], 
                   c='red', s=30, marker='o', alpha=0.7)
    ax2.set_xlabel('Sample Index')
    ax2.set_ylabel('X Position (mm)')
    ax2.set_title('X Position Over Time')
    ax2.grid(True, alpha=0.3)
    
    # 3. Y Position over time
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(time, y_mm, 'g-', linewidth=1)
    if camera_updates:
        ax3.scatter(time[camera_updates], y_mm[camera_updates], 
                   c='red', s=30, marker='o', alpha=0.7)
    ax3.set_xlabel('Sample Index')
    ax3.set_ylabel('Y Position (mm)')
    ax3.set_title('Y Position Over Time')
    ax3.grid(True, alpha=0.3)
    
    # 4. Theta over time
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(time, theta_deg, 'r-', linewidth=1)
    if camera_updates:
        ax4.scatter(time[camera_updates], theta_deg[camera_updates], 
                   c='red', s=30, marker='o', alpha=0.7)
    ax4.set_xlabel('Sample Index')
    ax4.set_ylabel('Theta (degrees)')
    ax4.set_title('Orientation Over Time')
    ax4.grid(True, alpha=0.3)
    
    # 5. Position change magnitude (to detect jumps)
    ax5 = plt.subplot(3, 3, 5)
    if len(x_mm) > 1:
        dx = np.diff(x_mm)
        dy = np.diff(y_mm)
        position_change = np.sqrt(dx**2 + dy**2)
        ax5.plot(time[1:], position_change, 'b-', linewidth=1, alpha=0.7)
        ax5.axhline(y=np.mean(position_change), color='r', linestyle='--', 
                   label=f'Mean: {np.mean(position_change):.3f}mm')
        ax5.set_xlabel('Sample Index')
        ax5.set_ylabel('Position Change (mm)')
        ax5.set_title('Position Change Between Samples')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
    
    # 6. Angle change (to detect jumps)
    ax6 = plt.subplot(3, 3, 6)
    if len(theta_deg) > 1:
        dtheta = np.diff(theta_deg)
        ax6.plot(time[1:], dtheta, 'r-', linewidth=1, alpha=0.7)
        ax6.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax6.set_xlabel('Sample Index')
        ax6.set_ylabel('Angle Change (degrees)')
        ax6.set_title('Angle Change Between Samples')
        ax6.grid(True, alpha=0.3)
    
    # 7. Velocities
    ax7 = plt.subplot(3, 3, 7)
    velocities = calculate_velocities(data)
    ax7.plot(time, velocities['vx'] * 1000, 'b-', label='Vx', alpha=0.7)
    ax7.plot(time, velocities['vy'] * 1000, 'g-', label='Vy', alpha=0.7)
    ax7.set_xlabel('Sample Index')
    ax7.set_ylabel('Velocity (mm/s)')
    ax7.set_title('Linear Velocities')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # 8. Angular velocity
    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(time, np.degrees(velocities['omega']), 'r-', linewidth=1, alpha=0.7)
    ax8.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax8.set_xlabel('Sample Index')
    ax8.set_ylabel('Angular Velocity (deg/s)')
    ax8.set_title('Angular Velocity')
    ax8.grid(True, alpha=0.3)
    
    # 9. Statistics
    ax9 = plt.subplot(3, 3, 9)
    ax9.axis('off')
    
    stats_text = f"""Data Statistics:
    
Total Samples: {len(data['x'])}
Possible Camera Updates: {len(camera_updates)}

Position Range:
  X: [{np.min(x_mm):.2f}, {np.max(x_mm):.2f}] mm
  Y: [{np.min(y_mm):.2f}, {np.max(y_mm):.2f}] mm
  
Angle Range:
  Theta: [{np.min(theta_deg):.2f}, {np.max(theta_deg):.2f}] degrees

Mean Position:
  X: {np.mean(x_mm):.2f} mm
  Y: {np.mean(y_mm):.2f} mm
  Theta: {np.mean(theta_deg):.2f} degrees

Std Dev:
  X: {np.std(x_mm):.2f} mm
  Y: {np.std(y_mm):.2f} mm
  Theta: {np.std(theta_deg):.2f} degrees
"""
    
    ax9.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
            verticalalignment='center')
    
    plt.suptitle('Robot Odometry Analysis - Kalman Filter Estimates', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    return fig

def main():
    """Main analysis function"""
    
    print("=" * 60)
    print("Robot Odometry Log Analysis")
    print("=" * 60)
    
    # Parse the log file
    data, camera_init_idx = parse_robot_log('robot0_odometry.log')
    
    if len(data['x']) == 0:
        print("No odometry data found in log file!")
        return
    
    print(f"\nFound {len(data['x'])} odometry samples")
    
    # Detect potential camera updates
    camera_updates = detect_camera_updates(data)
    print(f"Detected {len(camera_updates)} potential camera updates")
    
    # Create plots
    fig = plot_analysis(data, camera_updates, camera_init_idx)
    
    # Save figure
    output_file = 'robot_odometry_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlots saved to '{output_file}'")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()