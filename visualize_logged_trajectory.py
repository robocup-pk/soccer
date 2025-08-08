#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.animation import FuncAnimation
import sys
import os

def read_trajectory_log(filename):
    """Read trajectory data from log file"""
    waypoints = []
    trajectory_data = []
    trajectory_type = 2
    
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found!")
        print("Please run the demo_with_logging first to generate the trajectory log.")
        return None, None, None
    
    with open(filename, 'r') as f:
        data_started = False
        for line in f:
            line = line.strip()
            if line.startswith('#'):
                if line.startswith('# WP'):
                    parts = line.split()
                    if len(parts) >= 5:
                        idx = int(parts[2])
                        x = float(parts[3])
                        y = float(parts[4])
                        theta = float(parts[5])
                        waypoints.append([x, y, theta])
                elif line.startswith('# TRAJECTORY_TYPE'):
                    parts = line.split()
                    if len(parts) >= 3:
                        trajectory_type = int(parts[2])
                elif line == '# DATA_START':
                    data_started = True
            elif data_started and line:
                parts = line.split()
                if len(parts) >= 7:
                    timestamp = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    theta = float(parts[3])
                    vx = float(parts[4])
                    vy = float(parts[5])
                    omega = float(parts[6])
                    trajectory_data.append([timestamp, x, y, theta, vx, vy, omega])
    
    return np.array(waypoints), np.array(trajectory_data), trajectory_type

def generate_ideal_bspline(waypoints, num_samples=500):
    """Generate ideal B-spline path for comparison"""
    if len(waypoints) < 2:
        return np.array([])
    
    # For square path, create smoother corners
    points = []
    corner_radius = 0.1  # Radius for corner smoothing
    
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i, :2]
        p2 = waypoints[i+1, :2]
        
        # Check if this is a corner (significant angle change)
        if i > 0 and i < len(waypoints) - 1:
            v1 = p1 - waypoints[i-1, :2]
            v2 = p2 - p1
            
            # Normalize vectors
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                v1_norm = v1 / np.linalg.norm(v1)
                v2_norm = v2 / np.linalg.norm(v2)
                
                # Calculate angle between vectors
                dot_product = np.clip(np.dot(v1_norm, v2_norm), -1, 1)
                angle = np.arccos(dot_product)
                
                # If it's a significant corner (> 30 degrees)
                if angle > np.pi/6:
                    # Add points before corner
                    corner_start = p1 - v1_norm * corner_radius
                    for t in np.linspace(0, 1, 10):
                        point = waypoints[i-1, :2] * (1-t) + corner_start * t
                        points.append(point)
                    
                    # Add corner arc
                    corner_end = p1 + v2_norm * corner_radius
                    # Simple arc approximation
                    for t in np.linspace(0, 1, 20):
                        # Bezier curve for corner
                        point = (1-t)**2 * corner_start + 2*(1-t)*t * p1 + t**2 * corner_end
                        points.append(point)
                    
                    continue
        
        # Add straight line segments
        for t in np.linspace(0, 1, 30):
            point = p1 * (1 - t) + p2 * t
            points.append(point)
    
    return np.array(points) if points else waypoints[:, :2]

def calculate_errors(trajectory_data, ideal_path):
    """Calculate cross-track errors"""
    errors = []
    
    for i in range(len(trajectory_data)):
        robot_pos = trajectory_data[i, 1:3]  # x, y position
        
        # Find closest point on ideal path
        min_dist = float('inf')
        for j in range(len(ideal_path) - 1):
            p1 = ideal_path[j]
            p2 = ideal_path[j + 1]
            
            # Vector from p1 to p2
            v = p2 - p1
            v_norm = np.linalg.norm(v)
            if v_norm < 1e-10:
                continue
            
            # Project robot position onto line segment
            t = max(0, min(1, np.dot(robot_pos - p1, v) / (v_norm * v_norm)))
            closest = p1 + t * v
            
            dist = np.linalg.norm(robot_pos - closest)
            min_dist = min(min_dist, dist)
        
        errors.append(min_dist)
    
    return np.array(errors)

def plot_trajectory_analysis(waypoints, trajectory_data, trajectory_type):
    """Create comprehensive trajectory analysis plot"""
    
    # Generate ideal path
    ideal_path = generate_ideal_bspline(waypoints)
    
    # Calculate errors
    errors = calculate_errors(trajectory_data, ideal_path)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Main trajectory plot
    ax1 = plt.subplot(2, 3, (1, 4))
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    
    traj_type_names = {1: "B-spline", 2: "Uniform B-spline", 3: "Bezier"}
    title = f'Robot Trajectory Analysis - {traj_type_names.get(trajectory_type, "Unknown")}'
    ax1.set_title(title, fontsize=14, fontweight='bold')
    
    # Plot waypoints
    if len(waypoints) > 0:
        ax1.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=100, 
                   marker='o', zorder=5, label='Waypoints')
        # Add waypoint labels
        for i, wp in enumerate(waypoints):
            ax1.annotate(f'W{i}', (wp[0], wp[1]), 
                        xytext=(5, 5), textcoords='offset points', fontsize=9)
    
    # Plot ideal path
    if len(ideal_path) > 0:
        ax1.plot(ideal_path[:, 0], ideal_path[:, 1], 'b--', 
                linewidth=2, alpha=0.6, label='Ideal B-spline Path')
    
    # Plot actual robot trajectory
    if len(trajectory_data) > 0:
        ax1.plot(trajectory_data[:, 1], trajectory_data[:, 2], 'g-', 
                linewidth=2, label='Actual Robot Path')
        
        # Mark start and end
        ax1.plot(trajectory_data[0, 1], trajectory_data[0, 2], 'gs', 
                markersize=12, label='Start')
        ax1.plot(trajectory_data[-1, 1], trajectory_data[-1, 2], 'gD', 
                markersize=12, label='End')
        
        # Highlight high error regions
        high_error_idx = np.where(errors > 0.05)[0]  # More than 5cm error
        if len(high_error_idx) > 0:
            ax1.scatter(trajectory_data[high_error_idx, 1], 
                       trajectory_data[high_error_idx, 2],
                       c='orange', s=20, alpha=0.5, label='High Error (>5cm)')
        
        # Draw circles at corners to show deviation
        if len(waypoints) > 2:
            for i in range(1, len(waypoints) - 1):
                # Check if this is a corner
                v1 = waypoints[i] - waypoints[i-1]
                v2 = waypoints[i+1] - waypoints[i]
                angle = np.arccos(np.clip(np.dot(v1[:2], v2[:2]) / 
                                         (np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2])), -1, 1))
                
                if angle > np.pi/6:  # More than 30 degrees
                    circle = Circle(waypoints[i, :2], 0.15, 
                                  fill=False, edgecolor='red', 
                                  linewidth=2, linestyle='--', alpha=0.5)
                    ax1.add_patch(circle)
    
    ax1.legend(loc='best', fontsize=10)
    
    # Error over time plot
    ax2 = plt.subplot(2, 3, 2)
    if len(errors) > 0 and len(trajectory_data) > 0:
        ax2.plot(trajectory_data[:, 0], errors * 1000, 'r-', linewidth=2)
        ax2.axhline(y=50, color='orange', linestyle='--', alpha=0.5, label='50mm')
        ax2.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='100mm')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Cross-track Error (mm)')
        ax2.set_title('Tracking Error Over Time')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
    
    # Velocity profile
    ax3 = plt.subplot(2, 3, 3)
    if len(trajectory_data) > 0:
        speeds = np.sqrt(trajectory_data[:, 4]**2 + trajectory_data[:, 5]**2)
        ax3.plot(trajectory_data[:, 0], speeds, 'b-', linewidth=2, label='Linear Speed')
        ax3.plot(trajectory_data[:, 0], np.abs(trajectory_data[:, 6]), 'r-', 
                linewidth=2, alpha=0.7, label='Angular Speed')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (m/s, rad/s)')
        ax3.set_title('Velocity Profile')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
    
    # Error histogram
    ax4 = plt.subplot(2, 3, 5)
    if len(errors) > 0:
        ax4.hist(errors * 1000, bins=30, edgecolor='black', alpha=0.7, color='blue')
        ax4.axvline(x=np.mean(errors) * 1000, color='red', 
                   linestyle='--', linewidth=2, label=f'Mean: {np.mean(errors)*1000:.1f}mm')
        ax4.axvline(x=np.max(errors) * 1000, color='orange', 
                   linestyle='--', linewidth=2, label=f'Max: {np.max(errors)*1000:.1f}mm')
        ax4.set_xlabel('Cross-track Error (mm)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Error Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    
    # Statistics panel
    ax5 = plt.subplot(2, 3, 6)
    ax5.axis('off')
    
    stats_text = "TRAJECTORY STATISTICS\n" + "="*35 + "\n\n"
    
    if len(errors) > 0:
        stats_text += "Cross-track Error:\n"
        stats_text += f"  Mean:      {np.mean(errors)*1000:.2f} mm\n"
        stats_text += f"  Maximum:   {np.max(errors)*1000:.2f} mm\n"
        stats_text += f"  Std Dev:   {np.std(errors)*1000:.2f} mm\n"
        stats_text += f"  95th %ile: {np.percentile(errors, 95)*1000:.2f} mm\n\n"
    
    if len(trajectory_data) > 0:
        speeds = np.sqrt(trajectory_data[:, 4]**2 + trajectory_data[:, 5]**2)
        stats_text += "Velocity:\n"
        stats_text += f"  Avg Speed:    {np.mean(speeds):.3f} m/s\n"
        stats_text += f"  Max Speed:    {np.max(speeds):.3f} m/s\n"
        stats_text += f"  Avg Angular:  {np.mean(np.abs(trajectory_data[:, 6])):.3f} rad/s\n\n"
        stats_text += f"Duration: {trajectory_data[-1, 0]:.2f} seconds\n"
        stats_text += f"Data Points: {len(trajectory_data)}\n"
    
    # Identify corners with high deviation
    if len(waypoints) > 2 and len(errors) > 0:
        corner_errors = []
        for i in range(1, len(waypoints) - 1):
            # Find trajectory points near this waypoint
            wp_pos = waypoints[i, :2]
            distances = np.sqrt((trajectory_data[:, 1] - wp_pos[0])**2 + 
                              (trajectory_data[:, 2] - wp_pos[1])**2)
            near_idx = np.where(distances < 0.2)[0]
            if len(near_idx) > 0:
                max_error = np.max(errors[near_idx]) * 1000
                corner_errors.append((i, max_error))
        
        if corner_errors:
            stats_text += "\nCorner Deviations:\n"
            for corner_id, error in corner_errors:
                status = "✓" if error < 50 else "⚠" if error < 100 else "✗"
                stats_text += f"  Corner {corner_id}: {error:.1f} mm {status}\n"
    
    ax5.text(0.05, 0.5, stats_text, fontsize=10, family='monospace',
            verticalalignment='center',
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    plt.tight_layout()
    
    # Save figure
    output_file = 'trajectory_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Analysis saved to: {output_file}")
    
    # Show plot only if not in headless mode
    import os
    if os.environ.get('DISPLAY'):
        plt.show()
    else:
        print("Running in headless mode - plot saved but not displayed")

def main():
    log_file = "trajectory_log.txt"
    
    # Check if log file exists
    if not os.path.exists(log_file):
        print("=" * 60)
        print("Trajectory log file not found!")
        print("=" * 60)
        print(f"Please run the demo with logging first:")
        print(f"  ./build/libs/algos/demo/demo_with_logging [test_case] [traj_type]")
        print("")
        print("Test cases:")
        print("  1: Square path")
        print("  2: Circle")
        print("  3: Figure-8")
        print("  4: Straight line")
        print("")
        print("Trajectory types:")
        print("  1: B-spline")
        print("  2: Uniform B-spline")
        print("  3: Bezier")
        print("=" * 60)
        return
    
    # Read trajectory data
    print(f"Reading trajectory data from {log_file}...")
    waypoints, trajectory_data, trajectory_type = read_trajectory_log(log_file)
    
    if waypoints is None or trajectory_data is None:
        print("Error reading trajectory data!")
        return
    
    print(f"Loaded {len(waypoints)} waypoints and {len(trajectory_data)} trajectory points")
    print(f"Trajectory type: {trajectory_type}")
    
    # Create visualization
    plot_trajectory_analysis(waypoints, trajectory_data, trajectory_type)

if __name__ == "__main__":
    main()