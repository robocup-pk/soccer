#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_trajectory(filename):
    # Read the CSV file
    waypoints = []
    trajectory = []
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#') or line.strip() == '':
                    continue
                    
                parts = line.strip().split(',')
                if parts[0] == 'WAYPOINT':
                    x, y, theta = float(parts[1]), float(parts[2]), float(parts[3])
                    waypoints.append([x, y, theta])
                elif parts[0] == 'TRAJECTORY':
                    x, y, theta = float(parts[1]), float(parts[2]), float(parts[3])
                    trajectory.append([x, y, theta])
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    if not waypoints or not trajectory:
        print("No data found in file")
        return
    
    # Convert to numpy arrays
    waypoints = np.array(waypoints)
    trajectory = np.array(trajectory)
    
    # Create the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: X-Y trajectory
    ax1.set_title('B-Spline Trajectory (X-Y View)', fontsize=14)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Plot the square boundary
    square = plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor='gray', 
                          linewidth=2, linestyle='--', label='Boundary')
    ax1.add_patch(square)
    
    # Plot trajectory
    ax1.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, 
             label='B-Spline Trajectory', alpha=0.8)
    
    # Plot waypoints
    ax1.plot(waypoints[:, 0], waypoints[:, 1], 'ro', markersize=10, 
             label='Waypoints', zorder=5)
    
    # Add waypoint labels
    for i, (x, y, _) in enumerate(waypoints):
        ax1.annotate(f'W{i}', (x, y), xytext=(5, 5), textcoords='offset points')
    
    # Add start and end markers
    ax1.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=12, 
             label='Start', zorder=6)
    ax1.plot(trajectory[-1, 0], trajectory[-1, 1], 'rs', markersize=12, 
             label='End', zorder=6)
    
    # Calculate and display overshoot
    max_x = np.max(trajectory[:, 0])
    min_x = np.min(trajectory[:, 0])
    max_y = np.max(trajectory[:, 1])
    min_y = np.min(trajectory[:, 1])
    
    overshoot_info = []
    if max_x > 1:
        overshoot_info.append(f"X max: {max_x:.3f}m (overshoot: {max_x-1:.3f}m)")
    if min_x < 0:
        overshoot_info.append(f"X min: {min_x:.3f}m (overshoot: {-min_x:.3f}m)")
    if max_y > 1:
        overshoot_info.append(f"Y max: {max_y:.3f}m (overshoot: {max_y-1:.3f}m)")
    if min_y < 0:
        overshoot_info.append(f"Y min: {min_y:.3f}m (overshoot: {-min_y:.3f}m)")
    
    if overshoot_info:
        ax1.text(0.02, 0.98, '\n'.join(overshoot_info), 
                transform=ax1.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))
    
    ax1.legend(loc='upper right')
    ax1.set_xlim(-0.2, 1.3)
    ax1.set_ylim(-0.2, 1.3)
    
    # Plot 2: Heading angle over time
    ax2.set_title('Robot Heading Angle', fontsize=14)
    ax2.set_xlabel('Point Index')
    ax2.set_ylabel('Heading (degrees)')
    ax2.grid(True, alpha=0.3)
    
    # Convert radians to degrees
    trajectory_deg = trajectory[:, 2] * 180 / np.pi
    ax2.plot(trajectory_deg, 'b-', linewidth=2)
    
    # Mark waypoint positions
    if len(trajectory) > 0:
        # Estimate waypoint positions in trajectory
        for i, wp in enumerate(waypoints):
            # Find closest point in trajectory
            distances = np.sqrt((trajectory[:, 0] - wp[0])**2 + 
                              (trajectory[:, 1] - wp[1])**2)
            closest_idx = np.argmin(distances)
            ax2.axvline(x=closest_idx, color='r', linestyle='--', alpha=0.5)
            ax2.text(closest_idx, ax2.get_ylim()[1]*0.9, f'W{i}', 
                    ha='center', color='red')
    
    plt.tight_layout()
    
    # Save the plot
    output_file = filename.replace('.csv', '_plot.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Show the plot (comment out for non-interactive mode)
    # plt.show()

def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Default filename
        filename = "original_waypoints_trajectory.csv"
    
    print(f"Plotting trajectory from: {filename}")
    plot_trajectory(filename)

if __name__ == "__main__":
    main()