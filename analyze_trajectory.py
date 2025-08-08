#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import subprocess
import re
import sys
import time
from scipy.interpolate import interp1d

class TrajectoryAnalyzer:
    def __init__(self):
        self.waypoints = []
        self.robot_positions = []
        self.robot_velocities = []
        self.timestamps = []
        self.desired_positions = []
        self.control_points = []
        
    def run_demo_and_collect(self, demo_command, duration=10):
        """Run demo and collect trajectory data"""
        print(f"Running demo: {' '.join(demo_command)}")
        print(f"Collecting data for {duration} seconds...")
        
        process = subprocess.Popen(
            demo_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        start_time = time.time()
        current_pos = np.array([0.0, 0.0, 0.0])
        
        # Regex patterns
        waypoint_pattern = r'(\d+): \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)'
        velocity_pattern = r'\[hw::HardwareManager::SetBodyVelocity\] Body Velocity:\s+([-\d.e-]+)\s+([-\d.e-]+)\s+([-\d.e-]+)\s+m/s'
        control_point_pattern = r'Control point \d+: \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)'
        
        dt = 0.01  # Approximate time step
        
        try:
            for line in iter(process.stdout.readline, ''):
                if time.time() - start_time > duration:
                    break
                    
                # Parse waypoints
                waypoint_match = re.search(waypoint_pattern, line)
                if waypoint_match:
                    x = float(waypoint_match.group(2))
                    y = float(waypoint_match.group(3))
                    theta = float(waypoint_match.group(4))
                    self.waypoints.append([x, y, theta])
                    print(f"  Waypoint: ({x:.2f}, {y:.2f}, {theta:.2f})")
                
                # Parse control points (if debug output is enabled)
                cp_match = re.search(control_point_pattern, line)
                if cp_match:
                    x = float(cp_match.group(1))
                    y = float(cp_match.group(2))
                    theta = float(cp_match.group(3))
                    self.control_points.append([x, y, theta])
                
                # Parse robot velocity and integrate for position
                velocity_match = re.search(velocity_pattern, line)
                if velocity_match:
                    vx = float(velocity_match.group(1))
                    vy = float(velocity_match.group(2))
                    omega = float(velocity_match.group(3))
                    
                    # Store velocity
                    self.robot_velocities.append([vx, vy, omega])
                    
                    # Integrate to get position (simple Euler integration)
                    # Account for robot orientation
                    cos_theta = np.cos(current_pos[2])
                    sin_theta = np.sin(current_pos[2])
                    
                    # Transform body velocity to world frame
                    world_vx = vx * cos_theta - vy * sin_theta
                    world_vy = vx * sin_theta + vy * cos_theta
                    
                    # Update position
                    current_pos[0] += world_vx * dt
                    current_pos[1] += world_vy * dt
                    current_pos[2] += omega * dt
                    
                    # Store position and time
                    self.robot_positions.append(current_pos.copy())
                    self.timestamps.append(time.time() - start_time)
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            process.terminate()
            process.wait()
            
        print(f"Collected {len(self.robot_positions)} position samples")
        
    def generate_ideal_bspline(self, num_samples=500):
        """Generate ideal B-spline trajectory"""
        if len(self.waypoints) < 2:
            return []
        
        from scipy.interpolate import splprep, splev
        
        points = np.array(self.waypoints)
        
        # Check if it's a closed curve
        is_closed = np.allclose(points[0, :2], points[-1, :2], atol=0.01)
        
        try:
            if is_closed and len(points) > 3:
                # Closed curve - use periodic spline
                tck, u = splprep([points[:-1, 0], points[:-1, 1]], s=0, per=True, k=min(3, len(points)-2))
            else:
                # Open curve
                k = min(3, len(points) - 1)
                tck, u = splprep([points[:, 0], points[:, 1]], s=0, k=k)
            
            u_new = np.linspace(0, 1, num_samples)
            x_new, y_new = splev(u_new, tck)
            
            return np.column_stack([x_new, y_new])
        except Exception as e:
            print(f"Warning: Could not generate spline: {e}")
            return np.array(self.waypoints)[:, :2]
    
    def calculate_errors(self, spline_points):
        """Calculate cross-track and along-track errors"""
        errors = []
        
        for pos in self.robot_positions:
            if len(spline_points) < 2:
                errors.append(0.0)
                continue
                
            min_dist = float('inf')
            closest_point = None
            
            for i in range(len(spline_points) - 1):
                p1 = spline_points[i]
                p2 = spline_points[i + 1]
                
                # Vector from p1 to p2
                v = p2 - p1
                # Vector from p1 to robot
                w = pos[:2] - p1
                
                # Project w onto v
                c1 = np.dot(w, v)
                if c1 <= 0:
                    dist = np.linalg.norm(pos[:2] - p1)
                    if dist < min_dist:
                        min_dist = dist
                        closest_point = p1
                else:
                    c2 = np.dot(v, v)
                    if c1 >= c2:
                        dist = np.linalg.norm(pos[:2] - p2)
                        if dist < min_dist:
                            min_dist = dist
                            closest_point = p2
                    else:
                        b = c1 / c2
                        pb = p1 + b * v
                        dist = np.linalg.norm(pos[:2] - pb)
                        if dist < min_dist:
                            min_dist = dist
                            closest_point = pb
            
            errors.append(min_dist)
            
        return np.array(errors)
    
    def plot_analysis(self):
        """Create comprehensive analysis plots"""
        if not self.robot_positions:
            print("No data to plot!")
            return
            
        robot_array = np.array(self.robot_positions)
        waypoint_array = np.array(self.waypoints) if self.waypoints else None
        
        # Generate ideal B-spline
        spline_points = self.generate_ideal_bspline()
        
        # Calculate errors
        errors = self.calculate_errors(spline_points)
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 10))
        
        # Main trajectory plot
        ax1 = plt.subplot(2, 3, (1, 4))
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('Robot Trajectory Tracking Analysis', fontsize=14, fontweight='bold')
        
        # Plot waypoints
        if waypoint_array is not None:
            ax1.plot(waypoint_array[:, 0], waypoint_array[:, 1], 'ro', 
                    markersize=10, label='Waypoints', zorder=5)
            # Add waypoint labels
            for i, wp in enumerate(waypoint_array):
                ax1.annotate(f'W{i}', (wp[0], wp[1]), 
                           xytext=(5, 5), textcoords='offset points')
        
        # Plot ideal B-spline
        if len(spline_points) > 0:
            ax1.plot(spline_points[:, 0], spline_points[:, 1], 'b--', 
                    linewidth=2, alpha=0.6, label='Ideal B-spline')
        
        # Plot actual robot path
        ax1.plot(robot_array[:, 0], robot_array[:, 1], 'g-', 
                linewidth=2, label='Actual Path')
        
        # Mark start and end
        ax1.plot(robot_array[0, 0], robot_array[0, 1], 'gs', 
                markersize=12, label='Start')
        ax1.plot(robot_array[-1, 0], robot_array[-1, 1], 'gD', 
                markersize=12, label='End')
        
        # Highlight corners with deviation
        if waypoint_array is not None and len(waypoint_array) > 2:
            for i in range(1, len(waypoint_array) - 1):
                # Calculate angle change at waypoint
                v1 = waypoint_array[i] - waypoint_array[i-1]
                v2 = waypoint_array[i+1] - waypoint_array[i]
                angle = np.arccos(np.clip(np.dot(v1[:2], v2[:2]) / 
                                         (np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2])), -1, 1))
                
                if angle > np.pi/6:  # More than 30 degrees
                    circle = Circle(waypoint_array[i, :2], 0.15, 
                                  fill=False, edgecolor='red', 
                                  linewidth=2, linestyle='--', alpha=0.5)
                    ax1.add_patch(circle)
        
        ax1.legend(loc='best')
        ax1.set_xlim(np.min(robot_array[:, 0]) - 0.3, np.max(robot_array[:, 0]) + 0.3)
        ax1.set_ylim(np.min(robot_array[:, 1]) - 0.3, np.max(robot_array[:, 1]) + 0.3)
        
        # Error over time
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(self.timestamps, errors * 1000, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Cross-track Error (mm)')
        ax2.set_title('Tracking Error Over Time')
        ax2.grid(True, alpha=0.3)
        
        # Add horizontal lines for reference
        ax2.axhline(y=50, color='orange', linestyle='--', alpha=0.5, label='50mm')
        ax2.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='100mm')
        ax2.legend()
        
        # Velocity profile
        ax3 = plt.subplot(2, 3, 3)
        if self.robot_velocities:
            vel_array = np.array(self.robot_velocities)
            speed = np.sqrt(vel_array[:, 0]**2 + vel_array[:, 1]**2)
            ax3.plot(self.timestamps, speed, 'b-', label='Linear Speed')
            ax3.plot(self.timestamps, np.abs(vel_array[:, 2]), 'r-', label='Angular Speed')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Speed (m/s, rad/s)')
            ax3.set_title('Velocity Profile')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
        
        # Error histogram
        ax4 = plt.subplot(2, 3, 5)
        ax4.hist(errors * 1000, bins=30, edgecolor='black', alpha=0.7)
        ax4.set_xlabel('Cross-track Error (mm)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Error Distribution')
        ax4.axvline(x=np.mean(errors) * 1000, color='red', 
                   linestyle='--', label=f'Mean: {np.mean(errors)*1000:.1f}mm')
        ax4.axvline(x=np.max(errors) * 1000, color='orange', 
                   linestyle='--', label=f'Max: {np.max(errors)*1000:.1f}mm')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Corner analysis
        ax5 = plt.subplot(2, 3, 6)
        if waypoint_array is not None and len(waypoint_array) > 2:
            corner_errors = []
            corner_labels = []
            
            for i in range(1, len(waypoint_array) - 1):
                # Find robot positions near this waypoint
                wp_pos = waypoint_array[i, :2]
                distances = [np.linalg.norm(rp[:2] - wp_pos) for rp in robot_array]
                
                # Get errors near the corner (within 0.2m)
                near_corner_idx = [j for j, d in enumerate(distances) if d < 0.2]
                if near_corner_idx:
                    corner_error = np.max(errors[near_corner_idx]) * 1000
                    corner_errors.append(corner_error)
                    corner_labels.append(f'Corner {i}')
            
            if corner_errors:
                bars = ax5.bar(corner_labels, corner_errors)
                ax5.set_ylabel('Max Error (mm)')
                ax5.set_title('Maximum Error at Corners')
                ax5.grid(True, alpha=0.3)
                
                # Color bars based on error magnitude
                for bar, error in zip(bars, corner_errors):
                    if error > 100:
                        bar.set_color('red')
                    elif error > 50:
                        bar.set_color('orange')
                    else:
                        bar.set_color('green')
        
        # Add overall statistics
        stats_text = f"Statistics:\n"
        stats_text += f"Mean Error: {np.mean(errors)*1000:.2f} mm\n"
        stats_text += f"Max Error: {np.max(errors)*1000:.2f} mm\n"
        stats_text += f"Std Dev: {np.std(errors)*1000:.2f} mm\n"
        if self.robot_velocities:
            vel_array = np.array(self.robot_velocities)
            speed = np.sqrt(vel_array[:, 0]**2 + vel_array[:, 1]**2)
            stats_text += f"Avg Speed: {np.mean(speed):.3f} m/s\n"
            stats_text += f"Max Speed: {np.max(speed):.3f} m/s"
        
        fig.text(0.02, 0.02, stats_text, fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray"))
        
        plt.tight_layout()
        
        # Save figure
        filename = f"trajectory_analysis_{int(time.time())}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Analysis saved to {filename}")
        
        plt.show()

if __name__ == "__main__":
    # Default parameters
    test_case = "1"  # Square path
    traj_type = "2"  # Uniform B-spline
    duration = 10    # seconds
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        test_case = sys.argv[1]
    if len(sys.argv) > 2:
        traj_type = sys.argv[2]
    if len(sys.argv) > 3:
        duration = float(sys.argv[3])
    
    # Build demo command
    demo_cmd = ["./build/libs/algos/demo/basic_demo", test_case, traj_type]
    
    print("=" * 60)
    print("Trajectory Analysis Tool")
    print("=" * 60)
    print(f"Test case: {test_case}")
    print(f"  1: Square path")
    print(f"  2: Circle")
    print(f"  3: Figure-8")
    print(f"  4: Straight line")
    print(f"Trajectory type: {traj_type}")
    print(f"  1: B-spline (traditional)")
    print(f"  2: Uniform B-spline")
    print(f"  3: Bezier (RoboJackets)")
    print(f"Duration: {duration} seconds")
    print("=" * 60)
    
    # Run analysis
    analyzer = TrajectoryAnalyzer()
    analyzer.run_demo_and_collect(demo_cmd, duration)
    analyzer.plot_analysis()