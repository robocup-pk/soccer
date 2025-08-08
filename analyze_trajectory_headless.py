#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import subprocess
import re
import sys
import time
from datetime import datetime

class TrajectoryAnalyzer:
    def __init__(self):
        self.waypoints = []
        self.robot_positions = []
        self.robot_velocities = []
        self.timestamps = []
        
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
        
        # Regex patterns - fixed to handle scientific notation and spaces properly
        waypoint_pattern = r'(\d+): \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)'
        velocity_pattern = r'Body Velocity:\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)'
        
        dt = 0.01  # Approximate time step
        line_count = 0
        
        try:
            for line in iter(process.stdout.readline, ''):
                if time.time() - start_time > duration:
                    break
                
                line_count += 1
                    
                # Parse waypoints
                waypoint_match = re.search(waypoint_pattern, line)
                if waypoint_match:
                    x = float(waypoint_match.group(2))
                    y = float(waypoint_match.group(3))
                    theta = float(waypoint_match.group(4))
                    self.waypoints.append([x, y, theta])
                    print(f"  Waypoint {len(self.waypoints)-1}: ({x:.2f}, {y:.2f}, {theta:.2f})")
                
                # Parse robot velocity and integrate for position
                velocity_match = re.search(velocity_pattern, line)
                if velocity_match:
                    try:
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
                        
                        # Normalize angle
                        while current_pos[2] > np.pi:
                            current_pos[2] -= 2 * np.pi
                        while current_pos[2] < -np.pi:
                            current_pos[2] += 2 * np.pi
                        
                        # Store position and time
                        self.robot_positions.append(current_pos.copy())
                        self.timestamps.append(time.time() - start_time)
                    except ValueError as e:
                        pass  # Skip malformed velocity lines
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            process.terminate()
            try:
                process.wait(timeout=1)
            except:
                process.kill()
            
        print(f"Processed {line_count} lines")
        print(f"Collected {len(self.robot_positions)} position samples")
        
    def generate_ideal_bspline(self, num_samples=500):
        """Generate ideal B-spline trajectory using simple interpolation"""
        if len(self.waypoints) < 2:
            return []
        
        # For a simple square path, just connect the waypoints with straight lines
        # This is a simplified version that matches what the C++ code should produce
        points = []
        waypoint_array = np.array(self.waypoints)
        
        for i in range(len(waypoint_array) - 1):
            p1 = waypoint_array[i, :2]
            p2 = waypoint_array[i+1, :2]
            
            # Add intermediate points
            for t in np.linspace(0, 1, 50):
                point = p1 * (1 - t) + p2 * t
                points.append(point)
        
        return np.array(points)
    
    def calculate_errors(self, spline_points):
        """Calculate cross-track errors"""
        errors = []
        
        for pos in self.robot_positions:
            if len(spline_points) < 2:
                errors.append(0.0)
                continue
                
            min_dist = float('inf')
            
            for i in range(len(spline_points) - 1):
                p1 = spline_points[i]
                p2 = spline_points[i + 1]
                
                # Vector from p1 to p2
                v = p2 - p1
                v_norm = np.linalg.norm(v)
                if v_norm < 1e-10:
                    continue
                    
                # Vector from p1 to robot
                w = pos[:2] - p1
                
                # Project w onto v
                t = max(0, min(1, np.dot(w, v) / (v_norm * v_norm)))
                
                # Closest point on segment
                closest = p1 + t * v
                
                # Distance to closest point
                dist = np.linalg.norm(pos[:2] - closest)
                
                if dist < min_dist:
                    min_dist = dist
            
            errors.append(min_dist)
            
        return np.array(errors)
    
    def plot_analysis(self, output_file):
        """Create comprehensive analysis plots and save to file"""
        if not self.robot_positions:
            print("No data to plot!")
            return
            
        robot_array = np.array(self.robot_positions)
        waypoint_array = np.array(self.waypoints) if self.waypoints else None
        
        # Generate ideal path (simplified)
        spline_points = self.generate_ideal_bspline()
        
        # Calculate errors
        errors = self.calculate_errors(spline_points)
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Main trajectory plot
        ax1 = axes[0, 0]
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        ax1.set_title('Robot Trajectory (Square Path)', fontsize=12, fontweight='bold')
        
        # Plot waypoints
        if waypoint_array is not None:
            ax1.plot(waypoint_array[:, 0], waypoint_array[:, 1], 'ro', 
                    markersize=8, label='Waypoints', zorder=5)
            # Add waypoint labels
            for i, wp in enumerate(waypoint_array):
                ax1.annotate(f'{i}', (wp[0], wp[1]), 
                           xytext=(5, 5), textcoords='offset points', fontsize=8)
        
        # Plot ideal path
        if len(spline_points) > 0:
            ax1.plot(spline_points[:, 0], spline_points[:, 1], 'b--', 
                    linewidth=2, alpha=0.6, label='Ideal Path')
        
        # Plot actual robot path
        ax1.plot(robot_array[:, 0], robot_array[:, 1], 'g-', 
                linewidth=1.5, label='Actual Path', alpha=0.8)
        
        # Highlight areas of high deviation
        if len(errors) > 0:
            high_error_idx = np.where(errors > 0.05)[0]  # More than 5cm error
            if len(high_error_idx) > 0:
                ax1.scatter(robot_array[high_error_idx, 0], 
                          robot_array[high_error_idx, 1], 
                          c='red', s=10, alpha=0.5, label='High Error (>5cm)')
        
        ax1.legend(loc='best', fontsize=9)
        
        # Error over time
        ax2 = axes[0, 1]
        if len(errors) > 0 and len(self.timestamps) > 0:
            ax2.plot(self.timestamps[:len(errors)], errors * 1000, 'r-', linewidth=1.5)
            ax2.axhline(y=50, color='orange', linestyle='--', alpha=0.5, label='50mm')
            ax2.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='100mm')
            ax2.set_xlabel('Time (s)', fontsize=10)
            ax2.set_ylabel('Cross-track Error (mm)', fontsize=10)
            ax2.set_title('Tracking Error Over Time', fontsize=12)
            ax2.grid(True, alpha=0.3)
            ax2.legend(fontsize=9)
        
        # Velocity profile
        ax3 = axes[1, 0]
        if self.robot_velocities:
            vel_array = np.array(self.robot_velocities)
            speed = np.sqrt(vel_array[:, 0]**2 + vel_array[:, 1]**2)
            ax3.plot(self.timestamps[:len(speed)], speed, 'b-', label='Linear Speed', linewidth=1.5)
            ax3.plot(self.timestamps[:len(vel_array)], np.abs(vel_array[:, 2]), 'r-', 
                    label='Angular Speed', linewidth=1.5, alpha=0.7)
            ax3.set_xlabel('Time (s)', fontsize=10)
            ax3.set_ylabel('Speed (m/s, rad/s)', fontsize=10)
            ax3.set_title('Velocity Profile', fontsize=12)
            ax3.legend(fontsize=9)
            ax3.grid(True, alpha=0.3)
        
        # Statistics box
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        stats_text = "TRAJECTORY STATISTICS\n" + "="*30 + "\n\n"
        
        if len(errors) > 0:
            stats_text += f"Cross-track Error:\n"
            stats_text += f"  Mean:     {np.mean(errors)*1000:.2f} mm\n"
            stats_text += f"  Maximum:  {np.max(errors)*1000:.2f} mm\n"
            stats_text += f"  Std Dev:  {np.std(errors)*1000:.2f} mm\n"
            stats_text += f"  95th %ile: {np.percentile(errors, 95)*1000:.2f} mm\n\n"
            
            # Identify corners with high error
            corner_indices = []
            if waypoint_array is not None:
                for i in range(1, len(waypoint_array) - 1):
                    wp_pos = waypoint_array[i, :2]
                    distances = [np.linalg.norm(rp[:2] - wp_pos) for rp in robot_array]
                    near_idx = [j for j, d in enumerate(distances) if d < 0.3]
                    if near_idx and len(near_idx) < len(errors):
                        max_error_near = np.max(errors[near_idx]) * 1000
                        corner_indices.append((i, max_error_near))
            
            if corner_indices:
                stats_text += "Corner Analysis:\n"
                for corner_id, error in corner_indices:
                    stats_text += f"  Corner {corner_id}: {error:.1f} mm\n"
                stats_text += "\n"
        
        if self.robot_velocities:
            vel_array = np.array(self.robot_velocities)
            speed = np.sqrt(vel_array[:, 0]**2 + vel_array[:, 1]**2)
            stats_text += f"Velocity:\n"
            stats_text += f"  Avg Speed: {np.mean(speed):.3f} m/s\n"
            stats_text += f"  Max Speed: {np.max(speed):.3f} m/s\n"
            stats_text += f"  Avg Angular: {np.mean(np.abs(vel_array[:, 2])):.3f} rad/s\n"
        
        stats_text += f"\nData Points: {len(self.robot_positions)}\n"
        stats_text += f"Duration: {self.timestamps[-1] if self.timestamps else 0:.2f} s"
        
        ax4.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
                verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
        
        plt.suptitle(f'B-Spline Trajectory Analysis - {datetime.now().strftime("%Y-%m-%d %H:%M")}', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        # Save figure
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\nAnalysis saved to: {output_file}")
        
        # Also save raw data
        data_file = output_file.replace('.png', '_data.npz')
        np.savez(data_file,
                waypoints=self.waypoints,
                robot_positions=self.robot_positions,
                robot_velocities=self.robot_velocities,
                timestamps=self.timestamps,
                errors=errors)
        print(f"Raw data saved to: {data_file}")

if __name__ == "__main__":
    # Default parameters
    test_case = "1"  # Square path
    traj_type = "2"  # Uniform B-spline
    duration = 7     # seconds
    
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
    print("B-Spline Trajectory Analysis (Headless)")
    print("=" * 60)
    
    # Run analysis
    analyzer = TrajectoryAnalyzer()
    analyzer.run_demo_and_collect(demo_cmd, duration)
    
    # Generate output filename
    output_file = f"trajectory_analysis_test{test_case}_traj{traj_type}_{int(time.time())}.png"
    analyzer.plot_analysis(output_file)
    
    print("\nAnalysis complete!")