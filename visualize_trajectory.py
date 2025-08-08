#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation
import subprocess
import re
import threading
import queue
import time

class TrajectoryVisualizer:
    def __init__(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Main trajectory plot
        self.ax1.set_xlim(-1.5, 0.5)
        self.ax1.set_ylim(-0.5, 1.5)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_title('Robot Trajectory Tracking')
        
        # Error plot
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Error (m)')
        self.ax2.set_title('Cross-track Error Over Time')
        self.ax2.grid(True, alpha=0.3)
        
        # Data storage
        self.waypoints = []
        self.spline_points = []
        self.robot_positions = []
        self.robot_velocities = []
        self.timestamps = []
        self.errors = []
        
        # Plot elements
        self.waypoint_plot = None
        self.spline_plot = None
        self.robot_trail = None
        self.robot_current = None
        self.robot_circle = None
        self.velocity_arrow = None
        self.error_plot = None
        
        # Thread for reading demo output
        self.data_queue = queue.Queue()
        self.start_time = None
        
    def parse_demo_output(self, process):
        """Parse the demo output in real-time"""
        waypoint_pattern = r'(\d+): \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)'
        velocity_pattern = r'\[hw::HardwareManager::SetBodyVelocity\] Body Velocity:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)'
        spline_pattern = r'\[UniformBSplineTrajectoryPlanner::SetPath\].*Control points: (\d+)'
        
        for line in iter(process.stdout.readline, ''):
            if not line:
                break
                
            # Parse waypoints
            waypoint_match = re.search(waypoint_pattern, line)
            if waypoint_match:
                idx = int(waypoint_match.group(1))
                x = float(waypoint_match.group(2))
                y = float(waypoint_match.group(3))
                theta = float(waypoint_match.group(4))
                self.data_queue.put(('waypoint', (x, y, theta)))
            
            # Parse robot velocity (indicates position update)
            velocity_match = re.search(velocity_pattern, line)
            if velocity_match:
                vx = float(velocity_match.group(1))
                vy = float(velocity_match.group(2))
                omega = float(velocity_match.group(3))
                self.data_queue.put(('velocity', (vx, vy, omega)))
                
    def generate_bspline(self, waypoints, num_samples=200):
        """Generate B-spline curve from waypoints"""
        if len(waypoints) < 2:
            return []
        
        # Simple cubic B-spline generation (approximation)
        # For actual implementation, we'd need to match the C++ implementation
        from scipy.interpolate import splprep, splev
        
        points = np.array(waypoints)
        
        # Handle closed curves
        if np.allclose(points[0], points[-1], atol=0.01):
            # Closed curve
            tck, u = splprep([points[:-1, 0], points[:-1, 1]], s=0, per=True, k=3)
        else:
            # Open curve
            tck, u = splprep([points[:, 0], points[:, 1]], s=0, k=min(3, len(points)-1))
        
        u_new = np.linspace(0, 1, num_samples)
        x_new, y_new = splev(u_new, tck)
        
        return list(zip(x_new, y_new))
    
    def calculate_cross_track_error(self, robot_pos, spline_points):
        """Calculate minimum distance from robot to spline"""
        if len(spline_points) < 2:
            return 0.0
            
        min_dist = float('inf')
        for i in range(len(spline_points) - 1):
            p1 = np.array(spline_points[i])
            p2 = np.array(spline_points[i + 1])
            
            # Vector from p1 to p2
            v = p2 - p1
            # Vector from p1 to robot
            w = robot_pos - p1
            
            # Project w onto v
            c1 = np.dot(w, v)
            if c1 <= 0:
                dist = np.linalg.norm(robot_pos - p1)
            else:
                c2 = np.dot(v, v)
                if c1 >= c2:
                    dist = np.linalg.norm(robot_pos - p2)
                else:
                    b = c1 / c2
                    pb = p1 + b * v
                    dist = np.linalg.norm(robot_pos - pb)
            
            min_dist = min(min_dist, dist)
            
        return min_dist
    
    def update_plot(self, frame):
        """Update the plot with new data"""
        # Process queued data
        while not self.data_queue.empty():
            try:
                data_type, data = self.data_queue.get_nowait()
                
                if data_type == 'waypoint':
                    self.waypoints.append(data[:2])  # Only x, y
                    
                elif data_type == 'velocity':
                    # Integrate velocity to get position (simplified)
                    if self.start_time is None:
                        self.start_time = time.time()
                        self.robot_positions.append([0, 0])
                    else:
                        dt = 0.01  # Approximate time step
                        if self.robot_positions:
                            last_pos = self.robot_positions[-1]
                            new_pos = [
                                last_pos[0] + data[0] * dt,
                                last_pos[1] + data[1] * dt
                            ]
                            self.robot_positions.append(new_pos)
                            self.robot_velocities.append(data[:2])
                            
                            current_time = time.time() - self.start_time
                            self.timestamps.append(current_time)
                            
                            # Calculate error if we have spline
                            if self.spline_points:
                                error = self.calculate_cross_track_error(
                                    np.array(new_pos), self.spline_points
                                )
                                self.errors.append(error)
                            
            except queue.Empty:
                break
        
        # Generate spline if we have waypoints but no spline yet
        if self.waypoints and not self.spline_points:
            self.spline_points = self.generate_bspline(self.waypoints)
        
        # Update waypoint plot
        if self.waypoints:
            waypoint_array = np.array(self.waypoints)
            if self.waypoint_plot is None:
                self.waypoint_plot, = self.ax1.plot(
                    waypoint_array[:, 0], waypoint_array[:, 1],
                    'ro', markersize=8, label='Waypoints'
                )
            else:
                self.waypoint_plot.set_data(waypoint_array[:, 0], waypoint_array[:, 1])
        
        # Update spline plot
        if self.spline_points:
            spline_array = np.array(self.spline_points)
            if self.spline_plot is None:
                self.spline_plot, = self.ax1.plot(
                    spline_array[:, 0], spline_array[:, 1],
                    'b-', linewidth=2, alpha=0.6, label='Desired B-spline'
                )
            else:
                self.spline_plot.set_data(spline_array[:, 0], spline_array[:, 1])
        
        # Update robot trajectory
        if self.robot_positions:
            robot_array = np.array(self.robot_positions)
            if self.robot_trail is None:
                self.robot_trail, = self.ax1.plot(
                    robot_array[:, 0], robot_array[:, 1],
                    'g-', linewidth=1.5, alpha=0.8, label='Actual Path'
                )
            else:
                self.robot_trail.set_data(robot_array[:, 0], robot_array[:, 1])
            
            # Current robot position
            current_pos = robot_array[-1]
            if self.robot_current is None:
                self.robot_current, = self.ax1.plot(
                    current_pos[0], current_pos[1],
                    'go', markersize=10
                )
                # Robot circle (90mm radius)
                self.robot_circle = Circle(
                    (current_pos[0], current_pos[1]), 0.09,
                    fill=False, edgecolor='green', linewidth=2
                )
                self.ax1.add_patch(self.robot_circle)
            else:
                self.robot_current.set_data([current_pos[0]], [current_pos[1]])
                self.robot_circle.center = (current_pos[0], current_pos[1])
            
            # Velocity arrow
            if self.robot_velocities:
                vel = self.robot_velocities[-1]
                if self.velocity_arrow is not None:
                    self.velocity_arrow.remove()
                vel_scale = 0.2
                self.velocity_arrow = self.ax1.arrow(
                    current_pos[0], current_pos[1],
                    vel[0] * vel_scale, vel[1] * vel_scale,
                    head_width=0.03, head_length=0.02,
                    fc='red', ec='red', alpha=0.7
                )
        
        # Update error plot
        if self.errors and self.timestamps:
            if self.error_plot is None:
                self.error_plot, = self.ax2.plot(
                    self.timestamps, self.errors,
                    'r-', linewidth=1.5
                )
            else:
                self.error_plot.set_data(self.timestamps, self.errors)
            
            # Update axis limits
            if self.timestamps:
                self.ax2.set_xlim(0, max(self.timestamps) * 1.1)
                if self.errors:
                    max_error = max(self.errors) * 1.2
                    self.ax2.set_ylim(0, max(0.1, max_error))
        
        # Add legend
        if self.waypoint_plot and not self.ax1.get_legend():
            self.ax1.legend(loc='upper right')
        
        # Add statistics
        if self.errors:
            max_error = max(self.errors)
            avg_error = np.mean(self.errors)
            self.ax2.set_title(f'Cross-track Error (Max: {max_error:.3f}m, Avg: {avg_error:.3f}m)')
        
        return [self.waypoint_plot, self.spline_plot, self.robot_trail, 
                self.robot_current, self.error_plot]
    
    def run(self, demo_command):
        """Run the demo and visualize in real-time"""
        # Start the demo process
        process = subprocess.Popen(
            demo_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        # Start parser thread
        parser_thread = threading.Thread(
            target=self.parse_demo_output,
            args=(process,)
        )
        parser_thread.daemon = True
        parser_thread.start()
        
        # Set up animation
        ani = FuncAnimation(
            self.fig, self.update_plot,
            interval=50,  # Update every 50ms
            blit=False,
            cache_frame_data=False
        )
        
        plt.show()
        
        # Clean up
        process.terminate()
        parser_thread.join(timeout=1)

if __name__ == "__main__":
    import sys
    
    # Default command
    demo_cmd = ["./build/libs/algos/demo/basic_demo", "1", "2"]
    
    # Allow custom test case and trajectory type
    if len(sys.argv) > 1:
        demo_cmd[1] = sys.argv[1]
    if len(sys.argv) > 2:
        demo_cmd[2] = sys.argv[2]
    
    print(f"Running demo with command: {' '.join(demo_cmd)}")
    print("Test case 1: Square path")
    print("Test case 2: Circle")
    print("Test case 3: Figure-8")
    print("Test case 4: Straight line")
    print("\nTrajectory type 2: Uniform B-spline")
    
    visualizer = TrajectoryVisualizer()
    visualizer.run(demo_cmd)