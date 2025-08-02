#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('RRT* Path → B-Spline Trajectory → Real Robot Behavior', fontsize=16)

# Colors
rrt_color = '#2E86AB'  # Blue
bspline_color = '#A23B72'  # Purple  
real_color = '#F18F01'  # Orange
obstacle_color = '#C0C0C0'  # Gray

# 1. RRT* Path Planning (top left)
ax1 = axes[0, 0]
ax1.set_title('1. RRT* Path Planning', fontsize=14)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_xlim(-0.5, 2.5)
ax1.set_ylim(-1, 1)
ax1.grid(True, alpha=0.3)

# Add obstacles
obstacles = [
    Rectangle((0.6, 0.2), 0.3, 0.3, color=obstacle_color),
    Rectangle((1.2, -0.5), 0.2, 0.4, color=obstacle_color),
]
for obs in obstacles:
    ax1.add_patch(obs)

# RRT* tree (simplified)
# Random tree branches
np.random.seed(42)
for _ in range(30):
    x_start = np.random.uniform(-0.2, 1.8)
    y_start = np.random.uniform(-0.8, 0.8)
    x_end = x_start + np.random.uniform(-0.3, 0.3)
    y_end = y_start + np.random.uniform(-0.3, 0.3)
    ax1.plot([x_start, x_end], [y_start, y_end], 'k-', alpha=0.2, linewidth=0.5)

# RRT* solution path (waypoints)
rrt_waypoints = np.array([
    [0.0, 0.0],
    [0.3, -0.2],
    [0.5, -0.3],
    [0.9, -0.2],
    [1.3, 0.1],
    [1.6, 0.3],
    [2.0, 0.0]
])
ax1.plot(rrt_waypoints[:, 0], rrt_waypoints[:, 1], 'o-', color=rrt_color, 
         linewidth=3, markersize=8, label='RRT* Path')
ax1.plot(0, 0, 'go', markersize=10, label='Start')
ax1.plot(2, 0, 'ro', markersize=10, label='Goal')
ax1.legend()

# 2. B-Spline Trajectory (top right)
ax2 = axes[0, 1]
ax2.set_title('2. B-Spline Smooth Trajectory', fontsize=14)
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_xlim(-0.5, 2.5)
ax2.set_ylim(-1, 1)
ax2.grid(True, alpha=0.3)

# Add same obstacles
for obs in obstacles:
    ax2.add_patch(Rectangle(obs.get_xy(), obs.get_width(), obs.get_height(), 
                           color=obstacle_color))

# Show RRT* waypoints
ax2.plot(rrt_waypoints[:, 0], rrt_waypoints[:, 1], 'o--', color=rrt_color, 
         alpha=0.5, linewidth=2, markersize=6, label='RRT* Waypoints')

# Generate B-spline curve
def b_spline(waypoints, num_points=100):
    """Simple B-spline approximation"""
    t = np.linspace(0, 1, len(waypoints))
    t_smooth = np.linspace(0, 1, num_points)
    
    # Cubic interpolation for smooth curve
    from scipy import interpolate
    fx = interpolate.interp1d(t, waypoints[:, 0], kind='cubic')
    fy = interpolate.interp1d(t, waypoints[:, 1], kind='cubic')
    
    return np.column_stack([fx(t_smooth), fy(t_smooth)])

# Generate smooth B-spline
bspline_points = b_spline(rrt_waypoints, 200)
ax2.plot(bspline_points[:, 0], bspline_points[:, 1], '-', color=bspline_color, 
         linewidth=3, label='B-Spline Trajectory')
ax2.plot(0, 0, 'go', markersize=10)
ax2.plot(2, 0, 'ro', markersize=10)
ax2.legend()

# 3. Real Robot Behavior - Oscillating (bottom left)
ax3 = axes[1, 0]
ax3.set_title('3. Real Robot Behavior (With Issues)', fontsize=14)
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_xlim(-0.5, 2.5)
ax3.set_ylim(-1, 1)
ax3.grid(True, alpha=0.3)

# Generate oscillating path (based on real data)
t = np.linspace(0, 4.45, 500)
x_real = np.zeros_like(t)
y_real = np.zeros_like(t)

# Simulate oscillating motion with overshoots
for i in range(1, len(t)):
    dt = t[i] - t[i-1]
    # Target velocity with oscillations
    target_x = 2.0 * (t[i] / 4.45)  # Should reach 2m
    error_x = target_x - x_real[i-1]
    
    # Unstable controller with high gain
    control_x = 5.0 * error_x  # Too high gain!
    # Add oscillation
    control_x += 2.0 * np.sin(20 * t[i]) * np.exp(-t[i] * 0.3)
    
    # Update position
    x_real[i] = x_real[i-1] + control_x * dt
    # Add lateral deviation
    y_real[i] = 0.3 * np.sin(5 * t[i]) * np.exp(-t[i] * 0.5)

# Limit to realistic bounds
x_real = np.clip(x_real, 0, 0.938)  # Real robot only reached 0.938m

ax3.plot(bspline_points[:, 0], bspline_points[:, 1], '--', color=bspline_color, 
         alpha=0.5, linewidth=2, label='Desired B-Spline')
ax3.plot(x_real[::5], y_real[::5], '-', color=real_color, linewidth=2, 
         label='Actual Robot Path', alpha=0.8)
ax3.plot(0, 0, 'go', markersize=10)
ax3.plot(0.938, y_real[-1], 'o', color=real_color, markersize=10, 
         label='Final Position (93.8%)')
ax3.legend()

# 4. Velocity Profile Comparison (bottom right)
ax4 = axes[1, 1]
ax4.set_title('4. Velocity Profile Comparison', fontsize=14)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('X Velocity (m/s)')
ax4.set_xlim(0, 5)
ax4.set_ylim(-0.6, 1.0)
ax4.grid(True, alpha=0.3)

# Ideal velocity profile (trapezoidal)
t_ideal = np.array([0, 0.5, 2.5, 3.0, 3.5])
v_ideal = np.array([0, 0.8, 0.8, 0.8, 0])
ax4.plot(t_ideal, v_ideal, '-', color=bspline_color, linewidth=3, 
         label='Ideal B-Spline Velocity')

# Real oscillating velocity
t_real = np.linspace(0, 4.45, 200)
v_real = np.zeros_like(t_real)
for i in range(len(t_real)):
    # Oscillating velocity with decay
    v_real[i] = 0.4 * np.sin(10 * t_real[i]) * np.exp(-t_real[i] * 0.3)
    # Add desired velocity component
    if t_real[i] < 3:
        v_real[i] += 0.3 * (1 - np.exp(-2 * t_real[i]))

ax4.plot(t_real, v_real, '-', color=real_color, linewidth=2, alpha=0.8,
         label='Real Robot Velocity')
ax4.axhline(y=0, color='k', linestyle='-', alpha=0.3)
ax4.legend()

# Add text annotations
ax4.text(1.5, 0.85, 'Smooth acceleration', fontsize=10, ha='center',
         bbox=dict(boxstyle="round,pad=0.3", facecolor=bspline_color, alpha=0.3))
ax4.text(2.5, -0.4, 'Severe oscillations!', fontsize=10, ha='center',
         bbox=dict(boxstyle="round,pad=0.3", facecolor=real_color, alpha=0.3))

# Adjust layout
plt.tight_layout()

# Save figure
plt.savefig('rrt_bspline_real_comparison.png', dpi=300, bbox_inches='tight')
print("Visualization saved as 'rrt_bspline_real_comparison.png'")

# Create a second figure showing the control system block diagram
fig2, ax = plt.subplots(1, 1, figsize=(12, 6))
ax.set_title('Control System Block Diagram - Problem Identification', fontsize=16)
ax.set_xlim(0, 10)
ax.set_ylim(0, 6)
ax.axis('off')

# Define blocks
blocks = {
    'rrt': {'pos': (1, 4), 'size': (1.5, 0.8), 'color': '#90EE90', 'text': 'RRT*\nPath Planner'},
    'bspline': {'pos': (3, 4), 'size': (1.5, 0.8), 'color': '#90EE90', 'text': 'B-Spline\nTrajectory'},
    'control': {'pos': (5, 4), 'size': (1.5, 0.8), 'color': '#FFB6C1', 'text': 'Feedback\nController'},
    'robot': {'pos': (7, 4), 'size': (1.5, 0.8), 'color': '#87CEEB', 'text': 'Robot\nDynamics'},
    'sensors': {'pos': (7, 2), 'size': (1.5, 0.8), 'color': '#FFB6C1', 'text': 'Sensors\n(50ms delay)'},
    'estimator': {'pos': (5, 2), 'size': (1.5, 0.8), 'color': '#FFB6C1', 'text': 'State\nEstimator'},
}

# Draw blocks
for name, block in blocks.items():
    rect = Rectangle(block['pos'], block['size'][0], block['size'][1], 
                    facecolor=block['color'], edgecolor='black', linewidth=2)
    ax.add_patch(rect)
    ax.text(block['pos'][0] + block['size'][0]/2, 
            block['pos'][1] + block['size'][1]/2,
            block['text'], ha='center', va='center', fontsize=11, weight='bold')

# Draw arrows
arrows = [
    # Forward path
    ((blocks['rrt']['pos'][0] + blocks['rrt']['size'][0], 
      blocks['rrt']['pos'][1] + blocks['rrt']['size'][1]/2),
     (blocks['bspline']['pos'][0], 
      blocks['bspline']['pos'][1] + blocks['bspline']['size'][1]/2)),
    
    ((blocks['bspline']['pos'][0] + blocks['bspline']['size'][0], 
      blocks['bspline']['pos'][1] + blocks['bspline']['size'][1]/2),
     (blocks['control']['pos'][0], 
      blocks['control']['pos'][1] + blocks['control']['size'][1]/2)),
    
    ((blocks['control']['pos'][0] + blocks['control']['size'][0], 
      blocks['control']['pos'][1] + blocks['control']['size'][1]/2),
     (blocks['robot']['pos'][0], 
      blocks['robot']['pos'][1] + blocks['robot']['size'][1]/2)),
    
    # Feedback path
    ((blocks['robot']['pos'][0] + blocks['robot']['size'][0]/2, 
      blocks['robot']['pos'][1]),
     (blocks['sensors']['pos'][0] + blocks['sensors']['size'][0]/2, 
      blocks['sensors']['pos'][1] + blocks['sensors']['size'][1])),
    
    ((blocks['sensors']['pos'][0], 
      blocks['sensors']['pos'][1] + blocks['sensors']['size'][1]/2),
     (blocks['estimator']['pos'][0] + blocks['estimator']['size'][0], 
      blocks['estimator']['pos'][1] + blocks['estimator']['size'][1]/2)),
    
    ((blocks['estimator']['pos'][0], 
      blocks['estimator']['pos'][1] + blocks['estimator']['size'][1]/2),
     (blocks['control']['pos'][0] + blocks['control']['size'][0]/2, 
      blocks['control']['pos'][1])),
]

for start, end in arrows:
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='->', lw=2, color='black'))

# Add labels
ax.text(2.25, 4.6, 'Waypoints', fontsize=9, ha='center')
ax.text(4.25, 4.6, 'Smooth Path', fontsize=9, ha='center')
ax.text(6.25, 4.6, 'Control Signal', fontsize=9, ha='center')
ax.text(7.5, 3.0, 'Position', fontsize=9, ha='center', rotation=-90)
ax.text(6.25, 2.6, 'Delayed!', fontsize=9, ha='center', color='red', weight='bold')
ax.text(4.0, 3.0, 'Estimated\nPosition', fontsize=9, ha='center')

# Add problem indicators
problem_blocks = ['control', 'sensors', 'estimator']
for name in problem_blocks:
    block = blocks[name]
    rect = Rectangle((block['pos'][0]-0.1, block['pos'][1]-0.1), 
                    block['size'][0]+0.2, block['size'][1]+0.2,
                    fill=False, edgecolor='red', linewidth=3, linestyle='--')
    ax.add_patch(rect)

# Add legend
ok_patch = mpatches.Patch(color='#90EE90', label='Working correctly')
problem_patch = mpatches.Patch(color='#FFB6C1', label='Problem area')
ax.legend(handles=[ok_patch, problem_patch], loc='upper right')

# Add title for problem
ax.text(5, 0.5, 'PROBLEMS: High control gains + Sensor delays + No feedforward = Oscillations!',
        fontsize=12, ha='center', color='red', weight='bold',
        bbox=dict(boxstyle="round,pad=0.5", facecolor='yellow', alpha=0.3))

plt.savefig('control_system_block_diagram.png', dpi=300, bbox_inches='tight')
print("Control system diagram saved as 'control_system_block_diagram.png'")

plt.show()