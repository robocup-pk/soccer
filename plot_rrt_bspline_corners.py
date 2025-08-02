#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys

def load_trajectory_data(filename):
    """Load trajectory data from CSV file"""
    rrt_points = []
    bspline_points = []
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    traj_type = parts[0]
                    x = float(parts[1])
                    y = float(parts[2])
                    theta = float(parts[3])
                    
                    if traj_type == 'RRT':
                        rrt_points.append([x, y, theta])
                    elif traj_type == 'BSPLINE':
                        bspline_points.append([x, y, theta])
    except FileNotFoundError:
        print(f"File {filename} not found. Generating example data...")
        # Generate example data to show the concept
        rrt_points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, np.pi/2],
            [0.0, 1.0, np.pi],
            [0.0, 0.0, -np.pi/2]
        ]
        
        # Simulate B-spline smoothing
        t = np.linspace(0, 1, 100)
        for i in range(len(rrt_points)-1):
            p1 = np.array(rrt_points[i][:2])
            p2 = np.array(rrt_points[i+1][:2])
            
            # Add smooth transition
            if i > 0 and i < len(rrt_points)-2:
                # Create smooth corner
                for j in range(len(t)//4):
                    s = t[j*4] * 0.25
                    if i == 1:  # First corner
                        pt = p1 + s * (p2 - p1)
                        # Smooth the corner
                        corner_factor = np.sin(s * np.pi) * 0.1
                        pt[1] += corner_factor
                    elif i == 2:  # Second corner
                        pt = p1 + s * (p2 - p1)
                        corner_factor = np.sin(s * np.pi) * 0.1
                        pt[0] -= corner_factor
                    else:
                        pt = p1 + s * (p2 - p1)
                    bspline_points.append([pt[0], pt[1], 0])
            else:
                # Straight segments
                for s in t[::4]:
                    pt = p1 + s * (p2 - p1)
                    bspline_points.append([pt[0], pt[1], 0])
    
    return np.array(rrt_points), np.array(bspline_points)

def plot_trajectories(rrt_points, bspline_points, test_case=1):
    """Create visualization of RRT* waypoints and B-spline trajectory"""
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot 1: Overview
    ax1.set_title('RRT* Waypoints vs B-Spline Trajectory', fontsize=14, fontweight='bold')
    
    # Plot RRT* waypoints
    if len(rrt_points) > 0:
        ax1.plot(rrt_points[:, 0], rrt_points[:, 1], 'ro-', 
                linewidth=2, markersize=10, label='RRT* Waypoints', zorder=3)
        
        # Add waypoint numbers
        for i, (x, y, _) in enumerate(rrt_points):
            ax1.annotate(f'{i}', (x, y), xytext=(5, 5), textcoords='offset points',
                        fontsize=10, fontweight='bold')
    
    # Plot B-spline trajectory
    if len(bspline_points) > 0:
        ax1.plot(bspline_points[:, 0], bspline_points[:, 1], 'b-', 
                linewidth=3, label='B-Spline Trajectory', alpha=0.7, zorder=2)
        
        # Add direction arrows
        step = max(1, len(bspline_points) // 20)
        for i in range(0, len(bspline_points)-step, step):
            dx = bspline_points[i+step, 0] - bspline_points[i, 0]
            dy = bspline_points[i+step, 1] - bspline_points[i, 1]
            ax1.arrow(bspline_points[i, 0], bspline_points[i, 1], 
                     dx*0.5, dy*0.5, head_width=0.03, head_length=0.02, 
                     fc='blue', ec='blue', alpha=0.5)
    
    ax1.set_xlabel('X Position (m)', fontsize=12)
    ax1.set_ylabel('Y Position (m)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=11, loc='best')
    ax1.axis('equal')
    ax1.set_xlim(-0.5, 2.5)
    ax1.set_ylim(-0.5, 2.5)
    
    # Plot 2: Corner Analysis
    ax2.set_title('Corner Handling Analysis', fontsize=14, fontweight='bold')
    
    # Find corners (where direction changes significantly)
    if len(rrt_points) >= 3:
        corners = []
        for i in range(1, len(rrt_points)-1):
            v1 = rrt_points[i] - rrt_points[i-1]
            v2 = rrt_points[i+1] - rrt_points[i]
            angle = np.arccos(np.clip(np.dot(v1[:2], v2[:2]) / 
                                     (np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2])), -1, 1))
            if angle > np.pi/6:  # More than 30 degrees
                corners.append(i)
        
        # Zoom in on first corner
        if corners:
            corner_idx = corners[0]
            cx, cy = rrt_points[corner_idx, 0], rrt_points[corner_idx, 1]
            
            # Plot zoomed view
            zoom = 0.3
            ax2.set_xlim(cx - zoom, cx + zoom)
            ax2.set_ylim(cy - zoom, cy + zoom)
            
            # Plot RRT* path
            ax2.plot(rrt_points[:, 0], rrt_points[:, 1], 'ro-', 
                    linewidth=2, markersize=12, label='Sharp Corner (RRT*)', zorder=3)
            
            # Plot B-spline
            if len(bspline_points) > 0:
                ax2.plot(bspline_points[:, 0], bspline_points[:, 1], 'b-', 
                        linewidth=4, label='Smooth Curve (B-Spline)', alpha=0.7, zorder=2)
            
            # Highlight corner cutting
            ax2.fill_between([rrt_points[corner_idx-1, 0], rrt_points[corner_idx, 0], 
                             rrt_points[corner_idx+1, 0]],
                            [rrt_points[corner_idx-1, 1], rrt_points[corner_idx, 1], 
                             rrt_points[corner_idx+1, 1]],
                            alpha=0.2, color='red', label='Corner Cut Area')
            
            # Add annotations
            ax2.annotate('Corner Cut', xy=(cx, cy), xytext=(cx+0.1, cy+0.1),
                        arrowprops=dict(arrowstyle='->', color='red', lw=2),
                        fontsize=12, fontweight='bold', color='red')
    
    ax2.set_xlabel('X Position (m)', fontsize=12)
    ax2.set_ylabel('Y Position (m)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=11, loc='best')
    ax2.axis('equal')
    
    # Add main title
    fig.suptitle(f'B-Spline Corner Handling Analysis - Test Case {test_case}', 
                 fontsize=16, fontweight='bold')
    
    # Add text explanation
    fig.text(0.5, 0.02, 
             'B-spline creates smooth curves by "cutting corners" - trading exact waypoint following for smooth motion',
             ha='center', fontsize=12, style='italic')
    
    plt.tight_layout()
    return fig

def main():
    if len(sys.argv) > 1:
        test_case = int(sys.argv[1])
    else:
        test_case = 1
    
    filename = f"rrt_bspline_trajectory_test{test_case}.csv"
    
    print(f"Loading trajectory data from {filename}...")
    rrt_points, bspline_points = load_trajectory_data(filename)
    
    print(f"Loaded {len(rrt_points)} RRT* waypoints and {len(bspline_points)} B-spline points")
    
    # Create visualization
    fig = plot_trajectories(rrt_points, bspline_points, test_case)
    
    # Save the plot
    output_file = f"bspline_corner_analysis_test{test_case}.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {output_file}")
    
    plt.show()

if __name__ == "__main__":
    main()