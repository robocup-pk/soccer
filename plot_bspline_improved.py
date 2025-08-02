#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys

def load_improved_trajectory_data(filename):
    """Load trajectory data with original, processed, and B-spline points"""
    original_points = []
    processed_points = []
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
                    
                    if traj_type == 'ORIGINAL':
                        original_points.append([x, y, theta])
                    elif traj_type == 'PROCESSED':
                        processed_points.append([x, y, theta])
                    elif traj_type == 'BSPLINE':
                        bspline_points.append([x, y, theta])
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return None, None, None
    
    return np.array(original_points), np.array(processed_points), np.array(bspline_points)

def plot_improved_trajectories(original, processed, bspline, test_case=1):
    """Create improved visualization comparing corner handling"""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # Plot 1: Original vs B-spline (showing the problem)
    ax1 = axes[0, 0]
    ax1.set_title('Original: "Water Drop" Problem at Corners', fontsize=14, fontweight='bold')
    
    if len(original) > 0:
        ax1.plot(original[:, 0], original[:, 1], 'ro-', 
                linewidth=2, markersize=10, label='Sharp Corners', zorder=3)
        
        # Annotate corners
        for i in range(len(original)):
            ax1.annotate(f'W{i}', (original[i, 0], original[i, 1]), 
                        xytext=(5, 5), textcoords='offset points', fontsize=10)
    
    if len(bspline) > 0:
        # Show only a simple B-spline on original waypoints
        # This would show the "water drop" effect
        ax1.plot(bspline[:, 0], bspline[:, 1], 'b--', 
                linewidth=2, label='B-Spline (drops at corners)', alpha=0.7)
    
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # Plot 2: Processed waypoints solution
    ax2 = axes[0, 1]
    ax2.set_title('Solution: Add Corner Waypoints', fontsize=14, fontweight='bold')
    
    if len(original) > 0:
        ax2.plot(original[:, 0], original[:, 1], 'ro-', 
                linewidth=1, markersize=8, label='Original', alpha=0.5, zorder=2)
    
    if len(processed) > 0:
        ax2.plot(processed[:, 0], processed[:, 1], 'go-', 
                linewidth=2, markersize=6, label='Added Corner Points', zorder=3)
        
        # Highlight added points
        if len(processed) > len(original):
            for i, pt in enumerate(processed):
                ax2.plot(pt[0], pt[1], 'g*', markersize=12, alpha=0.7)
    
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.axis('equal')
    
    # Plot 3: Final improved result
    ax3 = axes[1, 0]
    ax3.set_title('Improved B-Spline: Smooth Corners Without Drops', fontsize=14, fontweight='bold')
    
    if len(processed) > 0:
        ax3.plot(processed[:, 0], processed[:, 1], 'go', 
                markersize=6, label='Control Points', alpha=0.5, zorder=2)
    
    if len(bspline) > 0:
        ax3.plot(bspline[:, 0], bspline[:, 1], 'b-', 
                linewidth=3, label='Smooth B-Spline', zorder=3)
        
        # Add direction arrows
        step = max(1, len(bspline) // 15)
        for i in range(0, len(bspline)-step, step*2):
            dx = bspline[i+step, 0] - bspline[i, 0]
            dy = bspline[i+step, 1] - bspline[i, 1]
            ax3.arrow(bspline[i, 0], bspline[i, 1], 
                     dx*0.8, dy*0.8, head_width=0.02, head_length=0.015, 
                     fc='blue', ec='blue', alpha=0.6)
    
    ax3.set_xlabel('X (m)', fontsize=12)
    ax3.set_ylabel('Y (m)', fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.axis('equal')
    
    # Plot 4: Close-up of corner
    ax4 = axes[1, 1]
    ax4.set_title('Corner Close-up: Smooth Transition', fontsize=14, fontweight='bold')
    
    # Find a corner to zoom in on
    if len(original) >= 3:
        # Focus on the first corner
        corner_idx = 1
        cx, cy = original[corner_idx, 0], original[corner_idx, 1]
        
        zoom = 0.3
        ax4.set_xlim(cx - zoom, cx + zoom)
        ax4.set_ylim(cy - zoom, cy + zoom)
        
        # Plot original corner
        ax4.plot([original[corner_idx-1, 0], original[corner_idx, 0], original[corner_idx+1, 0]],
                [original[corner_idx-1, 1], original[corner_idx, 1], original[corner_idx+1, 1]],
                'r--', linewidth=2, label='Sharp Corner', alpha=0.5)
        
        # Plot processed points
        if len(processed) > 0:
            ax4.plot(processed[:, 0], processed[:, 1], 'go', markersize=8, label='Added Points')
        
        # Plot smooth B-spline
        if len(bspline) > 0:
            ax4.plot(bspline[:, 0], bspline[:, 1], 'b-', 
                    linewidth=4, label='Smooth Path', alpha=0.8)
        
        # Add annotation
        ax4.annotate('Smooth corner\n(no drop!)', 
                    xy=(cx-0.05, cy-0.05), xytext=(cx-0.15, cy-0.15),
                    arrowprops=dict(arrowstyle='->', color='green', lw=2),
                    fontsize=11, fontweight='bold', color='green',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
    
    ax4.set_xlabel('X (m)', fontsize=12)
    ax4.set_ylabel('Y (m)', fontsize=12)
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.axis('equal')
    
    # Main title
    fig.suptitle(f'B-Spline Corner Improvement - Test Case {test_case}', 
                 fontsize=16, fontweight='bold')
    
    # Add explanation
    fig.text(0.5, 0.02, 
             'Solution: Add waypoints before and after corners to guide B-spline smoothly through turns',
             ha='center', fontsize=12, style='italic')
    
    plt.tight_layout()
    return fig

def main():
    if len(sys.argv) > 1:
        test_case = int(sys.argv[1])
    else:
        test_case = 1
    
    filename = f"improved_bspline_test{test_case}.csv"
    
    print(f"Loading improved trajectory data from {filename}...")
    original, processed, bspline = load_improved_trajectory_data(filename)
    
    if original is None:
        print("No data found. Run the demo first.")
        return
    
    print(f"Loaded {len(original)} original, {len(processed)} processed, {len(bspline)} B-spline points")
    
    # Create visualization
    fig = plot_improved_trajectories(original, processed, bspline, test_case)
    
    # Save the plot
    output_file = f"bspline_corner_improved_test{test_case}.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {output_file}")
    
    plt.show()

if __name__ == "__main__":
    main()