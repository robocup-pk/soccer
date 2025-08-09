#!/usr/bin/env python3
"""
Plot Kalman Filter internals from the detailed log
Shows how P-matrix, Kalman gain, and innovation evolve over time
"""

import numpy as np
import matplotlib.pyplot as plt
import re

def parse_kalman_log(filename='kalman_detailed_log.txt'):
    """Parse the detailed Kalman filter log"""
    
    data = {
        'predictions': [],
        'updates': [],
    }
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i]
        
        # Parse prediction steps
        if 'PREDICTION STEP (Encoder Update)' in line:
            match = re.search(r'#(\d+)', line)
            if match:
                step_num = int(match.group(1))
                # Read next lines for position and P-matrix
                i += 1
                if i < len(lines) and 'Position after prediction:' in lines[i]:
                    pos_match = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[i])
                    if len(pos_match) >= 3:
                        position = [float(x) for x in pos_match[:3]]
                        
                        # Read P-matrix (next 3 lines)
                        p_matrix = []
                        for j in range(3):
                            i += 1
                            if i < len(lines):
                                p_row = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[i])
                                if len(p_row) >= 3:
                                    p_matrix.append([float(x) for x in p_row[:3]])
                        
                        if len(p_matrix) == 3:
                            data['predictions'].append({
                                'step': step_num,
                                'position': position,
                                'P': p_matrix
                            })
        
        # Parse update steps
        elif 'UPDATE STEP (Camera Measurement)' in line:
            match = re.search(r'#(\d+)', line)
            if match:
                step_num = int(match.group(1))
                update_data = {'step': step_num}
                
                # Parse camera input, innovation, Kalman gain, etc.
                for j in range(1, min(30, len(lines) - i)):
                    next_line = lines[i + j]
                    
                    if 'Camera Input:' in next_line:
                        vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', next_line)
                        if len(vals) >= 3:
                            update_data['camera_input'] = [float(x) for x in vals[:3]]
                    
                    elif 'Position before update:' in next_line:
                        vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', next_line)
                        if len(vals) >= 3:
                            update_data['pos_before'] = [float(x) for x in vals[:3]]
                    
                    elif 'Innovation' in next_line and 'Covariance' not in next_line:
                        vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', next_line)
                        if len(vals) >= 3:
                            update_data['innovation'] = [float(x) for x in vals[:3]]
                    
                    elif 'Corrected Position:' in next_line:
                        vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', next_line)
                        if len(vals) >= 3:
                            update_data['pos_after'] = [float(x) for x in vals[:3]]
                    
                    elif 'P-Matrix after update:' in next_line:
                        # Read next 3 lines for P-matrix
                        p_matrix = []
                        for k in range(1, 4):  # Start from 1 to skip the label line
                            if i + j + k < len(lines):
                                p_row = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[i + j + k])
                                if len(p_row) >= 3:
                                    p_matrix.append([float(x) for x in p_row[:3]])
                        if len(p_matrix) == 3:
                            update_data['P_after'] = p_matrix
                        break
                
                if 'pos_after' in update_data:
                    data['updates'].append(update_data)
        
        i += 1
    
    return data

def plot_kalman_evolution(data):
    """Plot the evolution of Kalman filter parameters"""
    
    fig = plt.figure(figsize=(16, 10))
    
    # Extract update data
    updates = data['updates']
    if not updates:
        print("No update data found!")
        return
    
    steps = [u['step'] for u in updates]
    
    # Extract P-matrix diagonal elements (uncertainties)
    P_xx = []
    P_yy = []
    P_theta = []
    innovations = []
    
    for u in updates:
        if 'P_after' in u:
            P = u['P_after']
            P_xx.append(P[0][0])
            P_yy.append(P[1][1])
            P_theta.append(P[2][2])
        
        if 'innovation' in u:
            innovations.append(u['innovation'])
    
    # Convert to numpy arrays
    P_xx = np.array(P_xx)
    P_yy = np.array(P_yy)
    P_theta = np.array(P_theta)
    innovations = np.array(innovations)
    
    # 1. P-matrix diagonal evolution
    ax1 = plt.subplot(3, 2, 1)
    ax1.semilogy(steps[:len(P_xx)], P_xx, 'b-', label='P_xx (m²)')
    ax1.semilogy(steps[:len(P_yy)], P_yy, 'g-', label='P_yy (m²)')
    ax1.set_xlabel('Update Step')
    ax1.set_ylabel('Variance (log scale)')
    ax1.set_title('Position Uncertainty Evolution')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Orientation uncertainty
    ax2 = plt.subplot(3, 2, 2)
    ax2.semilogy(steps[:len(P_theta)], P_theta, 'r-', label='P_θθ (rad²)')
    ax2.set_xlabel('Update Step')
    ax2.set_ylabel('Variance (log scale)')
    ax2.set_title('Orientation Uncertainty Evolution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Innovation magnitude over time
    if len(innovations) > 0:
        ax3 = plt.subplot(3, 2, 3)
        innovation_norm = np.sqrt(innovations[:, 0]**2 + innovations[:, 1]**2)
        ax3.plot(steps[:len(innovation_norm)], innovation_norm * 1000, 'b-', alpha=0.7)
        ax3.set_xlabel('Update Step')
        ax3.set_ylabel('Position Innovation (mm)')
        ax3.set_title('Innovation Magnitude (Measurement - Prediction)')
        ax3.grid(True, alpha=0.3)
        
        # 4. Innovation in theta
        ax4 = plt.subplot(3, 2, 4)
        ax4.plot(steps[:len(innovations)], np.degrees(innovations[:, 2]), 'r-', alpha=0.7)
        ax4.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax4.set_xlabel('Update Step')
        ax4.set_ylabel('Angle Innovation (degrees)')
        ax4.set_title('Orientation Innovation')
        ax4.grid(True, alpha=0.3)
    
    # 5. Trace of P-matrix (total uncertainty)
    ax5 = plt.subplot(3, 2, 5)
    trace_P = P_xx + P_yy + P_theta
    ax5.semilogy(steps[:len(trace_P)], trace_P, 'purple', alpha=0.7)
    ax5.set_xlabel('Update Step')
    ax5.set_ylabel('Trace(P) (log scale)')
    ax5.set_title('Total Uncertainty (Trace of Covariance Matrix)')
    ax5.grid(True, alpha=0.3)
    
    # 6. Statistics panel
    ax6 = plt.subplot(3, 2, 6)
    ax6.axis('off')
    
    stats_text = f"""Kalman Filter Statistics:
    
Total Updates: {len(updates)}
Total Predictions: {len(data['predictions'])}

Final Uncertainties:
  σ_x: {np.sqrt(P_xx[-1])*1000:.2f} mm
  σ_y: {np.sqrt(P_yy[-1])*1000:.2f} mm
  σ_θ: {np.degrees(np.sqrt(P_theta[-1])):.2f} deg

Mean Innovation:
  Position: {np.mean(innovation_norm)*1000:.2f} mm
  Angle: {np.mean(np.abs(innovations[:, 2]))*180/np.pi:.2f} deg

Convergence:
  P_xx converged to: {P_xx[-1]:.2e} m²
  P_yy converged to: {P_yy[-1]:.2e} m²
  P_θθ converged to: {P_theta[-1]:.2e} rad²
"""
    
    ax6.text(0.1, 0.9, stats_text, fontsize=10, family='monospace',
            verticalalignment='top', transform=ax6.transAxes)
    
    plt.suptitle('Kalman Filter Internal Evolution\n(P-Matrix, Innovation, and Convergence)', 
                 fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    return fig

def main():
    """Main function"""
    
    print("=" * 60)
    print("Kalman Filter Internals Analysis")
    print("=" * 60)
    
    # Parse the log file
    data = parse_kalman_log('kalman_detailed_log.txt')
    
    print(f"Found {len(data['predictions'])} prediction steps")
    print(f"Found {len(data['updates'])} update steps")
    
    if len(data['updates']) == 0:
        print("No update data found in log file!")
        return
    
    # Create plots
    fig = plot_kalman_evolution(data)
    
    # Save figure
    output_file = 'kalman_internals_analysis.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlots saved to '{output_file}'")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()