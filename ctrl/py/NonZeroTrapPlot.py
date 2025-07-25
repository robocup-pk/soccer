import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as patches

def check_feasibility(v0, total_distance, total_time, v_limit, a_limit):
    """
    Check if the trajectory is feasible given system limits
    
    Returns:
    feasible: bool - whether trajectory is feasible
    reason: str - reason if not feasible
    """
    
    # Check if initial velocity exceeds limit
    if abs(v0) > v_limit:
        return False, f"Initial velocity {v0:.2f} exceeds limit ±{v_limit:.2f}"
    
    # Check if average velocity required exceeds limit
    avg_velocity_required = total_distance / total_time
    if abs(avg_velocity_required) > v_limit:
        return False, f"Average velocity required {avg_velocity_required:.2f} exceeds limit ±{v_limit:.2f}"
    
    # For minimum time check - if we need to go from v0 to 0 in time T covering distance D
    # We need to check if there exists a feasible trapezoidal profile
    
    # Quick feasibility check: can we reach the required average velocity?
    # For a trapezoidal profile, the peak velocity will be higher than average
    # Rough estimate: if |D/T| > 0.8 * v_limit, it might not be feasible
    
    # More rigorous check: calculate minimum time needed
    if total_distance == 0:
        # Special case: just need to decelerate to zero
        min_time = abs(v0) / a_limit
        if total_time < min_time:
            return False, f"Minimum time to decelerate from {v0:.2f} to 0 is {min_time:.2f}s"
        return True, "Feasible"
    
    # General case: estimate if trajectory is feasible
    # This is a simplified check - the actual calculation is complex
    direction = 1 if total_distance > 0 else -1
    
    # If we're going in opposite direction to initial velocity, we need more time
    if np.sign(v0) != 0 and np.sign(total_distance) != 0 and np.sign(v0) != np.sign(total_distance):
        # Need to first decelerate to zero, then accelerate in other direction
        time_to_zero = abs(v0) / a_limit
        remaining_time = total_time - time_to_zero
        remaining_distance = total_distance
        
        if remaining_time <= 0:
            return False, f"Not enough time to change direction. Need at least {time_to_zero:.2f}s to stop"
        
        # Check if remaining distance can be covered in remaining time
        max_distance_remaining = 0.5 * a_limit * remaining_time**2
        if abs(remaining_distance) > max_distance_remaining:
            return False, f"Cannot cover remaining distance {remaining_distance:.2f} in time {remaining_time:.2f}"
    
    return True, "Feasible"

def calculate_trapezoidal_trajectory(v0, total_distance, total_time, v_limit, a_limit):
    """
    Calculate trapezoidal velocity profile parameters with system limits
    
    Parameters:
    v0: initial velocity (can be negative)
    total_distance: total distance to cover (can be negative)
    total_time: total time for the trajectory
    v_limit: maximum velocity magnitude
    a_limit: maximum acceleration magnitude
    
    Returns:
    t1: time to reach maximum velocity
    t2: time when deceleration starts
    v_max: maximum velocity reached
    a1: acceleration in phase 1
    a3: acceleration in phase 3 (deceleration)
    feasible: whether trajectory is feasible
    """
    
    # Check feasibility first
    feasible, reason = check_feasibility(v0, total_distance, total_time, v_limit, a_limit)
    if not feasible:
        return None, None, None, None, None, False, reason
    
    # Special case: zero distance
    if abs(total_distance) < 1e-6:
        # Just need to decelerate to zero
        a3 = -v0 / total_time
        if abs(a3) > a_limit:
            return None, None, None, None, None, False, f"Required deceleration {abs(a3):.2f} exceeds limit {a_limit:.2f}"
        return 0, 0, v0, 0, a3, True, "Feasible"
    
    # Determine direction of motion
    direction = 1 if total_distance > 0 else -1
    
    # Try to find feasible trapezoidal profile
    best_solution = None
    best_error = float('inf')
    
    # Try different strategies
    for strategy in ['optimal', 'max_accel', 'triangular']:
        try:
            if strategy == 'triangular':
                # Triangular profile (no constant velocity phase)
                # Solve: v0*T + 0.5*a1*t1^2 + 0.5*a3*t3^2 = D
                # where t1 + t3 = T, v_max = v0 + a1*t1, 0 = v_max + a3*t3
                
                # For symmetric case: t1 = t3 = T/2
                t1 = t3 = total_time / 2
                t2 = total_time / 2
                
                # From distance equation
                a1 = (2 * total_distance - 2 * v0 * total_time) / (total_time**2)
                v_max = v0 + a1 * t1
                a3 = -v_max / t3
                
            elif strategy == 'max_accel':
                # Use maximum allowed acceleration
                a1 = direction * a_limit
                
                # Calculate when we reach v_limit or need to start decelerating
                t_to_limit = (direction * v_limit - v0) / a1 if direction * v_limit > v0 * direction else 0
                
                # Calculate minimum deceleration time
                if direction * v_limit > v0 * direction:
                    v_peak = direction * v_limit
                    t_decel_min = abs(v_peak) / a_limit
                else:
                    v_peak = v0
                    t_decel_min = abs(v_peak) / a_limit
                
                # Check if we have enough time
                if t_to_limit + t_decel_min > total_time:
                    # Use triangular profile instead
                    continue
                
                t1 = t_to_limit
                t2 = total_time - t_decel_min
                v_max = v_peak
                a3 = -v_max / (total_time - t2)
                
            else:  # optimal
                # Try to find optimal solution
                # Use numerical approach to find t1, t2 that satisfy constraints
                
                for t1_frac in np.linspace(0.1, 0.4, 20):
                    t1 = t1_frac * total_time
                    for t2_frac in np.linspace(0.6, 0.9, 20):
                        t2 = t2_frac * total_time
                        
                        if t1 >= t2:
                            continue
                        
                        t3 = total_time - t2
                        
                        # Calculate required accelerations
                        # Using the constraint equations:
                        # D = v0*t1 + 0.5*a1*t1^2 + v_max*(t2-t1) + v_max*t3 + 0.5*a3*t3^2
                        # v_max = v0 + a1*t1
                        # 0 = v_max + a3*t3
                        
                        # From the last equation: a3 = -v_max/t3
                        # Substituting into distance equation and solving for a1:
                        denominator = t1 * (t1 + 2*(t2 - t1) + 2*t3)
                        if abs(denominator) < 1e-10:
                            continue
                            
                        a1 = (2 * total_distance - 2 * v0 * total_time) / denominator
                        v_max = v0 + a1 * t1
                        a3 = -v_max / t3
                        
                        # Check constraints
                        if (abs(v_max) <= v_limit and abs(a1) <= a_limit and 
                            abs(a3) <= a_limit and v_max * direction >= v0 * direction):
                            
                            # Verify the solution
                            d_calc = (v0 * t1 + 0.5 * a1 * t1**2 + 
                                    v_max * (t2 - t1) + 
                                    v_max * t3 + 0.5 * a3 * t3**2)
                            v_final = v_max + a3 * t3
                            
                            error = abs(d_calc - total_distance) + abs(v_final)
                            
                            if error < best_error:
                                best_error = error
                                best_solution = (t1, t2, v_max, a1, a3)
            
            # Check if current strategy produced a valid solution
            if strategy != 'optimal':
                if (abs(v_max) <= v_limit and abs(a1) <= a_limit and abs(a3) <= a_limit):
                    # Verify the solution
                    d_calc = (v0 * t1 + 0.5 * a1 * t1**2 + 
                            v_max * (t2 - t1) + 
                            v_max * (total_time - t2) + 0.5 * a3 * (total_time - t2)**2)
                    v_final = v_max + a3 * (total_time - t2)
                    
                    error = abs(d_calc - total_distance) + abs(v_final)
                    
                    if error < best_error:
                        best_error = error
                        best_solution = (t1, t2, v_max, a1, a3)
        
        except:
            continue
    
    if best_solution is None:
        return None, None, None, None, None, False, "No feasible solution found"
    
    if best_error > 0.1:  # If error is too high
        return None, None, None, None, None, False, f"Solution error too high: {best_error:.3f}"
    
    t1, t2, v_max, a1, a3 = best_solution
    return t1, t2, v_max, a1, a3, True, "Feasible"

def generate_trajectory(v0, total_distance, total_time, v_limit, a_limit, num_points=1000):
    """Generate time, velocity, and position arrays for the trajectory"""
    
    result = calculate_trapezoidal_trajectory(v0, total_distance, total_time, v_limit, a_limit)
    t1, t2, v_max, a1, a3, feasible, reason = result
    
    if not feasible:
        # Return empty trajectory
        time = np.linspace(0, total_time, num_points)
        velocity = np.zeros(num_points)
        position = np.zeros(num_points)
        return time, velocity, position, None, None, None, feasible, reason
    
    time = np.linspace(0, total_time, num_points)
    velocity = np.zeros(num_points)
    position = np.zeros(num_points)
    
    for i, t in enumerate(time):
        if t <= t1:
            # Acceleration phase
            velocity[i] = v0 + a1 * t
            position[i] = v0 * t + 0.5 * a1 * t**2
        elif t <= t2:
            # Constant velocity phase
            velocity[i] = v_max
            d1 = v0 * t1 + 0.5 * a1 * t1**2
            position[i] = d1 + v_max * (t - t1)
        else:
            # Deceleration phase
            velocity[i] = v_max + a3 * (t - t2)
            d1 = v0 * t1 + 0.5 * a1 * t1**2
            d2 = v_max * (t2 - t1)
            position[i] = d1 + d2 + v_max * (t - t2) + 0.5 * a3 * (t - t2)**2
    
    return time, velocity, position, t1, t2, v_max, feasible, reason

# Create the main plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 12))
plt.subplots_adjust(bottom=0.35)

# Initial parameters
v0_init = 5.0
distance_init = 100.0
time_init = 10.0
v_limit_init = 20.0
a_limit_init = 5.0

# Generate initial trajectory
time, velocity, position, t1, t2, v_max, feasible, reason = generate_trajectory(
    v0_init, distance_init, time_init, v_limit_init, a_limit_init)

# Plot initial trajectory
line1, = ax1.plot(time, velocity, 'b-', linewidth=2, label='Velocity')
ax1.axhline(y=v_limit_init, color='r', linestyle='--', alpha=0.7, label=f'Velocity Limit = ±{v_limit_init}')
ax1.axhline(y=-v_limit_init, color='r', linestyle='--', alpha=0.7)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Trapezoidal Velocity Profile with System Limits')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Add phase indicators (only if feasible)
phase1_patch = None
phase2_patch = None  
phase3_patch = None

line2, = ax2.plot(time, position, 'r-', linewidth=2, label='Position')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position (m)')
ax2.set_title('Position vs Time')
ax2.grid(True, alpha=0.3)
ax2.legend()

# Create sliders
ax_v0 = plt.axes([0.1, 0.25, 0.8, 0.02])
ax_distance = plt.axes([0.1, 0.22, 0.8, 0.02])
ax_time = plt.axes([0.1, 0.19, 0.8, 0.02])
ax_v_limit = plt.axes([0.1, 0.16, 0.8, 0.02])
ax_a_limit = plt.axes([0.1, 0.13, 0.8, 0.02])

slider_v0 = Slider(ax_v0, 'Initial Velocity (m/s)', -30.0, 30.0, valinit=v0_init)
slider_distance = Slider(ax_distance, 'Total Distance (m)', -500.0, 500.0, valinit=distance_init)
slider_time = Slider(ax_time, 'Total Time (s)', 1.0, 30.0, valinit=time_init)
slider_v_limit = Slider(ax_v_limit, 'Velocity Limit (m/s)', 1.0, 50.0, valinit=v_limit_init)
slider_a_limit = Slider(ax_a_limit, 'Acceleration Limit (m/s²)', 0.5, 20.0, valinit=a_limit_init)

# Status text
status_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12, 
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.9))

# Update function
def update(val):
    global phase1_patch, phase2_patch, phase3_patch
    
    v0 = slider_v0.val
    distance = slider_distance.val
    total_time = slider_time.val
    v_limit = slider_v_limit.val
    a_limit = slider_a_limit.val
    
    # Generate new trajectory
    time, velocity, position, t1, t2, v_max, feasible, reason = generate_trajectory(
        v0, distance, total_time, v_limit, a_limit)
    
    # Update plots
    line1.set_data(time, velocity)
    line2.set_data(time, position)
    
    # Update velocity limit lines
    ax1.lines[1].set_ydata([v_limit, v_limit])
    ax1.lines[2].set_ydata([-v_limit, -v_limit])
    
    # Update axis limits
    ax1.set_xlim(0, total_time)
    ax1.set_ylim(min(-v_limit*1.2, min(velocity)*1.1), max(v_limit*1.2, max(velocity)*1.1))
    
    ax2.set_xlim(0, total_time)
    if len(position) > 0 and max(position) != min(position):
        ax2.set_ylim(min(position)*1.1, max(position)*1.1)
    else:
        ax2.set_ylim(-abs(distance)*0.1, abs(distance)*0.1)
    
    # Remove old phase patches
    if phase1_patch is not None:
        phase1_patch.remove()
    if phase2_patch is not None:
        phase2_patch.remove()
    if phase3_patch is not None:
        phase3_patch.remove()
    
    # Update status and add patches if feasible
    if feasible and t1 is not None:
        status_text.set_text(f'STATUS: FEASIBLE')
        status_text.set_color('green')
        
        # Add new phase patches
        y_min, y_max = ax1.get_ylim()
        height = y_max - y_min
        
        if t1 > 0:
            phase1_patch = patches.Rectangle((0, y_min), t1, height, alpha=0.1, facecolor='green')
            ax1.add_patch(phase1_patch)
        
        if t2 > t1:
            phase2_patch = patches.Rectangle((t1, y_min), t2-t1, height, alpha=0.1, facecolor='blue')
            ax1.add_patch(phase2_patch)
        
        if total_time > t2:
            phase3_patch = patches.Rectangle((t2, y_min), total_time-t2, height, alpha=0.1, facecolor='red')
            ax1.add_patch(phase3_patch)
        
        # Add parameter text
        params_text = (f'Max Velocity: {v_max:.2f} m/s\n'
                      f'Accel Time: {t1:.2f} s\n'
                      f'Const Time: {t2-t1:.2f} s\n'
                      f'Decel Time: {total_time-t2:.2f} s')
        
        ax1.text(0.02, 0.75, params_text, transform=ax1.transAxes, fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
        
        ax2.text(0.02, 0.88, f'Final Position: {position[-1]:.2f} m', transform=ax2.transAxes, fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
        ax2.text(0.02, 0.82, f'Final Velocity: {velocity[-1]:.3f} m/s', transform=ax2.transAxes, fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
    else:
        status_text.set_text(f'STATUS: NOT FEASIBLE\n{reason}')
        status_text.set_color('red')
        
        # Clear any parameter text
        ax1.text(0.02, 0.75, '', transform=ax1.transAxes, fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.8))
        
        ax2.text(0.02, 0.88, 'Trajectory not feasible', transform=ax2.transAxes, fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.8))
    
    plt.draw()

# Connect sliders to update function
slider_v0.on_changed(update)
slider_distance.on_changed(update)
slider_time.on_changed(update)
slider_v_limit.on_changed(update)
slider_a_limit.on_changed(update)

# Initial update
update(None)

plt.tight_layout()
plt.show()