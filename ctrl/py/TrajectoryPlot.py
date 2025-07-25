import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Initial values
init_a_max = 1.0
init_distance = 5.0
init_t_total = 5.0
init_v_limit = 3.0  # physical max velocity limit

def compute_vmax(distance, a_max, t_total):
    discrim = t_total**2 - 4 * distance / a_max
    if discrim < 0:
        return None, None, None
    sqrt_disc = np.sqrt(discrim)
    t_accel1 = (t_total - sqrt_disc) / 2
    t_accel2 = (t_total + sqrt_disc) / 2

    t_accel = None
    for candidate in [t_accel1, t_accel2]:
        if 0 < candidate < t_total:
            t_accel = candidate
            break
    if t_accel is None:
        return None, None, None

    v_max = a_max * t_accel
    t_flat = t_total - 2 * t_accel
    return v_max, t_accel, t_flat

def plot_trajectory(v_max, a_max, distance, t_total, v_limit):
    v_max, t_accel, t_flat = compute_vmax(distance, a_max, t_total)

    ax1.clear()
    ax2.clear()
    ax1.set_ylabel("Position (m)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_xlabel("Time (s)")
    ax1.grid(True)
    ax2.grid(True)

    if v_max is None or v_max > v_limit:
        # Not feasible trajectory
        ax1.text(0.5, 0.5, "Trajectory NOT feasible", fontsize=16, color='red',
                 ha='center', va='center', transform=ax1.transAxes)
        ax2.text(0.5, 0.5, "Adjust parameters", fontsize=14, color='red',
                 ha='center', va='center', transform=ax2.transAxes)
        fig.canvas.draw_idle()
        return

    dt = 0.01
    t_total_calc = 2 * t_accel + t_flat
    time = np.arange(0, t_total_calc + dt, dt)
    velocity = np.zeros_like(time)
    position = np.zeros_like(time)

    d_accel = 0.5 * a_max * t_accel**2

    for i, t in enumerate(time):
        if t < t_accel:
            velocity[i] = a_max * t
            position[i] = 0.5 * a_max * t**2
        elif t < t_accel + t_flat:
            velocity[i] = v_max
            position[i] = d_accel + v_max * (t - t_accel)
        else:
            t_decel = t - t_accel - t_flat
            velocity[i] = v_max - a_max * t_decel
            position[i] = d_accel + v_max * t_flat + v_max * t_decel - 0.5 * a_max * t_decel**2

    ax1.plot(time, position, lw=2)
    ax2.plot(time, velocity, lw=2, color='orange')
    fig.canvas.draw_idle()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
plt.subplots_adjust(left=0.1, bottom=0.35)

axcolor = 'lightgoldenrodyellow'
ax_amax = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
ax_dist = plt.axes([0.1, 0.2, 0.8, 0.03], facecolor=axcolor)
ax_ttotal = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)
ax_vlimit = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor=axcolor)

s_amax = Slider(ax_amax, 'Max Accel (m/s²)', 0.1, 10.0, valinit=init_a_max)
s_dist = Slider(ax_dist, 'Distance (m)', 0.1, 20.0, valinit=init_distance)
s_ttotal = Slider(ax_ttotal, 'Total Time (s)', 0.1, 20.0, valinit=init_t_total)
s_vlimit = Slider(ax_vlimit, 'Velocity Limit (m/s)', 0.1, 10.0, valinit=init_v_limit)

def update(val):
    a_max = s_amax.val
    distance = s_dist.val
    t_total = s_ttotal.val
    v_limit = s_vlimit.val
    plot_trajectory(None, a_max, distance, t_total, v_limit)

s_amax.on_changed(update)
s_dist.on_changed(update)
s_ttotal.on_changed(update)
s_vlimit.on_changed(update)

plot_trajectory(None, init_a_max, init_distance, init_t_total, init_v_limit)

plt.show()
