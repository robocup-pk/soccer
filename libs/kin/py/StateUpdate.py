import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.01
vx = 0.0
vy = 1.0
omega = 0 # np.pi / 6  # 30°/s

x0 = np.array([1, 1, 90])
steps = 100
true_states = [x0.copy()]
linear_pred = [x0.copy()]

for _ in range(steps):
    # --- Nonlinear motion (true) ---
    x, y, theta = true_states[-1]
    x_next = x + dt * (vx * np.cos(theta) - vy * np.sin(theta))
    y_next = y + dt * (vx * np.sin(theta) + vy * np.cos(theta))
    theta_next = theta + dt * omega
    true_states.append(np.array([x_next, y_next, theta_next]))

    # --- Linearized motion (update Φ each step) ---
    x_lin, y_lin, theta_lin = linear_pred[-1]
    Phi = np.array([
        [1, 0, -dt * (vx * np.sin(theta_lin) + vy * np.cos(theta_lin))],
        [0, 1,  dt * (vx * np.cos(theta_lin) - vy * np.sin(theta_lin))],
        [0, 0, 1]
    ])
    x_pred = Phi @ linear_pred[-1]
    x_pred[2] += dt * omega  # Add omega*dt manually
    linear_pred.append(x_pred)

# Convert to arrays
true_states = np.array(true_states)
linear_pred = np.array(linear_pred)

# Plot
plt.figure(figsize=(8, 6))
plt.plot(true_states[:, 0], true_states[:, 1], 'b-o', label='True Nonlinear Path')
plt.plot(linear_pred[:, 0], linear_pred[:, 1], 'r--x', label='Linearized Φ·x Prediction (updated)')
plt.title('True Motion vs. Linearized Prediction (Φ·x)')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
