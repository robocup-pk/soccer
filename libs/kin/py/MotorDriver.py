import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
time_steps = 50
rpm_set = 1500
a = 0.5  # Linear relation coefficient: rpm = a * pwm
b = 0    # Offset
Kp = 0.1 # Proportional gain for control

# Initialize variables
rpm_measured = 0
rpm_measured_history = []
pwm_history = []
rpm_set_history = []

pwm = 0

for t in range(time_steps):
    # Calculate error
    error = rpm_set - rpm_measured
    
    # Feedforward PWM for desired rpm
    pwm_base = (rpm_set - b) / a
    
    # PWM adjustment by proportional control
    pwm_adjust = Kp * error
    
    # New PWM signal
    pwm = pwm_base + pwm_adjust
    pwm = max(0, min(pwm, 255))  # Clamp PWM
    
    # Update rpm_measured based on PWM and motor dynamics
    # Here, a simple first order system with inertia
    rpm_measured += 0.2 * (a * pwm + b - rpm_measured)  # smooth approach
    
    # Store history for plotting
    rpm_measured_history.append(rpm_measured)
    pwm_history.append(pwm)
    rpm_set_history.append(rpm_set)

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

axs[0].plot(rpm_measured_history, label='Measured RPM')
axs[0].plot(rpm_set_history, 'r--', label='Set RPM')
axs[0].set_ylabel('RPM')
axs[0].set_title('Motor RPM Control')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(pwm_history, label='PWM Signal')
axs[1].set_ylabel('PWM')
axs[1].set_xlabel('Time steps')
axs[1].set_title('PWM Signal Over Time')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()
