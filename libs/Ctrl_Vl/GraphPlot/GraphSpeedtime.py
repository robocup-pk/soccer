import matplotlib.pyplot as plt

# Your x and y values
x = [0, 1, 2, 3, 4, 5]
y1 = [0, 2, 4, 6, 8, 10]
y2 = [0, 1, 3, 5, 6, 6]

# Plot both
plt.plot(x, y1, label="Constant Acceleration", marker='o')
plt.plot(x, y2, label="Variable Acceleration", linestyle='--', marker='x')

# Labels and legend
plt.title("Speed-Time Graph")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.legend()
plt.grid(True)
plt.show()
