# B-Spline Configuration for Real Robot

## Quick Fix for Real Robot Testing

Based on the comparison with the working original trajectory manager, here are the changes needed:

### 1. Reduce Feedback Gain

In `BSplineTrajectory.cpp`, change line around feedback gain:
```cpp
// Change from:
kp_ = 1.5;

// To:
kp_ = 0.1;  // Very low feedback for real robot
```

### 2. Disable Smoothing Filter

In your RobotManager or when initializing B-spline:
```cpp
bspline_manager.SetSmoothingEnabled(false);  // Disable for real robot
```

### 3. Reduce Velocity Limits

In `BSplineTrajectory.cpp` constructor:
```cpp
// Change from:
v_max_ = 0.6;    // m/s
a_max_ = 0.3;    // m/s²

// To match original:
v_max_ = 0.4;    // m/s - more conservative
a_max_ = 0.2;    // m/s² - smoother acceleration
```

### 4. Set Conservative Safety Limits

In `BSplineTrajectoryManager::ApplySafetyLimits()`:
```cpp
// Change from:
const double max_linear = 0.7;   // m/s
const double max_angular = 2.5;  // rad/s

// To match original:
const double max_linear = 0.5;   // m/s - very conservative
const double max_angular = 2.0;  // rad/s - match original
```

## Test Procedure for Real Robot:

1. **Start with straight line test**:
   ```cpp
   std::vector<Eigen::Vector3d> path;
   path.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
   path.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));  // 1m forward
   robot_manager.SetBSplinePath(path);
   ```

2. **Test pure rotation** (small angle first):
   ```cpp
   path.clear();
   path.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
   path.push_back(Eigen::Vector3d(0.0, 0.0, M_PI/4));  // 45 degrees
   ```

3. **Monitor debug output**:
   - Add logging to see actual velocities being sent
   - Check if pose estimation is stable
   - Watch for oscillations or jerky motion

## Key Insight:

The original trajectory manager works because it uses **open-loop control** (kp=0.0) with conservative limits. The B-spline's high feedback gain (kp=1.5) amplifies noise from real sensors, causing instability.

For real robots with noisy sensors, less feedback is often better!