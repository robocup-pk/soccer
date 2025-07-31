# Trajectory Manager Comparison: Original vs B-Spline

## Key Differences Found

### 1. Original TrajectoryManager (Working on Real Robot)

**Velocity Generation:**
```cpp
// In TrajectoryManager::Update()
Eigen::Vector3d velocity_fWorld = GetVelocityAtT(current_time_s);
Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
```

**GetVelocityAtT Implementation:**
```cpp
// Very simple feedback with LOW gain
double kp = 0.0;  // Almost no feedback!
Eigen::Vector3d Current_speed = current_trajectory->VelocityAtT(current_time_s);
Eigen::Vector3d position_error = current_trajectory->PositionAtT(current_time_s) - Current_position_fWorld;

// Apply safety limits to correction
Eigen::Vector3d correction = kp * position_error;
correction[0] = std::clamp(correction[0], -0.3, 0.3);
correction[1] = std::clamp(correction[1], -0.3, 0.3);
correction[2] = std::clamp(correction[2], -2.0, 2.0);

Eigen::Vector3d Final_determined_velocity = Current_speed + correction;

// Apply final velocity limits
Final_determined_velocity[0] = std::clamp(Final_determined_velocity[0], -1.0, 1.0);
Final_determined_velocity[1] = std::clamp(Final_determined_velocity[1], -1.0, 1.0);
Final_determined_velocity[2] = std::clamp(Final_determined_velocity[2], -2.0, 2.0);
```

### 2. B-Spline TrajectoryManager (Not Working on Real Robot)

**Velocity Generation:**
```cpp
// In BSplineTrajectoryManager::Update()
Eigen::Vector3d velocity_fWorld = bspline_trajectory_->Update(pose_est, current_time);

// Apply smoothing filter
if (smoothing_enabled_) {
    ApplySmoothingFilter(velocity_fWorld);  // EMA filter with alpha=0.3
}

// Apply safety limits
ApplySafetyLimits(velocity_fWorld);

// Convert to body frame
Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
```

**BSplineTrajectory::Update Implementation:**
```cpp
// Much higher feedback gain
double kp = 1.5;  // HIGH feedback gain!
Eigen::Vector3d position_error = desired_position - current_pose;

// Direct velocity calculation with feedback
Eigen::Vector3d feedforward_velocity = ComputeVelocityAtArcLength(current_arc_length);
Eigen::Vector3d feedback_velocity;
feedback_velocity.head<2>() = kp_ * position_error.head<2>();
feedback_velocity[2] = kp_ * NormalizeAngle(position_error[2]);

return feedforward_velocity + feedback_velocity;
```

## Critical Differences:

### 1. **Feedback Gain**
- Original: `kp = 0.0` (effectively NO feedback correction)
- B-Spline: `kp = 1.5` (high feedback correction)

### 2. **Velocity Limits**
- Original: Conservative limits (1.0 m/s linear, 2.0 rad/s angular)
- B-Spline: Higher limits (0.7 m/s after safety, but feedforward can be higher)

### 3. **Smoothing**
- Original: No smoothing, just clamping
- B-Spline: Exponential moving average filter (alpha=0.3)

### 4. **Trajectory Type**
- Original: BangBang trajectories with jerk limiting
- B-Spline: Smooth spline interpolation with arc-length parameterization

## Why B-Spline Fails on Real Robot:

1. **High Feedback Gain**: The B-spline uses kp=1.5 which amplifies sensor noise
2. **Smoothing Filter Lag**: The EMA filter introduces lag that compounds with sensor delays
3. **Complex Calculations**: Arc-length parameterization and spline evaluation add computational overhead

## Recommended Fix:

Modify B-spline to match original behavior:
1. Reduce feedback gain to 0.0 or very low (0.1-0.2)
2. Disable smoothing filter for real robot
3. Apply same conservative velocity limits as original
4. Consider simplifying velocity computation