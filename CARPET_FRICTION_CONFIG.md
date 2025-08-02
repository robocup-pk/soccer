# Carpet Friction Configuration for B-Spline

## Problem
High-friction carpet causes:
- Robot travels 0.75m instead of 1m
- Slower acceleration/deceleration
- More energy needed for rotation

## Quick Configuration

### 1. In your robot initialization:
```cpp
// For high-friction carpet
bspline_manager.SetVelocityLimits(0.4, 1.5);  // Reduce linear and angular velocity
bspline_manager.SetFeedbackGains(0.0, 0.0);   // No feedback (avoid oscillations)
bspline_manager.SetSmoothingEnabled(false);    // No smoothing

// Reduce acceleration limits in BSplineTrajectory
bspline_trajectory.SetLimits(
    0.4,   // v_max (was 0.6)
    0.15,  // a_max (was 0.3) - much lower for carpet
    1.5,   // omega_max (was 2.0)
    2.0    // alpha_max (was 3.0)
);
```

### 2. Distance compensation:
```cpp
// After setting waypoints, scale them up
for (auto& wp : waypoints) {
    wp[0] *= 1.33;  // X compensation
    wp[1] *= 1.33;  // Y compensation
    // Don't scale rotation (wp[2])
}
```

### 3. Surface-specific profiles:
```cpp
enum SurfaceType {
    SIMULATOR,
    SMOOTH_FLOOR,
    LOW_FRICTION_CARPET,
    HIGH_FRICTION_CARPET
};

void ConfigureForSurface(SurfaceType surface) {
    switch (surface) {
        case HIGH_FRICTION_CARPET:
            v_max = 0.3;
            a_max = 0.1;
            distance_scale = 1.4;
            break;
        case LOW_FRICTION_CARPET:
            v_max = 0.4;
            a_max = 0.15;
            distance_scale = 1.2;
            break;
        default:
            v_max = 0.6;
            a_max = 0.3;
            distance_scale = 1.0;
    }
}
```

## Testing Protocol

1. **Calibrate linear motion**:
   ```
   - Command: Move forward 1m
   - Measure: Actual distance
   - Calculate: scale_factor = 1.0 / actual_distance
   ```

2. **Calibrate rotation**:
   ```
   - Command: Rotate 360°
   - Measure: Actual rotation
   - Adjust: omega_max and alpha_max
   ```

3. **Test acceleration**:
   ```
   - Command: Quick start/stop
   - Observe: Wheel slip
   - Reduce: a_max until no slip
   ```

## Why BangBang Works Better on Carpet

BangBang trajectory uses maximum acceleration immediately, which helps overcome static friction. B-spline's smooth acceleration can get "stuck" in high friction.

Consider using BangBang for:
- Short movements
- Quick direction changes
- High-friction surfaces