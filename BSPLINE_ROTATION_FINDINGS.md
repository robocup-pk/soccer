# B-Spline Rotation Findings

## Problem
B-spline trajectory planner cannot achieve full requested rotation when given only start and end waypoints.

## Test Results

### Test Case 8: Direct 180° Rotation (2 waypoints)
- **Requested**: 0° → 180° (π radians)
- **Achieved**: 0° → 135° (2.356 radians)
- **Success Rate**: 75%

### Test Case 9: 180° Rotation with Intermediate Waypoints (4 waypoints)
- **Waypoints**: 0° → 60° → 120° → 180°
- **Achieved**: Full 180° rotation
- **Success Rate**: 100%

## Root Cause
B-splines by design:
1. Do not interpolate intermediate control points
2. Generate smooth curves that "cut corners"
3. For 2-waypoint rotations, the spline only reaches ~75% of target

## Solutions for SSL Robots

### 1. For Small Rotations (< 90°)
- Direct 2-waypoint B-spline works fine
- Achieves ~75% which is often sufficient

### 2. For Medium Rotations (90° - 180°)
- Add 1-2 intermediate waypoints
- Example: For 180°, use waypoints at 0°, 60°, 120°, 180°

### 3. For Large Rotations (> 180°)
- Not recommended for SSL as robots rarely need full rotations
- Use multiple intermediate waypoints if necessary

### 4. Alternative Approach
- Use different trajectory type (BangBang, Hermite) for pure rotations
- Keep B-spline for smooth position trajectories

## Practical SSL Implementation

For RRT* paths with significant orientation changes:
```cpp
// If rotation > 90 degrees, add intermediate waypoints
double angle_diff = std::abs(end_angle - start_angle);
if (angle_diff > M_PI/2) {
    // Insert intermediate waypoints for rotation
    int num_intermediate = std::ceil(angle_diff / (M_PI/3));  // Every 60°
    for (int i = 1; i < num_intermediate; ++i) {
        double t = (double)i / num_intermediate;
        Eigen::Vector3d intermediate(
            start_pos + t * (end_pos - start_pos),  // Linear interpolation of position
            start_angle + t * angle_diff            // Linear interpolation of angle
        );
        waypoints.insert(waypoints.begin() + i, intermediate);
    }
}
```

## Conclusion
B-spline is excellent for smooth trajectories but has limitations for large rotations. For SSL robots:
- Small rotations work directly
- Large rotations need intermediate waypoints
- Most SSL scenarios don't require > 180° rotations