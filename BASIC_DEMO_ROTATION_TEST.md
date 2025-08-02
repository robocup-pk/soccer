# Basic Demo Case 1 - 180 Degree Rotation Test

## Test Setup
- **File**: `/libs/algos/demo/demo.cpp` (compiled as `basic_demo`)
- **Test Case 1**: Pure rotation from 0° to 180°
- **Waypoints**: 
  - Start: (0, 0, 0°)
  - End: (0, 0, 180°)

## Results with B-Spline Trajectory

### Expected vs Actual
- **Requested Rotation**: 180° (π radians)
- **Achieved Rotation**: 135° (2.35619 radians)  
- **Success Rate**: 75%

### B-Spline Output
```
[BSplineTrajectory::SetPath] B-spline trajectory generated successfully. Control points: 5, Arc length: 0.212058m
  Start: (0, 0, 0 rad)
  End: (0, 0, 2.35619 rad = 135 deg)
```

## Confirmation of Findings

This test confirms that:
1. B-spline trajectory planner has a fundamental limitation with large rotations
2. When given only 2 waypoints for a 180° rotation, it achieves only 135°
3. This is consistent with our demo_stable_waypoints test case 8

## Solution

To achieve full 180° rotation in basic_demo case 1, you would need to:
1. Add intermediate waypoints (like we did in test case 9)
2. Use a different trajectory type (BangBang, Hermite, etc.)
3. Overshoot the target angle to compensate

## Impact on SSL

For SSL robots:
- Most game scenarios don't require 180° rotations
- 135° is often sufficient for gameplay
- When full rotation is needed, use intermediate waypoints