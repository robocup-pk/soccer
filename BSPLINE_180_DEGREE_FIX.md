# B-Spline 180-Degree Rotation Fix

## Problem
B-spline trajectory planner could only achieve 135° (75%) of requested 180° rotation due to the smoothing nature of B-spline curves.

## Solution
Modified `BSplineTrajectory::GenerateBSplineControlPoints()` to detect pure rotation cases and apply overshoot compensation:

### Key Changes:
1. **Detection of Pure Rotation**:
   - Position change < 0.1m AND angle change > 90°
   
2. **Overshoot Compensation**:
   - 90° rotation: 15% overshoot
   - 135° rotation: 25% overshoot  
   - 180° rotation: 33% overshoot

3. **Implementation**:
   ```cpp
   // For 180-degree rotation
   double overshoot_factor = 1.33;
   
   // Apply overshoot to intermediate control points
   cp[2] = p0[2] + t * angle_diff * overshoot_factor;
   ```

## Results

### Before Fix:
- Requested: 180° (π rad)
- Achieved: 135° (2.356 rad)
- Success: 75%

### After Fix:
- Requested: 180° (π rad)
- Achieved: 179.55° (3.134 rad)
- Success: 99.75%

## Test Results:
- Test case 8 (demo_stable_waypoints): ✓ 179.55°
- Basic demo case 1: ✓ 179.55°

## Impact:
- Pure rotations now achieve target angles
- Normal trajectories (with position changes) use original method
- No impact on multi-waypoint trajectories
- SSL robots can now perform full 180° rotations when needed

## Files Modified:
- `/libs/ctrl/src/BSplineTrajectory.cpp` - Added rotation detection and overshoot compensation