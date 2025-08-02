# B-Spline Smoothing Impact on Real Robot

## Current Smoothing Configuration
- **Filter Type**: Exponential Moving Average (EMA)
- **Alpha**: 0.3 (heavy smoothing)
- **Formula**: `velocity = 0.3 * new_velocity + 0.7 * previous_velocity`

## Effects on Trajectory

### 1. Position Tracking
- **With smoothing**: Slight overshoot on corners, delayed stopping
- **Without smoothing**: Tighter tracking, more accurate stops

### 2. Velocity Profile
- **With smoothing**: Gradual acceleration/deceleration
- **Without smoothing**: Sharper velocity changes

### 3. Response Time
- **With smoothing**: ~3-5 control cycles lag
- **Without smoothing**: Immediate response

## Real Robot Considerations

### Why Disable/Reduce for Real Robot:
1. **Sensor Feedback Loop**: Real robot already has physical inertia
2. **Double Filtering**: State estimator + smoothing = too much lag
3. **Control Delay**: Added lag can cause oscillations with PID feedback

### Recommended Settings:

#### For Simulator:
```cpp
bspline_manager.SetSmoothingEnabled(true);  // Keep smoothing
// Or reduce: SetSmoothingAlpha(0.3);  // Current default
```

#### For Real Robot:
```cpp
bspline_manager.SetSmoothingEnabled(false);  // Disable completely
// Or reduce: SetSmoothingAlpha(0.7);  // Light smoothing only
```

## Testing Procedure:

1. **Start with no smoothing**:
   ```cpp
   bspline_manager.SetSmoothingEnabled(false);
   ```

2. **If motion is too jerky**, add light smoothing:
   ```cpp
   bspline_manager.SetSmoothingEnabled(true);
   bspline_manager.SetSmoothingAlpha(0.7);  // 70% new, 30% old
   ```

3. **Monitor for issues**:
   - Oscillations → Reduce or disable smoothing
   - Jerky motion → Increase smoothing slightly
   - Overshooting → Reduce smoothing

## Impact on 180° Rotation:
- Smoothing affects rotation speed but not final angle
- With smoothing: Slower, smoother rotation
- Without smoothing: Faster, more direct rotation

## Conclusion:
For real robots, **disable or significantly reduce smoothing** (alpha ≥ 0.7) because:
1. Physical inertia provides natural smoothing
2. Reduces control lag and improves responsiveness
3. Prevents oscillations from feedback loops