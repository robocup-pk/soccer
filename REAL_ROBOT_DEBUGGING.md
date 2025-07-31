# Real Robot Debugging Guide for B-Spline Trajectory

## Coordinate System Overview

The system uses three coordinate frames:
1. **World Frame**: Fixed global coordinate system (from camera)
2. **Body Frame**: Robot-centered coordinate system 
3. **Wheel Frame**: Individual wheel coordinate systems

## Transformation Chain

```
World Frame → Body Frame → Wheel Speeds
```

1. **B-spline generates velocity in world frame**
   - Based on waypoints in world coordinates
   - Output: `velocity_fWorld = (vx_world, vy_world, omega)`

2. **Convert to body frame** (in BSplineTrajectoryManager::Update)
   ```cpp
   velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
   ```
   - `pose_est[2]` is robot orientation from sensors

3. **Convert to wheel speeds** (in HardwareManager)
   ```cpp
   wheel_speeds_rpm = robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
   ```

## Common Issues and Solutions

### 1. Robot Moves in Wrong Direction
**Possible Causes:**
- Camera coordinate system doesn't match robot convention
- Robot orientation (theta) has wrong sign
- Wheel configuration mismatch

**Debug Steps:**
```bash
# Run coordinate transform test
./test_coordinate_transform

# Check if forward motion (1,0,0) in world frame produces correct wheel speeds
# at different robot orientations
```

### 2. Rotation Issues
**Possible Causes:**
- Gyro calibration offset
- Gyro sign convention (clockwise vs counter-clockwise)
- Wheel angle configuration

**Debug Steps:**
- Command pure rotation (0,0,1) rad/s
- Verify all wheels spin correctly for rotation
- Check gyro reading matches commanded rotation

### 3. Sensor Noise
**Symptoms:** Jerky motion, oscillations

**Solutions:**
- Enable smoothing filter in BSplineTrajectoryManager
- Reduce feedback gains (kp_)
- Add low-pass filter to pose estimate

### 4. Coordinate System Verification

Run this test sequence:

1. **Test Forward Motion**
   - Command: Move forward 1m in world X
   - Robot at 0° should move in +X
   - Robot at 90° should move in +Y
   - Robot at 180° should move in -X

2. **Test Rotation**
   - Command: Rotate 90° CCW
   - Verify gyro reads positive angular velocity
   - Verify final orientation increases by π/2

3. **Test Combined Motion**
   - Command: Move in circle
   - Verify smooth curved motion

## Real Robot Checklist

- [ ] Camera calibrated and aligned with world frame
- [ ] Gyro calibrated (zero bias removed)
- [ ] Wheel configuration matches physical robot:
  ```
  Wheel 0: pos=(0.046, 0.080)m, angle=-30°
  Wheel 1: pos=(-0.065, 0.066)m, angle=45°
  Wheel 2: pos=(-0.065, -0.066)m, angle=135°
  Wheel 3: pos=(0.046, -0.080)m, angle=-150°
  ```
- [ ] Motor directions verified (positive command = forward rotation)
- [ ] State estimator properly fuses camera + gyro + motor data

## Debugging Commands

```cpp
// Add debug output to BSplineTrajectoryManager::Update
std::cout << "Pose: " << pose_est.transpose() << std::endl;
std::cout << "Vel World: " << velocity_fWorld.transpose() << std::endl;
std::cout << "Vel Body: " << velocity_fBody.transpose() << std::endl;

// Add to HardwareManager::SetBodyVelocity
std::cout << "Wheel RPMs: " << wheel_speeds_rpm.transpose() << std::endl;
```

## Parameter Tuning

If motion is correct but not smooth:
1. Reduce max velocity: `v_max_ = 0.4` (from 0.6)
2. Reduce acceleration: `a_max_ = 0.2` (from 0.3)
3. Reduce feedback gain: `kp_ = 1.0` (from 1.5)
4. Enable smoothing filter with `alpha = 0.3`

## Testing B-Spline with Real Robot

1. Start with simple straight line trajectory
2. Test pure rotation (small angles first)
3. Test figure-8 pattern
4. Finally test full RRT* waypoint following

Remember: The B-spline planner is working correctly in simulation. Issues with real robots are typically due to:
- Sensor calibration
- Coordinate system misalignment
- Hardware configuration mismatches