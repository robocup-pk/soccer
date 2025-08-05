# Robot Odometry Analysis Report

## Data Issues Identified

### 1. **Orientation (Theta) Instability**
- Theta values jump erratically: 0.1 → 0.37 → 0.55 → 0.27 radians
- Changes of up to 0.28 radians (16°) between consecutive readings
- This is physically impossible for smooth robot motion at 10Hz

### 2. **Position Discontinuities**
- Sudden position jumps up to 3.5mm (line 20: jump from one position to another)
- Most movements are very small (< 1mm) but occasionally have large jumps
- This suggests sensor noise or communication issues

### 3. **Scale Mismatch**
- The UniformBSplineTrajectoryPlanner is configured for RoboCup field (9m x 6m)
- Actual robot data shows movements in millimeter range
- This causes the trajectory planner to create unrealistic paths

## Potential Root Causes

### 1. **Sensor Issues**
- **Encoder noise**: The encoders might be giving noisy readings
- **Gyroscope drift**: The orientation sensor may not be properly calibrated
- **Communication delays**: Data packets might be arriving out of order

### 2. **State Estimation Problems**
- The state estimator might not be properly fusing sensor data
- Missing Kalman filtering or sensor fusion
- No outlier rejection for bad sensor readings

### 3. **Units/Scaling Issues**
- Theta might be in a different unit than expected
- Position units might need scaling
- Coordinate frame misalignment

## Recommendations

### 1. **Immediate Fixes for B-Spline Trajectory**

```cpp
// 1. Add data filtering before creating waypoints
std::vector<Eigen::Vector3d> FilterOdometry(const std::vector<OdometryPoint>& raw_data) {
    std::vector<Eigen::Vector3d> filtered;
    if (raw_data.empty()) return filtered;
    
    Eigen::Vector3d last_point(raw_data[0].x, raw_data[0].y, raw_data[0].theta);
    filtered.push_back(last_point);
    
    for (size_t i = 1; i < raw_data.size(); ++i) {
        Eigen::Vector3d current(raw_data[i].x, raw_data[i].y, raw_data[i].theta);
        
        // Filter out jumps
        double dist = (current.head<2>() - last_point.head<2>()).norm();
        double dtheta = std::abs(current[2] - last_point[2]);
        
        // Reject physically impossible jumps (>5mm or >10 degrees)
        if (dist < 0.005 && dtheta < 0.174) { // 0.174 rad = 10 degrees
            filtered.push_back(current);
            last_point = current;
        }
    }
    return filtered;
}

// 2. Use adaptive field boundaries
void SetAdaptiveFieldBoundaries(UniformBSplineTrajectoryPlanner& planner, 
                                const std::vector<Eigen::Vector3d>& waypoints) {
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& wp : waypoints) {
        min_x = std::min(min_x, wp[0]);
        max_x = std::max(max_x, wp[0]);
        min_y = std::min(min_y, wp[1]);
        max_y = std::max(max_y, wp[1]);
    }
    
    // Add 10% margin
    double margin = 0.1;
    double x_range = max_x - min_x;
    double y_range = max_y - min_y;
    
    // Set custom field boundaries (need to add this method to planner)
    // planner.SetFieldBoundaries(
    //     min_x - margin * x_range,
    //     max_x + margin * x_range,
    //     min_y - margin * y_range,
    //     max_y + margin * y_range
    // );
}

// 3. Smooth orientation data
void SmoothOrientation(std::vector<Eigen::Vector3d>& waypoints) {
    if (waypoints.size() < 3) return;
    
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        double prev_theta = waypoints[i-1][2];
        double curr_theta = waypoints[i][2];
        double next_theta = waypoints[i+1][2];
        
        // Simple moving average
        waypoints[i][2] = (prev_theta + curr_theta + next_theta) / 3.0;
    }
}
```

### 2. **Robot System Improvements**

1. **Calibrate Sensors**:
   - Calibrate gyroscope at startup
   - Check encoder resolution and scaling
   - Verify coordinate frame alignment

2. **Improve State Estimation**:
   - Implement Kalman filter for sensor fusion
   - Add outlier rejection
   - Use complementary filter for orientation

3. **Data Validation**:
   - Add physical constraints (max velocity, max angular velocity)
   - Reject impossible state transitions
   - Log sensor raw data for debugging

### 3. **Testing Recommendations**

1. **Static Test**: Place robot stationary and log data
   - Check for drift in position/orientation
   - Measure sensor noise levels

2. **Controlled Motion Test**: 
   - Move robot in straight line at constant speed
   - Rotate robot at constant angular velocity
   - Compare logged data with expected motion

3. **Sensor Isolation**:
   - Test encoders separately
   - Test gyroscope separately
   - Identify which sensor is causing issues

## Conclusion

The chaotic behavior is likely due to noisy sensor data rather than the B-spline algorithm itself. The trajectory planner is trying to follow waypoints that include physically impossible jumps, resulting in erratic motion commands. Implementing proper filtering and sensor calibration should resolve the issue.