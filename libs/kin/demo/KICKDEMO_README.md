# Enhanced Kick Demo - RRT + TrajectoryManager Integration

This enhanced kick demo demonstrates advanced path planning and trajectory execution capabilities for SSL RoboCup robots.

## Key Features

### 🎯 **RRT Path Planning**
- Uses RRT* algorithm for optimal path planning from robot start position to ball
- Configurable parameters for smooth, efficient paths
- Converts between coordinate systems (meters ↔ millimeters) for RRT compatibility
- Calculates optimal pre-kick position based on target direction

### 🤖 **TrajectoryManager Integration**
- Utilizes `ctrl::TrajectoryManager` for smooth robot movement
- Implements trapezoidal velocity profiles for natural acceleration/deceleration
- Provides precise trajectory following with real-time position updates
- Syncs with visualization system for smooth robot movement display

### ⚽ **Smart Kick Execution**
- SSL-compliant kick system with proper distance and orientation checks
- Dynamic kick power calculation based on target distance
- Ball physics simulation with realistic friction and movement
- Kick direction calculated towards final target (not just towards ball)

### 📊 **Comprehensive Tracking**
- Real-time ball position and velocity monitoring
- Distance-to-target tracking
- Speed reduction analysis (initial → final)
- Success metrics and completion detection

## Demo Phases

### 1. **PLANNING_PATH_TO_BALL**
```cpp
// Configure RRT parameters for optimal path planning
algos::RRTParams rrt_params = algos::DefaultRRTParams();
rrt_params.max_iterations = 2000;
rrt_params.step_size = 150.0; // 15cm steps for smooth path
rrt_params.goal_tolerance = 100.0; // 10cm tolerance
rrt_params.use_rrt_star = true; // Use RRT* for optimal paths

// Plan path using RRT
rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
```

### 2. **MOVING_TO_BALL**
```cpp
// Use RobotManager's SetPath with TrajectoryManager
robot_manager.SetPath(trajectory_path, current_time);

// Update robot position from TrajectoryManager during path execution
robot_manager.ControlLogic();
```

### 3. **ORIENTING_TO_BALL**  
```cpp
// Calculate required angle to face the ball for optimal kick
double dx = ball_pos.x() - robot_pos.x();
double dy = ball_pos.y() - robot_pos.y();
double required_angle = atan2(dy, dx);
```

### 4. **KICKING_BALL**
```cpp
// Calculate kick direction towards target (not just towards ball)
Eigen::Vector2d kick_direction = (target_position.head<2>() - ball_pos.head<2>()).normalized();

// Use optimal kick power for reaching target
double distance_to_target = (target_position - ball_pos).norm();
double kick_power = std::min(distance_to_target * 1.2 + 1.0, 5.5); // Dynamic power, SSL compliant
```

### 5. **BALL_TRACKING**
```cpp
// Track ball movement and provide updates
double current_ball_speed = ball->velocity.head<2>().norm();
double distance_to_target = (target_position - ball->position).norm();
```

## Technical Specifications

### **RRT Configuration**
- **Max Iterations**: 2000 (ensures path finding success)
- **Step Size**: 150mm (15cm steps for smooth paths)
- **Goal Tolerance**: 100mm (10cm precision)
- **Algorithm**: RRT* (optimal path selection)

### **Trajectory Execution**
- **Manager**: `ctrl::TrajectoryManager` with trapezoidal profiles
- **Update Rate**: Real-time via `robot_manager.ControlLogic()`
- **Synchronization**: Position sync between RobotManager and visualization

### **Kick Parameters**
- **Distance Check**: 25cm optimal kicking distance (SSL compliant)
- **Power Calculation**: Dynamic based on target distance
- **Max Power**: 5.5 m/s (within SSL 6.5 m/s limit)
- **Direction**: Calculated towards final target position

### **Physics Integration**
- **Ball Model**: Enhanced `state::Ball` with SSL-specific parameters
- **Friction**: High carpet friction (3.5 m/s² deceleration)
- **Collision**: Proper collision detection and resolution
- **2D Constraint**: Ball forced to stay on ground plane

## Demo Positions

- **Robot Start**: (-2.0, -1.5, 0.0) - Bottom-left corner
- **Ball Position**: (1.5, 0.5, 0.0) - Right side of field  
- **Target Position**: (2.0, 1.5, 0.0) - Top-right corner
- **Kicking Distance**: 0.25m from ball center

## Success Criteria

✅ **RRT Path Planning**: Successfully finds optimal path from start to ball  
✅ **Smooth Movement**: TrajectoryManager provides natural robot acceleration/deceleration  
✅ **Precise Positioning**: Robot reaches optimal kicking position (±5cm tolerance)  
✅ **SSL Compliance**: All actions follow SSL rules and constraints  
✅ **Accurate Kick**: Ball kicked towards target with appropriate power  
✅ **Physics Realism**: Ball movement follows realistic physics with friction  

## Usage

```bash
# Build the demo
make kick_demo

# Run the enhanced demo
./libs/kin/demo/kick_demo
```

## Key Improvements Over Original

1. **Advanced Path Planning**: RRT* vs simple direct movement
2. **Smooth Trajectories**: TrajectoryManager vs linear interpolation  
3. **Smart Kick Calculation**: Dynamic power vs fixed power
4. **Comprehensive Tracking**: Real-time monitoring vs basic status
5. **SSL Compliance**: Proper distance/orientation checks vs basic validation
6. **Target-Oriented**: Kick towards final target vs just away from robot

This enhanced demo showcases championship-level robot control techniques suitable for competitive SSL RoboCup environments.