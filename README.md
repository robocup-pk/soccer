# RoboCup SSL Soccer Robot System

This is our team's codebase for Small Size League (SSL) RoboCup soccer robots.

## What's Inside

The codebase is organized into several libraries, each handling different aspects of robot control:

### Core Libraries
- **`ctrl/`** - Three different trajectory planning systems (because one wasn't enough)
  - Trapezoidal trajectories for simple point-to-point motion
  - Advanced BangBang planner with obstacle avoidance
  - B-Spline trajectories for smooth curved paths
- **`rob/`** - Robot manager that coordinates everything and prevents chaos
- **`kin/`** - Kinematics for omni-wheel robots (the math that makes it move)
- **`hw/`** - Hardware interface for motors, sensors, and all the physical stuff
- **`est/`** - State estimation using Kalman filters and sensor fusion
- **`vis/`** - OpenGL visualization because we need to see what's happening

### Demo Programs
- **`multi_robot_demo`** - Shows 4 robots following different trajectories simultaneously
- **`demo_with_logging`** - Test different trajectory types and log performance data
- Various kinematic demos for testing motion primitives

## Getting Started

You'll need CMake and the usual suspects (OpenGL, Eigen3, etc.) to build this.

```bash
mkdir build && cd build
cmake ..
make -j4
```

## Running Demos

The most interesting demo is probably the multi-robot one:
```bash
./libs/vis/demo/multi_robot_demo
```

For testing individual trajectory planners with detailed logging:
```bash
./libs/algos/demo/demo_with_logging [test_case] [trajectory_type]
```

Where:
- `test_case`: 1 (square), 2 (figure-8), 3 (square no rotation), 4 (circle)
- `trajectory_type`: 1 (BangBang), 2 (B-Spline), 3 (Trapezoidal)

## Project Highlights

### Multiple Trajectory Systems
We implemented three different approaches because each has its strengths:
- **Trapezoidal**: Simple and predictable, great for basic movements
- **BangBang**: Fast and includes path planning around obstacles
- **B-Spline**: Smooth curves that look professional in demos

### Robust Architecture  
The system handles coordinate frame transformations properly (world → body → wheel) and includes safety checks to prevent robots from flying off the field.

### Real-time Performance
Everything runs in real-time with proper threading. The control loop hits 50Hz reliably, and sensor processing runs at 100Hz.

## Field Specifications

We're targeting RoboCup SSL Division B fields:
- Field size: 9m × 6m
- Robot diameter: ~180mm
- 4 omni-wheels per robot
- Camera-based global localization

## Known Issues and Quirks

- The B-Spline planner used to generate orientation automatically even when you didn't want it to (fixed now)
- Race conditions in multi-robot demos were a pain to debug (also fixed)
- Sometimes the visualization gets laggy if you run too many robots at once
- The logging system writes to specific directories, so make sure they exist

## Development Notes

This codebase has been through several iterations. We started with a simple point-to-point controller and gradually added more sophisticated planning. The current architecture can handle:
- Dynamic replanning when robots get off track
- Obstacle avoidance (though we don't have physical obstacles yet)
- Smooth cornering without overshooting
- Multiple robots operating simultaneously without conflicts

The code is reasonably well-tested, but like any robotics project, there are edge cases that occasionally surface during competitions.

## Contributing

If you're working on this code:
1. Test your changes with the demo programs first
2. Check that all three trajectory systems still work
3. Run the visualization to make sure nothing looks obviously wrong
4. The Python analysis scripts in `libs/algos/py/` are helpful for debugging trajectory issues