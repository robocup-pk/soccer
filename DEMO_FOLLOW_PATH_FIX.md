# DemoFollowPath Ball Attachment Fix

## Problem
The robot wasn't attaching/detaching the ball because:
1. No robot action was set (defaults to MOVE)
2. Kinematics wasn't being updated to handle ball physics

## Solution

### 1. Set Robot Action to DRIBBLE_BALL
```cpp
robot_manager.SetRobotAction(rob::RobotAction::DRIBBLE_BALL);
```

### 2. Update Kinematics in Main Loop
```cpp
// Update kinematics to handle ball physics and attachment
kin::UpdateKinematics(soccer_objects, util::CalculateDt());
```

## How Ball Attachment Works

1. **Ball Detection**: `IsBallInFrontOfRobot()` checks if ball is in front sector
2. **Attachment**: `HandleBallSticking()` attaches ball to robot
3. **Position Update**: `UpdateAttachedBallPosition()` keeps ball in front
4. **Detachment**: `DetachBall()` releases ball with velocity when:
   - Robot action changes to KICK_BALL (6.5 m/s)
   - Robot action changes to PASS_BALL (1.0 m/s)

## Robot Actions
- `MOVE`: Normal movement, no ball interaction
- `DRIBBLE_BALL`: Can attach ball, keeps it attached
- `KICK_BALL`: Detaches ball with high velocity
- `PASS_BALL`: Detaches ball with low velocity

## Testing
Run the demo and the robot should now:
1. Move to the ball
2. Attach it when in front
3. Carry it along the path
4. Detach when action changes