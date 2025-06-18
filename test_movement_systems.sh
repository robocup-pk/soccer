#!/bin/bash

echo "=== Testing Dual Movement Systems ==="
echo "This script will demonstrate both movement systems:"
echo "1. RRT Movement (obstacle avoidance)"
echo "2. Ball Intercept Movement (strategic interception)"
echo ""

echo "Starting demo with instructions..."
echo ""
echo "CONTROLS:"
echo "  SPACE - Toggle current movement system on/off"
echo "  1 - Switch to RRT Movement"
echo "  2 - Switch to Ball Intercept Movement"  
echo "  ESC - Exit"
echo ""
echo "SCENARIO:"
echo "  - robot0 (source player) should move to ball"
echo "  - robot1 (opponent) positioned between robot0 and ball"
echo "  - Both robots are 100x100 pixels"
echo "  - robot0 should avoid colliding with robot1"
echo ""

echo "=== Test Sequence ==="
echo "1. Testing RRT Movement (will start automatically)"
echo "   - Press SPACE to enable RRT movement"
echo "   - robot0 should use RRT to avoid robot1 and reach ball"
echo ""
echo "2. Testing Ball Intercept Movement"
echo "   - Press 2 to switch to Ball Intercept mode"
echo "   - Press SPACE to enable Ball Intercept movement"
echo "   - robot0 should strategically intercept ball considering robot1"
echo ""

read -p "Press Enter to start the demo..."

# Run the demo
./libs/vis/demo/DemoShape