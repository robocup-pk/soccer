#!/bin/bash
# Script to test trajectory-waypoint alignment

echo "Testing trajectory-waypoint alignment for Test Case 1 (Triangle)"
echo "Expected waypoints: (0,0), (0,1), (0.5,0.5), (0,0)"

# Run the demo and capture output
timeout 5 ./build/libs/algos/demo/demo_with_logging 1 2>&1 | tee trajectory_test.log

# Extract waypoints from log
echo -e "\nExtracted waypoints from log:"
grep -E "^\s+[0-9]+: \(" trajectory_test.log

# Check if trajectory passes through waypoints
echo -e "\nCheck control points to see if they align with waypoints:"
grep "CP\[" trajectory_test.log | head -10

echo -e "\nTo generate a new plot with correct alignment:"
echo "python3 analyze_trajectory.py ./soccer/build/libs/algos/demo/demo_with_logging 1"