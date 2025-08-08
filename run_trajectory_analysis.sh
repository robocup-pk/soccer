#!/bin/bash

# Script to run trajectory demo with logging and visualize results

echo "=============================================="
echo "   B-Spline Trajectory Analysis Tool"
echo "=============================================="
echo ""
echo "This will run the demo with GUI and log trajectory data,"
echo "then create a visualization showing path deviation."
echo ""

# Default values
TEST_CASE=${1:-1}
TRAJ_TYPE=${2:-2}

echo "Test case: $TEST_CASE"
echo "  1: Square path"
echo "  2: Circle"
echo "  3: Figure-8"
echo "  4: Straight line"
echo ""
echo "Trajectory type: $TRAJ_TYPE"
echo "  1: B-spline"
echo "  2: Uniform B-spline (EWOK)"
echo "  3: Bezier"
echo ""
echo "=============================================="
echo ""
echo "Running demo with logging..."
echo "Press ESC in the demo window when you want to stop recording."
echo ""

# Run the demo with logging
./build/libs/algos/demo/demo_with_logging $TEST_CASE $TRAJ_TYPE

# Check if trajectory log was created
if [ -f "trajectory_log.txt" ]; then
    echo ""
    echo "Demo finished. Generating visualization..."
    python3 visualize_logged_trajectory.py
else
    echo "Error: trajectory_log.txt was not created!"
    exit 1
fi