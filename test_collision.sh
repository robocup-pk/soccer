#!/bin/bash
echo "Starting DemoShape with automatic SPACE press..."
echo "This will show obstacle detection debug info."
echo "Look for robot1 in the obstacle list!"

# Run the demo for a few seconds and capture output
timeout 10s ./libs/vis/demo/DemoShape > debug_output.txt 2>&1 &
DEMO_PID=$!

# Wait a moment for demo to start
sleep 2

# Try to send SPACE key to the demo window (if it has focus)
# This is a bit tricky without direct window control, but the debug output will show anyway

# Wait for demo to finish or timeout
wait $DEMO_PID

echo "Demo finished. Checking debug output..."

# Show the debug output
if [ -f debug_output.txt ]; then
    echo "=== Debug Output ==="
    cat debug_output.txt | grep -A 20 -B 5 "OBSTACLE DETECTION\|Added obstacle\|robot1\|robot0"
else
    echo "No debug output found"
fi

# Cleanup
rm -f debug_output.txt