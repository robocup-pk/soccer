#!/bin/bash

echo "🤖 Starting Soccer Robot Demo with IntelligentMovement2"
echo "============================================="
echo ""
echo "📋 SCENARIO:"
echo "  • robot0 (source): Will use ball interception strategy"
echo "  • robot1 (opponent): Acts as obstacle (100x100px)"
echo "  • ball (target): Moving target to intercept"
echo ""
echo "🎮 CONTROLS:"
echo "  SPACE    - Toggle IntelligentMovement2 on/off"
echo "  1        - Switch to RRT Movement (IntelligentMovement)"
echo "  2        - Switch to Ball Intercept (IntelligentMovement2) ← DEFAULT"
echo "  ESC      - Exit demo"
echo ""
echo "🔍 WHAT TO OBSERVE:"
echo "  • robot0 should analyze ball interception opportunities"
echo "  • Strategic decisions: 'Should chase: YES/NO'"
echo "  • Actions: TURN, DASH, TURN_AND_DASH based on situation"
echo "  • Collision avoidance with robot1 (100x100px robots)"
echo "  • Console output shows decision-making process"
echo ""
echo "✅ IntelligentMovement2 is now the DEFAULT system!"
echo "   It will start automatically when the demo loads."
echo ""

read -p "Press Enter to start the demo..."

# Change to build directory and run
cd /home/kodek/RoboCupProjects/soccer/build || {
    echo "❌ Error: Could not find build directory"
    exit 1
}

echo ""
echo "🚀 Starting demo..."
echo "   Watch the console for IntelligentMovement2 debug output!"
echo ""

./libs/vis/demo/DemoShape