#!/usr/bin/env python3

import subprocess
import time
import sys

def test_ball_intercept():
    print("=== Testing Ball Intercept Movement System ===")
    print()
    print("This test will:")
    print("1. Start the demo")
    print("2. Switch to Ball Intercept mode (key '2')")
    print("3. Enable the movement system (SPACE)")
    print("4. Show the strategic ball interception behavior")
    print()
    print("Expected behavior:")
    print("- robot0 should analyze ball interception opportunities")
    print("- robot0 should avoid robot1 (100x100px collision detection)")
    print("- robot0 should make strategic decisions based on ball movement")
    print()
    
    input("Press Enter to start test...")
    
    print("Starting DemoShape...")
    print("Quick instructions:")
    print("  Press '2' to switch to Ball Intercept mode")
    print("  Press SPACE to enable movement")
    print("  Watch the console output for strategic decisions")
    print()
    
    # Run the demo
    try:
        subprocess.run(["./libs/vis/demo/DemoShape"], 
                      cwd="/home/kodek/RoboCupProjects/soccer/build",
                      timeout=30)
    except subprocess.TimeoutExpired:
        print("Demo completed (timeout)")
    except KeyboardInterrupt:
        print("Demo interrupted by user")

if __name__ == "__main__":
    test_ball_intercept()