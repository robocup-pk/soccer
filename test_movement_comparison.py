#!/usr/bin/env python3

import subprocess
import time
import threading

def test_movement_systems():
    print("=== Comparing IntelligentMovement vs IntelligentMovement2 ===")
    print()
    print("Test Scenario:")
    print("- robot0 (source): positioned at (50, 180)")  
    print("- robot1 (opponent): positioned at (25, 150) - blocking path")
    print("- ball (target): positioned at (0, 0)")
    print("- Robot size: 100x100 pixels each")
    print("- Distance robot0 to robot1: ~35 pixels (collision risk!)")
    print()
    print("We will test both systems with this challenging scenario...")
    print()
    
    print("=== TEST 1: IntelligentMovement (RRT-based) ===")
    input("Press Enter to start RRT test (will auto-enable)...")
    
    print("Starting RRT Movement test...")
    print("Expected: robot0 should use RRT to find path around robot1")
    
    try:
        # Test RRT movement (mode 1, already default)
        result = subprocess.run(
            ["timeout", "6s", "./libs/vis/demo/DemoShape"],
            cwd="/home/kodek/RoboCupProjects/soccer/build",
            capture_output=True,
            text=True
        )
        
        # Analyze RRT results
        rrt_output = result.stderr
        print("\n--- RRT Movement Analysis ---")
        
        if "RRT] Path found!" in rrt_output:
            print("✅ RRT successfully found collision-free path")
            # Count RRT iterations
            iterations = []
            for line in rrt_output.split('\n'):
                if "Goal reached in" in line:
                    iter_count = line.split("Goal reached in ")[1].split(" iterations")[0]
                    iterations.append(int(iter_count))
            
            if iterations:
                avg_iterations = sum(iterations) / len(iterations)
                print(f"✅ Average RRT iterations: {avg_iterations:.1f}")
                print(f"✅ Path finding attempts: {len(iterations)}")
            
        if "Following trajectory point" in rrt_output:
            trajectory_lines = [line for line in rrt_output.split('\n') if "Following trajectory point" in line]
            print(f"✅ Robot followed {len(trajectory_lines)} trajectory waypoints")
            
        if "ADDED OBSTACLE: robot1" in rrt_output:
            print("✅ robot1 correctly detected as obstacle with 80px radius")
        else:
            print("❌ robot1 obstacle detection failed")
            
    except Exception as e:
        print(f"❌ RRT test failed: {e}")
    
    print("\n" + "="*60)
    
    print("\n=== TEST 2: IntelligentMovement2 (Ball Intercept) ===")
    input("Press Enter to start Ball Intercept test...")
    
    print("Starting Ball Intercept Movement test...")
    print("Expected: robot0 should strategically intercept ball while avoiding robot1")
    
    # Create a simple script to switch to mode 2 and enable it
    test_script = """
import subprocess
import time
import sys
from pynput import keyboard
import threading

def send_keys():
    time.sleep(1)  # Wait for demo to start
    
    # Simulate pressing '2' to switch to intercept mode
    print("Sending key '2' to switch to Ball Intercept mode...")
    keyboard_controller = keyboard.Controller()
    keyboard_controller.press('2')
    keyboard_controller.release('2')
    
    time.sleep(0.5)
    
    # Simulate pressing SPACE to enable movement
    print("Sending SPACE to enable movement...")
    keyboard_controller.press(' ')
    keyboard_controller.release(' ')

# Start key sending in background
key_thread = threading.Thread(target=send_keys)
key_thread.daemon = True
key_thread.start()

# Run the demo
subprocess.run(["timeout", "6s", "./libs/vis/demo/DemoShape"], 
               cwd="/home/kodek/RoboCupProjects/soccer/build")
"""
    
    # For now, let's manually test by running the demo and showing instructions
    print("\nManual Test Instructions:")
    print("1. Demo will start")
    print("2. Press '2' to switch to Ball Intercept mode")
    print("3. Press SPACE to enable Ball Intercept movement")
    print("4. Observe robot0 behavior")
    print()
    input("Press Enter to start manual test...")
    
    try:
        subprocess.run(
            ["timeout", "8s", "./libs/vis/demo/DemoShape"],
            cwd="/home/kodek/RoboCupProjects/soccer/build"
        )
    except Exception as e:
        print(f"Ball Intercept test: {e}")
    
    print("\n=== COMPARISON SUMMARY ===")
    print("Please observe and compare:")
    print("1. Which system avoids robot1 more effectively?")
    print("2. Which system reaches the ball faster?") 
    print("3. Which system handles the 100x100px collision detection better?")
    print("4. Which system shows smoother movement?")

if __name__ == "__main__":
    test_movement_systems()