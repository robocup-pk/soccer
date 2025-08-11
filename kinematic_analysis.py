#!/usr/bin/env python3
"""
Analysis of robot kinematic model with current wheel configuration
"""
import numpy as np
import matplotlib.pyplot as plt

def analyze_kinematic_model():
    # Current wheel configuration from RobotDescription.h
    wheel_angles_deg = [-30, 45, 135, -150]
    wheel_angles_rad = [angle * np.pi / 180 for angle in wheel_angles_deg]
    
    wheel_positions_m = [
        [0.045601, 0.080113],   # wheel 1: front-left
        [-0.064798, 0.065573],  # wheel 2: rear-left  
        [-0.064798, -0.065573], # wheel 3: rear-right
        [0.045601, -0.080113]   # wheel 4: front-right
    ]
    
    wheel_radius_m = 0.03225
    
    print("=== KINEMATIC MODEL ANALYSIS ===")
    print(f"Wheel angles (degrees): {wheel_angles_deg}")
    print(f"Wheel positions (m): {wheel_positions_m}")
    print(f"Wheel radius (m): {wheel_radius_m}")
    print()
    
    # Compute inverse Jacobian matrix
    num_wheels = len(wheel_angles_rad)
    J_inv = np.zeros((num_wheels, 3))
    
    for i in range(num_wheels):
        x_i, y_i = wheel_positions_m[i]
        beta_i = wheel_angles_rad[i]
        
        # Inverse kinematics: wheel_speed = J_inv * robot_velocity
        J_inv[i, 0] = (1.0 / wheel_radius_m) * np.cos(beta_i)  # x-velocity
        J_inv[i, 1] = (1.0 / wheel_radius_m) * np.sin(beta_i)  # y-velocity  
        J_inv[i, 2] = (1.0 / wheel_radius_m) * (x_i * np.sin(beta_i) - y_i * np.cos(beta_i))  # angular
    
    print("Inverse Jacobian Matrix (wheel_speeds = J_inv * robot_velocity):")
    print(J_inv)
    print()
    
    # Compute forward Jacobian using Moore-Penrose pseudoinverse
    J_forward = np.linalg.pinv(J_inv)
    
    print("Forward Jacobian Matrix (robot_velocity = J_forward * wheel_speeds):")
    print(J_forward)
    print()
    
    # Check if the system is well-conditioned
    condition_number = np.linalg.cond(J_inv)
    print(f"Condition number: {condition_number:.2f}")
    if condition_number > 100:
        print("⚠️  WARNING: High condition number indicates ill-conditioned system!")
    else:
        print("✅ Good condition number - system is well-conditioned")
    print()
    
    # Test forward-inverse consistency
    test_velocity = np.array([0.5, 0.3, 1.0])  # [vx, vy, omega]
    wheel_speeds = J_inv @ test_velocity
    recovered_velocity = J_forward @ wheel_speeds
    
    print("Forward-Inverse Consistency Test:")
    print(f"Input velocity: {test_velocity}")
    print(f"Wheel speeds: {wheel_speeds}")
    print(f"Recovered velocity: {recovered_velocity}")
    print(f"Error: {np.linalg.norm(test_velocity - recovered_velocity):.6f}")
    print()
    
    # Check for singular directions (directions robot cannot move)
    U, S, Vt = np.linalg.svd(J_inv)
    print("Singular Value Decomposition:")
    print(f"Singular values: {S}")
    
    if np.any(S < 1e-6):
        print("⚠️  WARNING: Near-zero singular values indicate robot cannot move in some directions!")
        null_space_directions = Vt[S < 1e-6, :]
        print(f"Null space directions: {null_space_directions}")
    else:
        print("✅ All singular values are good - robot can move in all directions")
    print()
    
    # Compare with ideal symmetric configuration
    print("=== COMPARISON WITH IDEAL SYMMETRIC CONFIG ===")
    ideal_angles_deg = [-45, 45, 135, -135]
    ideal_angles_rad = [angle * np.pi / 180 for angle in ideal_angles_deg]
    
    # For symmetric configuration, assume square layout
    robot_half_width = 0.075  # Approximate from current positions
    ideal_positions = [
        [robot_half_width, robot_half_width],    # front-left
        [-robot_half_width, robot_half_width],   # rear-left
        [-robot_half_width, -robot_half_width],  # rear-right
        [robot_half_width, -robot_half_width]    # front-right
    ]
    
    # Compute ideal inverse Jacobian
    J_inv_ideal = np.zeros((4, 3))
    for i in range(4):
        x_i, y_i = ideal_positions[i]
        beta_i = ideal_angles_rad[i]
        
        J_inv_ideal[i, 0] = (1.0 / wheel_radius_m) * np.cos(beta_i)
        J_inv_ideal[i, 1] = (1.0 / wheel_radius_m) * np.sin(beta_i) 
        J_inv_ideal[i, 2] = (1.0 / wheel_radius_m) * (x_i * np.sin(beta_i) - y_i * np.cos(beta_i))
    
    print(f"Ideal symmetric angles (degrees): {ideal_angles_deg}")
    print("Ideal Inverse Jacobian:")
    print(J_inv_ideal)
    
    ideal_condition = np.linalg.cond(J_inv_ideal)
    print(f"Ideal condition number: {ideal_condition:.2f}")
    print(f"Current condition number: {condition_number:.2f}")
    
    if condition_number > ideal_condition * 2:
        print("⚠️  Current configuration is significantly worse than ideal!")
    else:
        print("✅ Current configuration is reasonably close to ideal")

if __name__ == "__main__":
    analyze_kinematic_model()