#!/usr/bin/env python3
"""
Check mechanical stop positions at different joint angles.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP POSITION ANALYSIS")
print("="*80)

# Get joint and geom IDs
f1_palm_id = model.joint('F1_palm_knuckle').id

# Get palm stop positions
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

# Get finger stop positions (relative to finger body)
finger_lower_stop_pos = model.geom('f1l1_stop_lower').pos
finger_upper_stop_pos = model.geom('f1l1_stop_upper').pos

# Get finger body position
finger_base_pos = model.body('F1_L1').pos

print(f"\nFinger base position: {finger_base_pos}")
print(f"Palm lower stop: {palm_lower_stop_pos}")
print(f"Palm upper stop: {palm_upper_stop_pos}")
print(f"Finger lower stop (relative): {finger_lower_stop_pos}")
print(f"Finger upper stop (relative): {finger_upper_stop_pos}")

print("\n" + "="*50)
print("STOP POSITIONS AT DIFFERENT JOINT ANGLES")
print("="*50)

# Test at different joint angles
test_angles = [-90, -45, 0, 25]  # degrees

for angle in test_angles:
    # Set joint angle
    data.qpos[f1_palm_id] = np.radians(angle)
    
    # Update kinematics
    mujoco.mj_kinematics(model, data)
    
    # Get finger stop world positions
    finger_lower_world = data.geom_xpos[model.geom('f1l1_stop_lower').id]
    finger_upper_world = data.geom_xpos[model.geom('f1l1_stop_upper').id]
    
    print(f"\nAt {angle:3.0f}°:")
    print(f"  Finger lower stop world: [{finger_lower_world[0]:.4f}, {finger_lower_world[1]:.4f}, {finger_lower_world[2]:.4f}]")
    print(f"  Finger upper stop world: [{finger_upper_world[0]:.4f}, {finger_upper_world[1]:.4f}, {finger_upper_world[2]:.4f}]")
    
    # Check distances to palm stops
    dist_lower = np.linalg.norm(finger_lower_world - palm_lower_stop_pos)
    dist_upper = np.linalg.norm(finger_upper_world - palm_upper_stop_pos)
    
    print(f"  Distance to palm lower stop: {dist_lower:.4f}")
    print(f"  Distance to palm upper stop: {dist_upper:.4f}")
    
    # Check if stops are colliding (very small distance)
    if dist_lower < 0.01:
        print(f"  → LOWER STOPS COLLIDING at {angle}°")
    if dist_upper < 0.01:
        print(f"  → UPPER STOPS COLLIDING at {angle}°")

print("\n" + "="*80)
print("ANALYSIS:")
print("Stops should only collide at joint limits (-90° and +25°)")
print("If they collide at other angles, positions are wrong")
print("="*80)
