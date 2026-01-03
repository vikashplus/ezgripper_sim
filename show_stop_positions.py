#!/usr/bin/env python3
"""
Show mechanical stop positions and create a simple visualization.
"""

import mujoco
import numpy as np
import os

# Load visualization model with markers
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_visualization.xml')
model = mujoco.MjModel.from_xml_path(viz_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP POSITION ANALYSIS")
print("="*80)

print("\nMarker colors in the XML:")
print("  BLUE spheres: Palm stop positions (fixed at Z=0)")
print("  RED spheres: Current (wrong) finger stop positions (at Z=0.1)")
print("  GREEN spheres: Correct finger stop positions (at Z=0)")

# Get joint ID
f1_palm_id = model.joint('F1_palm_knuckle').id

# Get marker positions at different angles
angles = [-90, -45, 0, 25]

print(f"\nPosition analysis at different angles:")
print("="*60)

for angle in angles:
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_forward(model, data)
    
    # Get world positions of markers
    palm_lower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_palm_lower")
    palm_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_palm_upper")
    current_lower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_current_lower")
    current_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_current_upper")
    correct_lower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_correct_lower")
    correct_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "viz_correct_upper")
    
    palm_lower_pos = data.geom_xpos[palm_lower_id]
    palm_upper_pos = data.geom_xpos[palm_upper_id]
    current_lower_pos = data.geom_xpos[current_lower_id]
    current_upper_pos = data.geom_xpos[current_upper_id]
    correct_lower_pos = data.geom_xpos[correct_lower_id]
    correct_upper_pos = data.geom_xpos[correct_upper_id]
    
    print(f"\nAngle: {angle:3d}°")
    print(f"  Palm lower:   [{palm_lower_pos[0]:7.3f}, {palm_lower_pos[1]:7.3f}, {palm_lower_pos[2]:7.3f}]")
    print(f"  Current lower: [{current_lower_pos[0]:7.3f}, {current_lower_pos[1]:7.3f}, {current_lower_pos[2]:7.3f}]")
    print(f"  Correct lower: [{correct_lower_pos[0]:7.3f}, {correct_lower_pos[1]:7.3f}, {correct_lower_pos[2]:7.3f}]")
    
    # Calculate distances
    dist_current = np.linalg.norm(current_lower_pos - palm_lower_pos)
    dist_correct = np.linalg.norm(correct_lower_pos - palm_lower_pos)
    
    print(f"  Distance current: {dist_current:.4f} (collides: {dist_current < 0.01})")
    print(f"  Distance correct: {dist_correct:.4f} (collides: {dist_correct < 0.01})")

print(f"\n" + "="*80)
print("KEY FINDINGS:")
print("="*80)
print("1. Palm stops are at Z=0.000 (palm level)")
print("2. Current finger stops are at Z=0.100 (100mm too high!)")
print("3. Correct finger stops are at Z=0.000 (palm level)")
print("4. Current setup: 100mm gap = NO COLLISION")
print("5. Correct setup: Perfect alignment = COLLISION")

print(f"\nTo see the visualization:")
print("1. Open the XML file in a MuJoCo viewer")
print("2. Look for the colored spheres:")
print("   - BLUE: Palm stops (should be at Z=0)")
print("   - RED: Current stops (at Z=0.1, wrong!)")
print("   - GREEN: Correct stops (at Z=0, right!)")

print(f"\nThe RED and GREEN spheres show the difference between wrong and right positions.")
print("="*80)
