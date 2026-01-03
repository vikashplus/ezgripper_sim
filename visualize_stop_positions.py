#!/usr/bin/env python3
"""
Visualize current vs correct mechanical stop positions.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP POSITION VISUALIZATION")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id

# Get current stop positions
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

# Calculate correct positions (from previous script)
data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)
finger_transform_lower = data.xpos[model.body('F1_L1').id]
correct_lower_relative = palm_lower_stop_pos - finger_transform_lower

data.qpos[f1_palm_id] = np.radians(25)
mujoco.mj_kinematics(model, data)
finger_transform_upper = data.xpos[model.body('F1_L1').id]
correct_upper_relative = palm_upper_stop_pos - finger_transform_upper

print(f"\nSTOP POSITION COMPARISON:")
print("="*50)

print(f"\nPALM STOPS (fixed):")
print(f"  Lower stop: [{palm_lower_stop_pos[0]:.4f}, {palm_lower_stop_pos[1]:.4f}, {palm_lower_stop_pos[2]:.4f}]")
print(f"  Upper stop: [{palm_upper_stop_pos[0]:.4f}, {palm_upper_stop_pos[1]:.4f}, {palm_upper_stop_pos[2]:.4f}]")

print(f"\nCURRENT FINGER STOPS:")
print(f"  Lower stop: pos=\"0.01 0 0\"")
print(f"  Upper stop: pos=\"-0.01 0 0\"")

print(f"\nCORRECT FINGER STOPS:")
print(f"  Lower stop: pos=\"{correct_lower_relative[0]:.4f} {correct_lower_relative[1]:.4f} {correct_lower_relative[2]:.4f}\"")
print(f"  Upper stop: pos=\"{correct_upper_relative[0]:.4f} {correct_upper_relative[1]:.4f} {correct_upper_relative[2]:.4f}\"")

print(f"\nVISUALIZATION:")
print("="*50)

# Create visual representation
print(f"\nAt -90° (lower limit):")
print(f"  Finger body position: [{finger_transform_lower[0]:.4f}, {finger_transform_lower[1]:.4f}, {finger_transform_lower[2]:.4f}]")
print(f"  Palm lower stop:      [{palm_lower_stop_pos[0]:.4f}, {palm_lower_stop_pos[1]:.4f}, {palm_lower_stop_pos[2]:.4f}]")
print(f"  Current finger stop:  [{finger_transform_lower[0] + 0.01:.4f}, {finger_transform_lower[1]:.4f}, {finger_transform_lower[2]:.4f}]")
print(f"  Correct finger stop:  [{finger_transform_lower[0] + correct_lower_relative[0]:.4f}, {finger_transform_lower[1] + correct_lower_relative[1]:.4f}, {finger_transform_lower[2] + correct_lower_relative[2]:.4f}]")

print(f"\nAt +25° (upper limit):")
print(f"  Finger body position: [{finger_transform_upper[0]:.4f}, {finger_transform_upper[1]:.4f}, {finger_transform_upper[2]:.4f}]")
print(f"  Palm upper stop:      [{palm_upper_stop_pos[0]:.4f}, {palm_upper_stop_pos[1]:.4f}, {palm_upper_stop_pos[2]:.4f}]")
print(f"  Current finger stop:  [{finger_transform_upper[0] - 0.01:.4f}, {finger_transform_upper[1]:.4f}, {finger_transform_upper[2]:.4f}]")
print(f"  Correct finger stop:  [{finger_transform_upper[0] + correct_upper_relative[0]:.4f}, {finger_transform_upper[1] + correct_upper_relative[1]:.4f}, {finger_transform_upper[2] + correct_upper_relative[2]:.4f}]")

print(f"\nDISTANCE ANALYSIS:")
print("="*50)

# Calculate distances at each limit
data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)

# Current stops
current_lower_world = finger_transform_lower + np.array([0.01, 0, 0])
correct_lower_world = finger_transform_lower + correct_lower_relative

dist_current_lower = np.linalg.norm(current_lower_world - palm_lower_stop_pos)
dist_correct_lower = np.linalg.norm(correct_lower_world - palm_lower_stop_pos)

print(f"\nAt -90°:")
print(f"  Distance (current): {dist_current_lower:.6f}")
print(f"  Distance (correct): {dist_correct_lower:.6f}")
print(f"  Current collides: {dist_current_lower < 0.01}")
print(f"  Correct collides: {dist_correct_lower < 0.01}")

# Upper limit
data.qpos[f1_palm_id] = np.radians(25)
mujoco.mj_kinematics(model, data)

current_upper_world = finger_transform_upper + np.array([-0.01, 0, 0])
correct_upper_world = finger_transform_upper + correct_upper_relative

dist_current_upper = np.linalg.norm(current_upper_world - palm_upper_stop_pos)
dist_correct_upper = np.linalg.norm(correct_upper_world - palm_upper_stop_pos)

print(f"\nAt +25°:")
print(f"  Distance (current): {dist_current_upper:.6f}")
print(f"  Distance (correct): {dist_correct_upper:.6f}")
print(f"  Current collides: {dist_current_upper < 0.01}")
print(f"  Correct collides: {dist_correct_upper < 0.01}")

print(f"\n3D VISUALIZATION GUIDE:")
print("="*50)
print(f"Imagine looking from the side (Y-axis view):")
print(f"")
print(f"Z-axis: Vertical (up/down)")
print(f"X-axis: Horizontal (forward/back)")
print(f"")
print(f"At -90° (finger pointing down):")
print(f"  Palm stop at:  X={palm_lower_stop_pos[0]:.3f}, Z={palm_lower_stop_pos[2]:.3f}")
print(f"  Current stop at: X={current_lower_world[0]:.3f}, Z={current_lower_world[2]:.3f}")
print(f"  Correct stop at: X={correct_lower_world[0]:.3f}, Z={correct_lower_world[2]:.3f}")
print(f"")
print(f"At +25° (finger pointing up):")
print(f"  Palm stop at:  X={palm_upper_stop_pos[0]:.3f}, Z={palm_upper_stop_pos[2]:.3f}")
print(f"  Current stop at: X={current_upper_world[0]:.3f}, Z={current_upper_world[2]:.3f}")
print(f"  Correct stop at: X={correct_upper_world[0]:.3f}, Z={correct_upper_world[2]:.3f}")

print(f"\n" + "="*80)
print("VALIDATION COMPLETE")
print("The 'correct' positions should have distances < 0.01 at their respective limits")
print("="*80)
