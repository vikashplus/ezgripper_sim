#!/usr/bin/env python3
"""
Calculate correct mechanical stop positions for -90° and +25° limits.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("CALCULATING CORRECT STOP POSITIONS")
print("="*80)

# Get joint and body info
f1_palm_id = model.joint('F1_palm_knuckle').id
finger_base_pos = model.body('F1_L1').pos
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

print(f"Finger base position: {finger_base_pos}")
print(f"Palm lower stop position: {palm_lower_stop_pos}")
print(f"Palm upper stop position: {palm_upper_stop_pos}")

print("\n" + "="*50)
print("CALCULATING FINGER STOP POSITIONS")
print("="*50)

# Calculate where finger stops should be positioned (relative to finger base)
# to collide with palm stops at the correct joint angles

# At -90° (lower limit)
data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)

# We want finger lower stop to be at palm lower stop position
# So relative position = palm_stop_pos - finger_base_transformed
finger_transform = data.xpos[model.body('F1_L1').id]
correct_lower_relative = palm_lower_stop_pos - finger_transform

print(f"\nAt -90° (lower limit):")
print(f"  Finger body position: [{finger_transform[0]:.6f}, {finger_transform[1]:.6f}, {finger_transform[2]:.6f}]")
print(f"  Required finger lower stop (relative): [{correct_lower_relative[0]:.6f}, {correct_lower_relative[1]:.6f}, {correct_lower_relative[2]:.6f}]")

# At +25° (upper limit)
data.qpos[f1_palm_id] = np.radians(25)
mujoco.mj_kinematics(model, data)

finger_transform = data.xpos[model.body('F1_L1').id]
correct_upper_relative = palm_upper_stop_pos - finger_transform

print(f"\nAt +25° (upper limit):")
print(f"  Finger body position: [{finger_transform[0]:.6f}, {finger_transform[1]:.6f}, {finger_transform[2]:.6f}]")
print(f"  Required finger upper stop (relative): [{correct_upper_relative[0]:.6f}, {correct_upper_relative[1]:.6f}, {correct_upper_relative[2]:.6f}]")

print("\n" + "="*50)
print("CORRECT POSITIONS:")
print("="*50)
print(f"Current finger lower stop: pos=\"0.01 0 0\"")
print(f"Correct finger lower stop: pos=\"{correct_lower_relative[0]:.4f} {correct_lower_relative[1]:.4f} {correct_lower_relative[2]:.4f}\"")
print(f"Current finger upper stop: pos=\"-0.01 0 0\"")
print(f"Correct finger upper stop: pos=\"{correct_upper_relative[0]:.4f} {correct_upper_relative[1]:.4f} {correct_upper_relative[2]:.4f}\"")

print("\n" + "="*80)
print("RECOMMENDED CHANGES:")
print("Update finger stop positions to these calculated values")
print("This ensures stops only collide at correct joint limits")
print("="*80)
