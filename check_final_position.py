#!/usr/bin/env python3
"""
Check what's limiting the gripper at -19.9°.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FINAL POSITION ANALYSIS")
print("="*80)

# Set to final position from test
f1_palm_id = model.joint('F1_palm_knuckle').id
data.qpos[f1_palm_id] = np.radians(-19.9)

# Update kinematics
mujoco.mj_kinematics(model, data)

print(f"\nAt -19.9°:")

# Check mechanical stop distances
palm_lower_pos = data.geom_xpos[model.geom('palm_stop_f1_lower').id]
palm_upper_pos = data.geom_xpos[model.geom('palm_stop_f1_upper').id]
finger_lower_pos = data.geom_xpos[model.geom('f1l1_stop_lower').id]
finger_upper_pos = data.geom_xpos[model.geom('f1l1_stop_upper').id]

dist_lower = np.linalg.norm(palm_lower_pos - finger_lower_pos)
dist_upper = np.linalg.norm(palm_upper_pos - finger_upper_pos)

print(f"  Lower stop distance: {dist_lower:.6f}")
print(f"  Upper stop distance: {dist_upper:.6f}")

# Step and check contacts
mujoco.mj_step(model, data)
print(f"  Contacts: {data.ncon}")

for i in range(data.ncon):
    contact = data.contact[i]
    geom1_name = model.geom(contact.geom1).name if model.geom(contact.geom1).name else f"geom_{contact.geom1}"
    geom2_name = model.geom(contact.geom2).name if model.geom(contact.geom2).name else f"geom_{contact.geom2}"
    print(f"    Contact {i}: {geom1_name} ↔ {geom2_name} (dist={contact.dist:.6f})")

# Check spring torques
torque_palm = data.qfrc_passive[f1_palm_id]
print(f"  Spring torque: {torque_palm:.6f} N·m")

# Check if at -90° would be better
print(f"\n" + "="*50)
print("CHECKING AT -90°:")
print("="*50)

data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)

palm_lower_pos = data.geom_xpos[model.geom('palm_stop_f1_lower').id]
finger_lower_pos = data.geom_xpos[model.geom('f1l1_stop_lower').id]
dist_lower = np.linalg.norm(palm_lower_pos - finger_lower_pos)

print(f"At -90°:")
print(f"  Lower stop distance: {dist_lower:.6f}")
print(f"  Should collide if distance < 0.01")

# Check equilibrium position
print(f"\n" + "="*50)
print("EQUILIBRIUM ANALYSIS:")
print("="*50)

# Find where spring torque = 0
for angle in np.linspace(-90, 25, 24):
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_kinematics(model, data)
    torque = data.qfrc_passive[f1_palm_id]
    
    if abs(torque) < 0.001:
        print(f"  Spring equilibrium at {angle:.1f}° (torque={torque:.6f})")
        break

print("="*80)
