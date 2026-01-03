#!/usr/bin/env python3
"""
Test if gripper can reach -90° with mesh collisions disabled.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("TEST -90° WITH MESH COLLISIONS DISABLED")
print("="*80)

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id

# Manually set to -90°
data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)

print(f"\nAt -90°:")

# Check mechanical stop distances
palm_lower_pos = data.geom_xpos[model.geom('palm_stop_f1_lower').id]
finger_lower_pos = data.geom_xpos[model.geom('f1l1_stop_lower').id]
dist_lower = np.linalg.norm(palm_lower_pos - finger_lower_pos)

print(f"  Lower stop distance: {dist_lower:.6f}")
print(f"  Should collide if distance < 0.01")

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

# Check if spring torque is zero at -90°
print(f"\n" + "="*50)
print("SPRING TORQUE ANALYSIS:")
print("="*50)

for angle in [-90, -45, -20, 0, 25]:
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_kinematics(model, data)
    torque = data.qfrc_passive[f1_palm_id]
    print(f"  {angle:3.0f}°: Spring torque = {torque:.6f} N·m")

print("="*80)
