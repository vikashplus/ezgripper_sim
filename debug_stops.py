#!/usr/bin/env python3
"""
Debug mechanical stop collisions in detail.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("DETAILED STOP COLLISION DEBUG")
print("="*80)

# Set joint to -0.8° (where we saw collisions)
f1_palm_id = model.joint('F1_palm_knuckle').id
data.qpos[f1_palm_id] = np.radians(-0.8)

# Update kinematics
mujoco.mj_kinematics(model, data)

# Get detailed position info
palm_lower_id = model.geom('palm_stop_f1_lower').id
finger_lower_id = model.geom('f1l1_stop_lower').id

palm_lower_pos = data.geom_xpos[palm_lower_id]
finger_lower_pos = data.geom_xpos[finger_lower_id]

print(f"\nAt -0.8° (problem angle):")
print(f"Palm lower stop position: [{palm_lower_pos[0]:.6f}, {palm_lower_pos[1]:.6f}, {palm_lower_pos[2]:.6f}]")
print(f"Finger lower stop position: [{finger_lower_pos[0]:.6f}, {finger_lower_pos[1]:.6f}, {finger_lower_pos[2]:.6f}]")

distance = np.linalg.norm(palm_lower_pos - finger_lower_pos)
print(f"Distance: {distance:.6f}")

# Check contact
mujoco.mj_step(model, data)
print(f"\nContacts after step: {data.ncon}")
for i in range(data.ncon):
    contact = data.contact[i]
    if contact.geom1 == palm_lower_id or contact.geom2 == palm_lower_id:
        if contact.geom1 == finger_lower_id or contact.geom2 == finger_lower_id:
            print(f"  Contact {i}: palm_stop_f1_lower ↔ f1l1_stop_lower (dist={contact.dist:.6f})")

# Check all geom positions
print("\n" + "="*50)
print("ALL STOP POSITIONS:")
print("="*50)

stops = [
    ('palm_stop_f1_lower', 'palm'),
    ('palm_stop_f1_upper', 'palm'),
    ('f1l1_stop_lower', 'finger'),
    ('f1l1_stop_upper', 'finger'),
]

for name, body in stops:
    geom_id = model.geom(name).id
    pos = data.geom_xpos[geom_id]
    print(f"{name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

print("\n" + "="*80)
print("ISSUE ANALYSIS:")
print("If stops are colliding at -0.8°, their positions are wrong")
print("They should only collide at -90° (lower) and +25° (upper)")
print("="*80)
