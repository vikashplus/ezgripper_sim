#!/usr/bin/env python3
"""
Test finger tip collision detection and forces.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FINGER TIP COLLISION TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

# Get geometry IDs
f1_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1_tip")
f2_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f2_tip")

print(f"Finger tip geometry IDs: F1={f1_tip_geom_id}, F2={f2_tip_geom_id}")

# Check geometry properties
print(f"\nF1 Tip Geometry Properties:")
print(f"  Type: {model.geom_type[f1_tip_geom_id]}")
print(f"  Contype: {model.geom_contype[f1_tip_geom_id]}")
print(f"  Conaffinity: {model.geom_conaffinity[f1_tip_geom_id]}")
print(f"  Group: {model.geom_group[f1_tip_geom_id]}")

print(f"\nF2 Tip Geometry Properties:")
print(f"  Type: {model.geom_type[f2_tip_geom_id]}")
print(f"  Contype: {model.geom_contype[f2_tip_geom_id]}")
print(f"  Conaffinity: {model.geom_conaffinity[f2_tip_geom_id]}")
print(f"  Group: {model.geom_group[f2_tip_geom_id]}")

# Test collision at contact position
print(f"\nTesting collision at finger contact position:")

# Reset and close to contact
mujoco.mj_resetData(model, data)
data.ctrl[0] = 0.140  # Close to contact position
data.ctrl[1] = 0.140

# Run to steady state
for i in range(300):
    mujoco.mj_step(model, data)

# Check contacts
print(f"\nContact Analysis:")
print(f"  Total contacts: {data.ncon}")

if data.ncon > 0:
    print(f"\nContact Details:")
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        
        # Get contact force
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        force_magnitude = np.linalg.norm(force[:3])
        
        print(f"  Contact {i}: {geom1_name} <-> {geom2_name}")
        print(f"    Distance: {contact.dist:.6f}")
        print(f"    Force: {force_magnitude:.6f} N")
        print(f"    Contype: {contact.contype}")
        print(f"    Conaffinity: {contact.conaffinity}")
        
        # Check if this is finger tip contact
        if (contact.geom1 == f1_tip_geom_id and contact.geom2 == f2_tip_geom_id) or \
           (contact.geom1 == f2_tip_geom_id and contact.geom2 == f1_tip_geom_id):
            print(f"    *** FINGER TIP CONTACT DETECTED ***")
else:
    print(f"  No contacts detected")

print("\n" + "="*80)
print("COLLISION TEST COMPLETE")
print("="*80)
