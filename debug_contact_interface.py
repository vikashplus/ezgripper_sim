#!/usr/bin/env python3
"""
Debug the contact interface to understand why collision isn't working.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("CONTACT INTERFACE DEBUG")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get collision geom IDs
f1_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1_tip")
f2_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f2_tip")

print(f"\nCollision geometry properties:")
print(f"f1_tip geom ID: {f1_tip_geom_id}")
print(f"f2_tip geom ID: {f2_tip_geom_id}")

# Check geom properties
f1_geom = model.geom(f1_tip_geom_id)
f2_geom = model.geom(f2_tip_geom_id)

print(f"\nf1_tip properties:")
print(f"  type: {f1_geom.type} (0=plane, 1=hfield, 2=sphere, 3=capsule, 4=box, 5=ellipsoid, 6=cylinder, 7=mesh)")
print(f"  contype: {f1_geom.contype}")
print(f"  conaffinity: {f1_geom.conaffinity}")
print(f"  size: {f1_geom.size}")
print(f"  pos: {f1_geom.pos}")

print(f"\nf2_tip properties:")
print(f"  type: {f2_geom.type}")
print(f"  contype: {f2_geom.contype}")
print(f"  conaffinity: {f2_geom.conaffinity}")
print(f"  size: {f2_geom.size}")
print(f"  pos: {f2_geom.pos}")

# Check contact pairs
print(f"\nContact pairs:")
for i in range(model.npair):
    pair = model.pair[i]
    geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair.geom1)
    geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair.geom2)
    print(f"  Pair {i}: {geom1_name} <-> {geom2_name}")
    print(f"    condim: {pair.condim}, margin: {pair.margin}, gap: {pair.gap}")

# Test with manual positioning to force contact
print(f"\nTesting manual positioning:")

# Reset and manually position fingers at contact point
mujoco.mj_resetData(model, data)

# Set joints to positions where we know contact should occur (from geometry debug)
data.qpos[f1_palm_id] = np.radians(25)  # Contact position
data.qpos[f2_palm_id] = np.radians(25)  
data.qpos[f1_tip_id] = 0.0
data.qpos[f2_tip_id] = 0.0

# Forward kinematics
mujoco.mj_forward(model, data)

# Check positions and contacts
f1_tip_pos = data.geom_xpos[f1_tip_geom_id]
f2_tip_pos = data.geom_xpos[f2_tip_geom_id]
distance = np.linalg.norm(f1_tip_pos - f2_tip_pos)

print(f"Manual positioning at 25°:")
print(f"  F1 tip pos: [{f1_tip_pos[0]:.4f}, {f1_tip_pos[1]:.4f}, {f1_tip_pos[2]:.4f}]")
print(f"  F2 tip pos: [{f2_tip_pos[0]:.4f}, {f2_tip_pos[1]:.4f}, {f2_tip_pos[2]:.4f}]")
print(f"  Distance: {distance:.6f}")
print(f"  Contacts: {data.ncon}")

if data.ncon > 0:
    print(f"  Contact details:")
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        dist = contact.dist
        print(f"    {i}: {geom1_name} <-> {geom2_name}, dist={dist:.6f}")
else:
    print(f"  No contacts detected!")
    
    # Check if geoms are in collision range
    print(f"  Checking collision detection:")
    for i in range(model.npair):
        pair = model.pair[i]
        if pair.geom1 == f1_tip_geom_id or pair.geom2 == f1_tip_geom_id:
            print(f"    f1_tip pair found: {pair.geom1} <-> {pair.geom2}")
            print(f"    margin: {pair.margin}, gap: {pair.gap}")

print("\n" + "="*80)
print("CONTACT INTERFACE DEBUG COMPLETE")
print("="*80)
