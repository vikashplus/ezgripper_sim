#!/usr/bin/env python3
"""
Test mechanical stops effectiveness.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOPS TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Check mechanical stop geoms
print(f"\nMechanical stop geoms:")
stop_geoms = []
for i in range(model.ngeom):
    geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    if geom_name and 'stop' in geom_name.lower():
        pos = model.geom_pos[i]
        size = model.geom_size[i]
        contype = model.geom_contype[i]
        conaffinity = model.geom_conaffinity[i]
        print(f"  {geom_name}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f})")
        print(f"    contype={contype}, conaffinity={conaffinity}")
        stop_geoms.append(i)

print(f"\nTesting mechanical stops with extreme positions:")

# Test 1: Try to rotate L1 beyond limits
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.050  # Extreme closing
data.ctrl[f2_actuator_id] = 0.050

# Run for many steps
for i in range(1000):
    mujoco.mj_step(model, data)

# Check final positions
f1_palm = np.degrees(data.qpos[f1_palm_id])
f1_tip = np.degrees(data.qpos[f1_tip_id])
f2_palm = np.degrees(data.qpos[f2_palm_id])
f2_tip = np.degrees(data.qpos[f2_tip_id])

print(f"\nAfter extreme closing:")
print(f"F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
print(f"F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")

# Check for contacts
print(f"\nContacts: {data.ncon}")
if data.ncon > 0:
    for i in range(min(data.ncon, 10)):  # Show first 10 contacts
        contact = data.contact[i]
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        force = np.linalg.norm(data.efc_force[contact.efc_address:contact.efc_address+6])
        print(f"  {i}: {geom1_name} <-> {geom2_name}, force={force:.1f}N")
else:
    print("  No contacts detected!")

# Test 2: Try to rotate L2 backward
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.200  # Open position
data.ctrl[f2_actuator_id] = 0.200

# Run for many steps
for i in range(500):
    mujoco.mj_step(model, data)

# Check final positions
f1_palm = np.degrees(data.qpos[f1_palm_id])
f1_tip = np.degrees(data.qpos[f1_tip_id])
f2_palm = np.degrees(data.qpos[f2_palm_id])
f2_tip = np.degrees(data.qpos[f2_tip_id])

print(f"\nAfter opening:")
print(f"F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
print(f"F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")

# Check for contacts
print(f"\nContacts: {data.ncon}")
if data.ncon > 0:
    for i in range(min(data.ncon, 10)):  # Show first 10 contacts
        contact = data.contact[i]
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        force = np.linalg.norm(data.efc_force[contact.efc_address:contact.efc_address+6])
        print(f"  {i}: {geom1_name} <-> {geom2_name}, force={force:.1f}N")
else:
    print("  No contacts detected!")

print("\n" + "="*80)
print("MECHANICAL STOPS TEST COMPLETE")
print("="*80)
