#!/usr/bin/env python3
"""
Test joint limits and mechanical constraints.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("JOINT LIMITS TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Check joint limits
print(f"\nJoint Limits:")
print(f"F1 Palm: [{np.degrees(model.jnt_range[f1_palm_id][0]):.1f}°, {np.degrees(model.jnt_range[f1_palm_id][1]):.1f}°]")
print(f"F1 Tip:  [{np.degrees(model.jnt_range[f1_tip_id][0]):.1f}°, {np.degrees(model.jnt_range[f1_tip_id][1]):.1f}°]")
print(f"F2 Palm: [{np.degrees(model.jnt_range[f2_palm_id][0]):.1f}°, {np.degrees(model.jnt_range[f2_palm_id][1]):.1f}°]")
print(f"F2 Tip:  [{np.degrees(model.jnt_range[f2_tip_id][0]):.1f}°, {np.degrees(model.jnt_range[f2_tip_id][1]):.1f}°]")

print(f"\nTesting joint limit enforcement:")

# Test extreme closing to see if limits are enforced
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.100  # Extreme closing
data.ctrl[f2_actuator_id] = 0.100

# Run for many steps
for i in range(500):
    mujoco.mj_step(model, data)

# Check final positions
f1_palm = np.degrees(data.qpos[f1_palm_id])
f1_tip = np.degrees(data.qpos[f1_tip_id])
f2_palm = np.degrees(data.qpos[f2_palm_id])
f2_tip = np.degrees(data.qpos[f2_tip_id])

print(f"\nFinal positions with extreme closing:")
print(f"F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
print(f"F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")

# Check if joints are at limits
f1_palm_at_min = abs(f1_palm - np.degrees(model.jnt_range[f1_palm_id][0])) < 1.0
f1_palm_at_max = abs(f1_palm - np.degrees(model.jnt_range[f1_palm_id][1])) < 1.0
f1_tip_at_min = abs(f1_tip - np.degrees(model.jnt_range[f1_tip_id][0])) < 1.0
f1_tip_at_max = abs(f1_tip - np.degrees(model.jnt_range[f1_tip_id][1])) < 1.0

print(f"\nJoint limit status:")
print(f"F1 Palm: at_min={f1_palm_at_min}, at_max={f1_palm_at_max}")
print(f"F1 Tip:  at_min={f1_tip_at_min}, at_max={f1_tip_at_max}")

# Check for contacts
print(f"\nContacts: {data.ncon}")
if data.ncon > 0:
    for i in range(min(data.ncon, 5)):  # Show first 5 contacts
        contact = data.contact[i]
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        print(f"  {i}: {geom1_name} <-> {geom2_name}")

print("\n" + "="*80)
print("JOINT LIMITS TEST COMPLETE")
print("="*80)
