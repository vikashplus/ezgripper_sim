#!/usr/bin/env python3
"""
Simple diagnosis of finger asymmetry.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("SIMPLE FINGER ASYMMETRY DIAGNOSIS")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

print("\nTesting with IDENTICAL commands (0.150):")

# Reset and test
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.150
data.ctrl[f2_actuator_id] = 0.150

print(f"Initial state:")
print(f"  F1: Palm={np.degrees(data.qpos[f1_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f1_tip_id]):6.1f}°")
print(f"  F2: Palm={np.degrees(data.qpos[f2_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f2_tip_id]):6.1f}°")

# Run simulation
for i in range(100):
    mujoco.mj_step(model, data)

print(f"\nAfter 100 steps:")
print(f"  F1: Palm={np.degrees(data.qpos[f1_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f1_tip_id]):6.1f}°")
print(f"  F2: Palm={np.degrees(data.qpos[f2_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f2_tip_id]):6.1f}°")

palm_diff = np.degrees(data.qpos[f1_palm_id] - data.qpos[f2_palm_id])
tip_diff = np.degrees(data.qpos[f1_tip_id] - data.qpos[f2_tip_id])
print(f"\nDifferences:")
print(f"  Palm: F1 - F2 = {palm_diff:6.1f}°")
print(f"  Tip:  F1 - F2 = {tip_diff:6.1f}°")

# Check tendon lengths
f1_tendon_len = data.ten_length[0]
f2_tendon_len = data.ten_length[1]
print(f"\nTendon lengths:")
print(f"  F1: {f1_tendon_len:.6f}")
print(f"  F2: {f2_tendon_len:.6f}")
print(f"  Diff: {f1_tendon_len - f2_tendon_len:.6f}")

# Check commands
print(f"\nActuator commands:")
print(f"  F1: {data.ctrl[f1_actuator_id]:.6f}")
print(f"  F2: {data.ctrl[f2_actuator_id]:.6f}")
print(f"  Diff: {data.ctrl[f1_actuator_id] - data.ctrl[f2_actuator_id]:.6f}")

print("\n" + "="*80)
print("DIAGNOSIS COMPLETE")
print("="*80)
