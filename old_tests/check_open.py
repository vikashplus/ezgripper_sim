#!/usr/bin/env python3
"""
Check gripper open state
"""
import mujoco
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')

f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]

print("=" * 80)
print("GRIPPER OPEN STATE CHECK")
print("=" * 80)

# Test 1: No control (springs only)
print("\nTest 1: No control (ctrl=0, springs push open)")
mujoco.mj_resetData(model, data)
data.ctrl[gripper_f1_id] = 0.0
data.ctrl[gripper_f2_id] = 0.0

for _ in range(2000):
    mujoco.mj_step(model, data)

print(f"  Palm-L1: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}° (spring ref: {np.rad2deg(model.qpos_spring[f1_palm_adr]):.2f}°)")
print(f"  L1-L2:   {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}° (spring ref: {np.rad2deg(model.qpos_spring[f1_l1l2_adr]):.2f}°)")
print(f"  Tendon:  {data.ten_length[f1_tendon]:.6f} m")

# Test 2: Maximum open command
print("\nTest 2: Maximum open command (ctrl=0.160)")
mujoco.mj_resetData(model, data)
data.ctrl[gripper_f1_id] = 0.160
data.ctrl[gripper_f2_id] = 0.160

for _ in range(2000):
    mujoco.mj_step(model, data)

print(f"  Palm-L1: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}° (range: {np.rad2deg(model.jnt_range[f1_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_palm, 1]):.1f}°)")
print(f"  L1-L2:   {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}° (range: {np.rad2deg(model.jnt_range[f1_l1l2, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_l1l2, 1]):.1f}°)")
print(f"  Tendon:  {data.ten_length[f1_tendon]:.6f} m (target: 0.160)")
print(f"  Error:   {(0.160 - data.ten_length[f1_tendon])*1000:.2f} mm")

# Test 3: Even longer tendon
print("\nTest 3: Extra long tendon (ctrl=0.180)")
mujoco.mj_resetData(model, data)
data.ctrl[gripper_f1_id] = 0.180
data.ctrl[gripper_f2_id] = 0.180

for _ in range(2000):
    mujoco.mj_step(model, data)

print(f"  Palm-L1: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}°")
print(f"  L1-L2:   {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}°")
print(f"  Tendon:  {data.ten_length[f1_tendon]:.6f} m (target: 0.180)")
print(f"  Error:   {(0.180 - data.ten_length[f1_tendon])*1000:.2f} mm")

# Check what the natural tendon length is
print("\n" + "=" * 80)
print("NATURAL TENDON LENGTH (at spring rest positions)")
print("=" * 80)

mujoco.mj_resetData(model, data)
# Set joints to spring reference positions
data.qpos[f1_palm_adr] = model.qpos_spring[f1_palm_adr]
data.qpos[f1_l1l2_adr] = model.qpos_spring[f1_l1l2_adr]
mujoco.mj_forward(model, data)

print(f"  Palm-L1 at spring ref: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}°")
print(f"  L1-L2 at spring ref:   {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}°")
print(f"  Natural tendon length: {data.ten_length[f1_tendon]:.6f} m")

print("\n" + "=" * 80)
