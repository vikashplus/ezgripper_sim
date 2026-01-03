#!/usr/bin/env python3
"""
Test velocity actuator behavior
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
f1_actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
f2_actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
f2_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

print("=" * 80)
print("VELOCITY ACTUATOR TEST")
print("=" * 80)

# Test 1: No control (springs only)
print("\nTest 1: No control (ctrl=0)")
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator] = 0.0
data.ctrl[f2_actuator] = 0.0

for _ in range(1000):
    mujoco.mj_step(model, data)

f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]

print(f"  F1 Palm-L1 angle: {np.rad2deg(data.qpos[f1_palm_adr]):.1f}°")
print(f"  F1 L1-L2 angle: {np.rad2deg(data.qpos[f1_l1l2_adr]):.1f}°")
print(f"  F1 tendon length: {data.ten_length[f1_tendon]:.6f} m")
print(f"  F2 tendon length: {data.ten_length[f2_tendon]:.6f} m")

# Test 2: Closing command
print("\nTest 2: Closing (ctrl=-0.02 m/s)")
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator] = -0.02
data.ctrl[f2_actuator] = -0.02

print(f"  Initial tendon length: {data.ten_length[f1_tendon]:.6f} m")

for i in range(500):
    mujoco.mj_step(model, data)
    if i % 100 == 0:
        print(f"  Step {i:3d}: Palm={np.rad2deg(data.qpos[f1_palm_adr]):6.1f}°  L1L2={np.rad2deg(data.qpos[f1_l1l2_adr]):6.1f}°  Tendon={data.ten_length[f1_tendon]:.6f}m")

print(f"\n  Final F1 Palm-L1 angle: {np.rad2deg(data.qpos[f1_palm_adr]):.1f}°")
print(f"  Final F1 L1-L2 angle: {np.rad2deg(data.qpos[f1_l1l2_adr]):.1f}°")
print(f"  Final tendon length: {data.ten_length[f1_tendon]:.6f} m")

# Test 3: Opening command
print("\nTest 3: Opening (ctrl=+0.02 m/s)")
data.ctrl[f1_actuator] = 0.02
data.ctrl[f2_actuator] = 0.02

for i in range(500):
    mujoco.mj_step(model, data)
    if i % 100 == 0:
        print(f"  Step {i:3d}: Palm={np.rad2deg(data.qpos[f1_palm_adr]):6.1f}°  L1L2={np.rad2deg(data.qpos[f1_l1l2_adr]):6.1f}°  Tendon={data.ten_length[f1_tendon]:.6f}m")

print(f"\n  Final F1 Palm-L1 angle: {np.rad2deg(data.qpos[f1_palm_adr]):.1f}°")
print(f"  Final F1 L1-L2 angle: {np.rad2deg(data.qpos[f1_l1l2_adr]):.1f}°")
print(f"  Final tendon length: {data.ten_length[f1_tendon]:.6f} m")

print("\n" + "=" * 80)
print("Actuator info:")
print(f"  Type: {model.actuator_dyntype[f1_actuator]}")
print(f"  Ctrl range: {model.actuator_ctrlrange[f1_actuator]}")
print(f"  Force range: {model.actuator_forcerange[f1_actuator]}")
print(f"  Gain (kv): {model.actuator_gainprm[f1_actuator]}")
print("=" * 80)
