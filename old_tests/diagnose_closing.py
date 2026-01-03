#!/usr/bin/env python3
"""
Diagnose why L1 stops rotating inward
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
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f2_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
f2_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')

print("=" * 80)
print("JOINT LIMIT DIAGNOSIS")
print("=" * 80)

# Print joint ranges
f1_palm_adr = model.jnt_qposadr[f1_palm]
f2_palm_adr = model.jnt_qposadr[f2_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]
f2_l1l2_adr = model.jnt_qposadr[f2_l1l2]

print("\nJoint Ranges (from model):")
print(f"  F1_palm_knuckle: {np.rad2deg(model.jnt_range[f1_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_palm, 1]):.1f}°")
print(f"  F2_palm_knuckle: {np.rad2deg(model.jnt_range[f2_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f2_palm, 1]):.1f}°")
print(f"  F1_knuckle_tip:  {np.rad2deg(model.jnt_range[f1_l1l2, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_l1l2, 1]):.1f}°")
print(f"  F2_knuckle_tip:  {np.rad2deg(model.jnt_range[f2_l1l2, 0]):.1f}° to {np.rad2deg(model.jnt_range[f2_l1l2, 1]):.1f}°")

print("\nSpring References:")
print(f"  F1_palm_knuckle: {np.rad2deg(model.qpos_spring[f1_palm_adr]):.1f}°")
print(f"  F2_palm_knuckle: {np.rad2deg(model.qpos_spring[f2_palm_adr]):.1f}°")
print(f"  F1_knuckle_tip:  {np.rad2deg(model.qpos_spring[f1_l1l2_adr]):.1f}°")
print(f"  F2_knuckle_tip:  {np.rad2deg(model.qpos_spring[f2_l1l2_adr]):.1f}°")

# Simulate closing
print("\n" + "=" * 80)
print("CLOSING SIMULATION")
print("=" * 80)

mujoco.mj_resetData(model, data)

# Command fully closed
target_length = 0.135
data.ctrl[f1_actuator] = target_length
data.ctrl[f2_actuator] = target_length

print(f"\nCommanding tendon length: {target_length:.4f} m")
print("\nSimulating for 5 seconds...")

for i in range(5000):  # 5 seconds at 1ms timestep
    mujoco.mj_step(model, data)
    
    if i % 1000 == 0:
        print(f"\nTime: {data.time:.2f}s")
        print(f"  F1 Palm-L1: {np.rad2deg(data.qpos[f1_palm_adr]):7.2f}°  (range: {np.rad2deg(model.jnt_range[f1_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_palm, 1]):.1f}°)")
        print(f"  F2 Palm-L1: {np.rad2deg(data.qpos[f2_palm_adr]):7.2f}°  (range: {np.rad2deg(model.jnt_range[f2_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f2_palm, 1]):.1f}°)")
        print(f"  F1 L1-L2:   {np.rad2deg(data.qpos[f1_l1l2_adr]):7.2f}°  (range: {np.rad2deg(model.jnt_range[f1_l1l2, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_l1l2, 1]):.1f}°)")
        print(f"  F2 L1-L2:   {np.rad2deg(data.qpos[f2_l1l2_adr]):7.2f}°  (range: {np.rad2deg(model.jnt_range[f2_l1l2, 0]):.1f}° to {np.rad2deg(model.jnt_range[f2_l1l2, 1]):.1f}°)")
        print(f"  F1 Tendon:  {data.ten_length[f1_tendon]:.6f} m (target: {target_length:.4f})")
        print(f"  F2 Tendon:  {data.ten_length[f2_tendon]:.6f} m (target: {target_length:.4f})")
        print(f"  F1 Actuator force: {data.actuator_force[f1_actuator]:.2f} N (limit: ±50 N)")
        print(f"  F2 Actuator force: {data.actuator_force[f2_actuator]:.2f} N (limit: ±50 N)")

# Final state
print("\n" + "=" * 80)
print("FINAL STATE")
print("=" * 80)
print(f"\nF1 Palm-L1: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}°")
print(f"F2 Palm-L1: {np.rad2deg(data.qpos[f2_palm_adr]):.2f}°")
print(f"F1 L1-L2:   {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}°")
print(f"F2 L1-L2:   {np.rad2deg(data.qpos[f2_l1l2_adr]):.2f}°")

# Check what's limiting
print("\n" + "=" * 80)
print("LIMITING FACTORS")
print("=" * 80)

# Check if at joint limits
if abs(data.qpos[f1_palm_adr] - model.jnt_range[f1_palm, 0]) < 0.01:
    print("✗ F1 Palm-L1 at MINIMUM joint limit")
elif abs(data.qpos[f1_palm_adr] - model.jnt_range[f1_palm, 1]) < 0.01:
    print("✗ F1 Palm-L1 at MAXIMUM joint limit")
else:
    print(f"✓ F1 Palm-L1 NOT at joint limit (current: {np.rad2deg(data.qpos[f1_palm_adr]):.1f}°, range: {np.rad2deg(model.jnt_range[f1_palm, 0]):.1f}° to {np.rad2deg(model.jnt_range[f1_palm, 1]):.1f}°)")

# Check if force saturated
if abs(data.actuator_force[f1_actuator]) > 45:
    print(f"✗ F1 Actuator force SATURATED ({data.actuator_force[f1_actuator]:.1f} N, limit: ±50 N)")
else:
    print(f"✓ F1 Actuator force NOT saturated ({data.actuator_force[f1_actuator]:.1f} N)")

# Check tendon error
tendon_error = target_length - data.ten_length[f1_tendon]
print(f"\nTendon position error: {tendon_error*1000:.2f} mm")
if abs(tendon_error) > 0.001:
    print("✗ Large tendon error - actuator cannot reach target")
else:
    print("✓ Tendon at target position")

print("\n" + "=" * 80)
