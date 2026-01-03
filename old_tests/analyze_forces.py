#!/usr/bin/env python3
"""
Analyze forces preventing L1 from closing
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
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'flex_tendon')
f1_palm_l1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1_l2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

# Get qpos addresses
f1_palm_adr = model.jnt_qposadr[f1_palm_l1_id]
f1_l1l2_adr = model.jnt_qposadr[f1_l1_l2_id]

print("=" * 80)
print("FORCE ANALYSIS: Why L1 Won't Close")
print("=" * 80)

# Test at different control levels
for ctrl_val in [0.0, -0.25, -0.5, -0.75, -1.0]:
    mujoco.mj_resetData(model, data)
    data.ctrl[gripper_actuator_id] = ctrl_val
    
    # Let it settle
    for _ in range(1000):
        mujoco.mj_step(model, data)
    
    # Get joint angles
    palm_angle = data.qpos[f1_palm_adr]
    l1l2_angle = data.qpos[f1_l1l2_adr]
    
    # Get spring forces
    palm_springref = model.qpos_spring[f1_palm_adr]
    palm_stiffness = model.jnt_stiffness[f1_palm_l1_id]
    palm_deflection = palm_angle - palm_springref
    palm_spring_force = palm_stiffness * palm_deflection
    
    l1l2_springref = model.qpos_spring[f1_l1l2_adr]
    l1l2_stiffness = model.jnt_stiffness[f1_l1_l2_id]
    l1l2_deflection = l1l2_angle - l1l2_springref
    l1l2_spring_force = l1l2_stiffness * l1l2_deflection
    
    # Get tendon force (actuator_force is just ctrl, need to multiply by gear)
    actuator_gear = model.actuator_gear[gripper_actuator_id, 0]
    tendon_force = data.ctrl[gripper_actuator_id] * actuator_gear
    
    # Get moment arms
    qvel_palm_adr = model.jnt_dofadr[f1_palm_l1_id]
    qvel_l1l2_adr = model.jnt_dofadr[f1_l1_l2_id]
    moment_arm_palm = data.ten_J[tendon_id, qvel_palm_adr]
    moment_arm_l1l2 = data.ten_J[tendon_id, qvel_l1l2_adr]
    
    # Calculate torques
    tendon_torque_palm = tendon_force * moment_arm_palm
    tendon_torque_l1l2 = tendon_force * moment_arm_l1l2
    
    # Net torques
    net_torque_palm = tendon_torque_palm + palm_spring_force
    net_torque_l1l2 = tendon_torque_l1l2 + l1l2_spring_force
    
    print(f"\n{'=' * 80}")
    print(f"Control: {ctrl_val:.2f}")
    print(f"{'=' * 80}")
    print(f"\nPalm-L1 Joint:")
    print(f"  Angle: {np.rad2deg(palm_angle):.1f}°")
    print(f"  Spring force: {palm_spring_force:.6f} N·m (pulling open)")
    print(f"  Moment arm: {moment_arm_palm:.6f} m/rad")
    print(f"  Tendon force: {tendon_force:.3f} N")
    print(f"  Tendon torque: {tendon_torque_palm:.6f} N·m (closing)")
    print(f"  NET torque: {net_torque_palm:.6f} N·m")
    
    print(f"\nL1-L2 Joint:")
    print(f"  Angle: {np.rad2deg(l1l2_angle):.1f}°")
    print(f"  Spring force: {l1l2_spring_force:.6f} N·m (pulling open)")
    print(f"  Moment arm: {moment_arm_l1l2:.6f} m/rad")
    print(f"  Tendon force: {tendon_force:.3f} N")
    print(f"  Tendon torque: {tendon_torque_l1l2:.6f} N·m (closing)")
    print(f"  NET torque: {net_torque_l1l2:.6f} N·m")
    
    # Ratio analysis
    if abs(moment_arm_l1l2) > 0.0001:
        moment_ratio = abs(moment_arm_palm / moment_arm_l1l2)
        print(f"\nMoment Arm Ratio (Palm/L1L2): {moment_ratio:.3f}")
        print(f"  → Palm-L1 has {moment_ratio:.1f}x MORE leverage than L1-L2")

print("\n" + "=" * 80)
print("ANALYSIS COMPLETE")
print("=" * 80)
