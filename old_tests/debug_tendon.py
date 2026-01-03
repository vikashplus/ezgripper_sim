#!/usr/bin/env python3
"""
Diagnostic script to analyze tendon routing and joint behavior
"""
import mujoco
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("=" * 80)
print("EZGRIPPER TENDON DIAGNOSTIC")
print("=" * 80)

# Get IDs
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'flex_tendon')

# Joint IDs
f1_palm_l1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1_l2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_palm_l1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_l1_l2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

print("\n" + "=" * 80)
print("JOINT CONFIGURATION")
print("=" * 80)

# Print joint info
joints = [
    ('F1_palm_knuckle', f1_palm_l1_id),
    ('F1_knuckle_tip', f1_l1_l2_id),
    ('F2_palm_knuckle', f2_palm_l1_id),
    ('F2_knuckle_tip', f2_l1_l2_id)
]

for name, jid in joints:
    jnt_adr = model.jnt_qposadr[jid]
    stiffness = model.jnt_stiffness[jid]
    springref = model.qpos_spring[jnt_adr]
    range_min = model.jnt_range[jid, 0]
    range_max = model.jnt_range[jid, 1]
    
    print(f"\n{name}:")
    print(f"  Range: [{range_min:.4f}, {range_max:.4f}] rad = [{np.rad2deg(range_min):.1f}°, {np.rad2deg(range_max):.1f}°]")
    print(f"  Stiffness: {stiffness:.6f}")
    print(f"  Springref: {springref:.4f} rad = {np.rad2deg(springref):.1f}°")

print("\n" + "=" * 80)
print("TENDON INFORMATION")
print("=" * 80)

# Tendon info
print(f"\nTendon: flex_tendon (ID: {tendon_id})")
print(f"  Limited: {model.tendon_limited[tendon_id]}")
if model.tendon_limited[tendon_id]:
    print(f"  Range: [{model.tendon_range[tendon_id, 0]:.6f}, {model.tendon_range[tendon_id, 1]:.6f}]")

print("\n" + "=" * 80)
print("SIMULATION: OPEN POSITION (ctrl=0)")
print("=" * 80)

# Reset and step
mujoco.mj_resetData(model, data)
data.ctrl[gripper_actuator_id] = 0.0  # Fully open

# Let it settle
for _ in range(1000):
    mujoco.mj_step(model, data)

print(f"\nActuator control: {data.ctrl[gripper_actuator_id]:.3f}")
print(f"Tendon length: {data.ten_length[tendon_id]:.6f} m")

for name, jid in joints:
    jnt_adr = model.jnt_qposadr[jid]
    angle_rad = data.qpos[jnt_adr]
    angle_deg = np.rad2deg(angle_rad)
    
    # Calculate spring force
    springref = model.qpos_spring[jnt_adr]
    stiffness = model.jnt_stiffness[jid]
    deflection = angle_rad - springref
    spring_force = stiffness * deflection
    
    print(f"\n{name}:")
    print(f"  Angle: {angle_rad:.4f} rad = {angle_deg:.1f}°")
    print(f"  Spring deflection: {deflection:.4f} rad = {np.rad2deg(deflection):.1f}°")
    print(f"  Spring force: {spring_force:.6f} N·m")

print("\n" + "=" * 80)
print("SIMULATION: HALF CLOSED (ctrl=-0.5)")
print("=" * 80)

mujoco.mj_resetData(model, data)
data.ctrl[gripper_actuator_id] = -0.5

for _ in range(1000):
    mujoco.mj_step(model, data)

print(f"\nActuator control: {data.ctrl[gripper_actuator_id]:.3f}")
print(f"Tendon length: {data.ten_length[tendon_id]:.6f} m")

for name, jid in joints:
    jnt_adr = model.jnt_qposadr[jid]
    angle_rad = data.qpos[jnt_adr]
    angle_deg = np.rad2deg(angle_rad)
    
    springref = model.qpos_spring[jnt_adr]
    stiffness = model.jnt_stiffness[jid]
    deflection = angle_rad - springref
    spring_force = stiffness * deflection
    
    print(f"\n{name}:")
    print(f"  Angle: {angle_rad:.4f} rad = {angle_deg:.1f}°")
    print(f"  Spring deflection: {deflection:.4f} rad = {np.rad2deg(deflection):.1f}°")
    print(f"  Spring force: {spring_force:.6f} N·m")

print("\n" + "=" * 80)
print("SIMULATION: FULLY CLOSED (ctrl=-1.0)")
print("=" * 80)

mujoco.mj_resetData(model, data)
data.ctrl[gripper_actuator_id] = -1.0

for _ in range(1000):
    mujoco.mj_step(model, data)

print(f"\nActuator control: {data.ctrl[gripper_actuator_id]:.3f}")
print(f"Tendon length: {data.ten_length[tendon_id]:.6f} m")

for name, jid in joints:
    jnt_adr = model.jnt_qposadr[jid]
    angle_rad = data.qpos[jnt_adr]
    angle_deg = np.rad2deg(angle_rad)
    
    springref = model.qpos_spring[jnt_adr]
    stiffness = model.jnt_stiffness[jid]
    deflection = angle_rad - springref
    spring_force = stiffness * deflection
    
    print(f"\n{name}:")
    print(f"  Angle: {angle_rad:.4f} rad = {angle_deg:.1f}°")
    print(f"  Spring deflection: {deflection:.4f} rad = {np.rad2deg(deflection):.1f}°")
    print(f"  Spring force: {spring_force:.6f} N·m")

print("\n" + "=" * 80)
print("TENDON MOMENT ARMS (at current position)")
print("=" * 80)

# Get moment arms (tendon Jacobian)
print(f"\nTendon moment arms (how much joint motion affects tendon length):")
for name, jid in joints:
    # The moment arm is in data.ten_J (tendon Jacobian)
    # ten_J[tendon_id, qvel_adr] gives moment arm
    qvel_adr = model.jnt_dofadr[jid]
    moment_arm = data.ten_J[tendon_id, qvel_adr]
    print(f"  {name}: {moment_arm:.6f} m/rad")

print("\n" + "=" * 80)
print("ANALYSIS COMPLETE")
print("=" * 80)
