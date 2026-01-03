#!/usr/bin/env python3
"""
Debug why spring torque is constant across all angles.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("SPRING BEHAVIOR DEBUG")
print("="*80)

# Get joint and spring info
f1_palm_id = model.joint('F1_palm_knuckle').id
# MuJoCo joint attributes access
joint_stiffness = model.jnt_stiffness[f1_palm_id]
joint_springref = model.jnt_springref[f1_palm_id] 
joint_damping = model.jnt_damping[f1_palm_id]
joint_range = model.jnt_range[f1_palm_id]

print(f"\nJoint configuration:")
print(f"  Joint ID: {f1_palm_id}")
print(f"  Range: [{np.degrees(joint_range[0]):.1f}°, {np.degrees(joint_range[1]):.1f}°]")
print(f"  Stiffness: {joint_stiffness}")
print(f"  Springref: {np.degrees(joint_springref):.1f}°")
print(f"  Damping: {joint_damping}")

# Check if this is a joint spring or actuator spring
print(f"\nSpring type analysis:")
print(f"  Joint has stiffness: {joint_stiffness > 0}")
print(f"  Joint has springref: {abs(joint_springref) > 0.001}")

# Test torque calculation manually
print(f"\n" + "="*50)
print("MANUAL TORQUE CALCULATION:")
print("="*50)

for angle in [-90, -45, -20, 0, 25]:
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_kinematics(model, data)
    
    # Get actual torque from MuJoCo
    actual_torque = data.qfrc_passive[f1_palm_id]
    
    # Calculate expected torque manually: torque = -stiffness * (angle - springref)
    angle_rad = np.radians(angle)
    springref_rad = joint_springref
    expected_torque = -joint_stiffness * (angle_rad - springref_rad)
    
    print(f"  {angle:3.0f}°:")
    print(f"    Actual torque:   {actual_torque:.6f} N·m")
    print(f"    Expected torque: {expected_torque:.6f} N·m")
    print(f"    Difference:      {abs(actual_torque - expected_torque):.6f} N·m")
    
    if abs(actual_torque - expected_torque) > 0.001:
        print(f"    → MISMATCH!")

# Check all passive forces
print(f"\n" + "="*50)
print("ALL PASSIVE FORCES:")
print("="*50)

data.qpos[f1_palm_id] = np.radians(-20)
mujoco.mj_kinematics(model, data)

print(f"At -20°:")
for i in range(model.nv):
    if abs(data.qfrc_passive[i]) > 0.0001:
        joint_name = model.joint(i).name if i < model.njnt else f"dof_{i}"
        print(f"  {joint_name}: {data.qfrc_passive[i]:.6f}")

# Check if there are any tendon forces affecting this
print(f"\n" + "="*50)
print("TENDON FORCES:")
print("="*50)

if hasattr(data, 'ten_force') and data.ten_force.size > 0:
    for i in range(data.ten_force.size):
        if abs(data.ten_force[i]) > 0.0001:
            tendon_name = model.tendon(i).name if i < model.ntendon else f"tendon_{i}"
            print(f"  {tendon_name}: {data.ten_force[i]:.6f}")
else:
    print("  No tendon forces (tendons disabled)")

print("="*80)
