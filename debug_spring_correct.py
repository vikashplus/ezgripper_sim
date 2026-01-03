#!/usr/bin/env python3
"""
Debug spring behavior with correct MuJoCo API.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("SPRING BEHAVIOR DEBUG - CORRECTED")
print("="*80)

# Get joint and spring info
f1_palm_id = model.joint('F1_palm_knuckle').id

# MuJoCo joint attributes access
joint_stiffness = model.jnt_stiffness[f1_palm_id]
joint_springref = model.qpos_spring[model.joint(f1_palm_id).qposadr[0]] 
joint_range = model.jnt_range[f1_palm_id]
joint = model.joint(f1_palm_id)
joint_damping = joint.damping[0]

print(f"\nJoint configuration:")
print(f"  Joint ID: {f1_palm_id}")
print(f"  Range: [{np.degrees(joint_range[0]):.1f}°, {np.degrees(joint_range[1]):.1f}°]")
print(f"  Stiffness: {joint_stiffness}")
print(f"  Springref: {np.degrees(joint_springref):.1f}°")
print(f"  Damping: {joint_damping}")

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
    expected_torque = -joint_stiffness * (angle_rad - joint_springref)
    
    print(f"  {angle:3.0f}°:")
    print(f"    Angle (rad):       {angle_rad:.6f}")
    print(f"    Springref (rad):   {joint_springref:.6f}")
    print(f"    Difference (rad): {angle_rad - joint_springref:.6f}")
    print(f"    Actual torque:     {actual_torque:.6f} N·m")
    print(f"    Expected torque:   {expected_torque:.6f} N·m")
    print(f"    Difference:        {abs(actual_torque - expected_torque):.6f} N·m")
    
    if abs(actual_torque - expected_torque) > 0.001:
        print(f"    → MISMATCH!")
    else:
        print(f"    → MATCH!")

# Check if the spring is being overridden by something
print(f"\n" + "="*50)
print("PASSIVE FORCES BREAKDOWN:")
print("="*50)

data.qpos[f1_palm_id] = np.radians(-20)
mujoco.mj_kinematics(model, data)

# Check individual force contributions
print(f"At -20°:")
print(f"  Total passive force: {data.qfrc_passive[f1_palm_id]:.6f}")

# Check if there are any constraint forces
print(f"  Constraint force: {data.qfrc_constraint[f1_palm_id]:.6f}")

# Check if there are any actuator forces  
print(f"  Actuator force: {data.qfrc_actuator[f1_palm_id]:.6f}")

print("="*80)
