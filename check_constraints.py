#!/usr/bin/env python3
"""
Check what's constraining the gripper from opening.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("CONSTRAINT ANALYSIS")
print("="*80)

# Set tendon to zero force position (151mm)
data.ctrl[:] = 0.151

# Reset and step a few times to settle
mujoco.mj_resetData(model, data)
for _ in range(1000):
    mujoco.mj_step(model, data)

print(f"\nCurrent Position:")
print(f"  F1_palm: {np.degrees(data.qpos[0]):.1f}° | F1_tip: {np.degrees(data.qpos[1]):.1f}°")
print(f"  F2_palm: {np.degrees(data.qpos[2]):.1f}° | F2_tip: {np.degrees(data.qpos[3]):.1f}°")

print(f"\nSpring Torques:")
for i in range(4):
    torque = data.qfrc_passive[i]
    print(f"  Joint {i}: {torque:.4f} N·m")

print(f"\nTendon Forces:")
force_f1 = data.actuator_force[0]
force_f2 = data.actuator_force[1]
print(f"  F1: {force_f1:.1f}N | F2: {force_f2:.1f}N")

print(f"\nActive Constraints:")
print(f"  Number of active constraints: {data.nefc}")
print(f"  Constraint forces (first 10):")
for i in range(min(10, data.nefc)):
    print(f"    [{i}]: {data.efc_force[i]:.1f}N")

print(f"\nContact Forces:")
print(f"  Number of contacts: {data.ncon}")
print(f"  Contact details (first 5):")
for i in range(min(5, data.ncon)):
    contact = data.contact[i]
    print(f"    Contact {i}: geom1={contact.geom1}, geom2={contact.geom2}, dist={contact.dist:.4f}")

print(f"\nJoint Limits:")
for i in range(4):
    lower = model.jnt_range[i, 0]
    upper = model.jnt_range[i, 1]
    current = data.qpos[i]
    print(f"  Joint {i}: {np.degrees(lower):.1f}° ≤ {np.degrees(current):.1f}° ≤ {np.degrees(upper):.1f}°")

print(f"\nJoint Velocities:")
for i in range(4):
    vel = data.qvel[i]
    print(f"  Joint {i}: {vel:.4f} rad/s")

print("\n" + "="*80)
print("DIAGNOSIS:")
print("="*80)
print("If springs have torque but gripper doesn't move:")
print("1. Check for active constraints (should be few)")
print("2. Check for contacts (should be none in open position)")
print("3. Check if joints are at limits (should not be)")
print("4. Check if velocities are zero (indicates equilibrium)")
print("="*80)
