#!/usr/bin/env python3
"""
Test force application after finger contact.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FORCE AFTER CONTACT TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

print("\nTesting force application after finger contact:")

# Test sequence: close to contact, then apply more force
test_sequence = [
    (0.140, "Initial contact"),
    (0.130, "Apply more force"),
    (0.120, "Apply maximum force"),
    (0.110, "Overdrive force"),
]

for target, description in test_sequence:
    # Reset
    mujoco.mj_resetData(model, data)
    data.ctrl[f1_actuator_id] = target
    data.ctrl[f2_actuator_id] = target
    
    # Run to steady state
    for i in range(400):
        mujoco.mj_step(model, data)
    
    # Get final positions
    f1_palm = np.degrees(data.qpos[f1_palm_id])
    f1_tip = np.degrees(data.qpos[f1_tip_id])
    f2_palm = np.degrees(data.qpos[f2_palm_id])
    f2_tip = np.degrees(data.qpos[f2_tip_id])
    
    # Get tendon lengths and spring torques
    f1_tendon = data.ten_length[0]
    f1_spring = data.qfrc_passive[f1_palm_id]
    
    # Check contacts and forces
    contacts = data.ncon
    finger_contacts = 0
    total_force = 0
    
    if contacts > 0:
        for i in range(contacts):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            # Get contact force
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            force_magnitude = np.linalg.norm(force[:3])
            
            if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                finger_contacts += 1
                total_force += force_magnitude
    
    print(f"\n{description} (Target {target:.3f}):")
    print(f"  F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
    print(f"  F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")
    print(f"  Tendon: {f1_tendon:.6f}")
    print(f"  Spring torque: {f1_spring:.4f} N·m")
    print(f"  Contacts: {contacts} total, {finger_contacts} finger-to-finger")
    print(f"  Contact force: {total_force:.3f} N")
    
    # Check behavior
    if finger_contacts > 0:
        if total_force > 1.0:
            print(f"  Status: ✓ STRONG GRIP (force > 1N)")
        elif total_force > 0.1:
            print(f"  Status: ✓ LIGHT GRIP (force > 0.1N)")
        else:
            print(f"  Status: ⚠ WEAK CONTACT (force < 0.1N)")
    else:
        print(f"  Status: ⚠ NO CONTACT")

print("\n" + "="*80)
print("FORCE AFTER CONTACT TEST COMPLETE")
print("="*80)
