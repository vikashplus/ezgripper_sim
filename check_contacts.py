#!/usr/bin/env python3
"""
Check finger contacts to identify asymmetry source.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FINGER CONTACT ANALYSIS")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Test positions where asymmetry occurs
test_positions = [0.100, 0.125, 0.200]

for target in test_positions:
    print(f"\n" + "="*60)
    print(f"ANALYZING TARGET {target:.3f}")
    print("="*60)
    
    # Reset
    mujoco.mj_resetData(model, data)
    data.ctrl[f1_actuator_id] = target
    data.ctrl[f2_actuator_id] = target
    
    # Run to steady state
    for i in range(200):
        mujoco.mj_step(model, data)
    
    # Get final positions
    f1_palm = np.degrees(data.qpos[f1_palm_id])
    f1_tip = np.degrees(data.qpos[f1_tip_id])
    f2_palm = np.degrees(data.qpos[f2_palm_id])
    f2_tip = np.degrees(data.qpos[f2_tip_id])
    
    print(f"Final positions:")
    print(f"  F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
    print(f"  F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")
    print(f"  Diff: Palm={abs(f1_palm-f2_palm):6.3f}°, Tip={abs(f1_tip-f2_tip):6.3f}°")
    
    # Analyze contacts
    print(f"\nContacts: {data.ncon}")
    
    if data.ncon > 0:
        print("Contact details:")
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            # Get contact force
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            force_magnitude = np.linalg.norm(force[:3])
            
            print(f"  Contact {i}: {geom1_name} <-> {geom2_name}")
            print(f"    Force: {force_magnitude:.6f} N")
            print(f"    Distance: {contact.dist:.6f}")
            print(f"    Normal: [{contact.normal[0]:.3f}, {contact.normal[1]:.3f}, {contact.normal[2]:.3f}]")
    else:
        print("No contacts detected")
    
    # Check if fingers are contacting each other
    finger_contacts = []
    if data.ncon > 0:
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                finger_contacts.append((geom1_name, geom2_name))
    
    if finger_contacts:
        print(f"\n⚠ FINGER-TO-FINGER CONTACTS DETECTED:")
        for geom1, geom2 in finger_contacts:
            print(f"  {geom1} <-> {geom2}")
    else:
        print(f"\n✓ No finger-to-finger contacts")

print("\n" + "="*80)
print("CONTACT ANALYSIS COMPLETE")
print("="*80)
