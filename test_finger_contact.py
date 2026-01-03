#!/usr/bin/env python3
"""
Test if fingers can touch at their tips with extended joint range.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FINGER CONTACT TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Test closing progressively
print("\nTesting progressive closing to find finger contact:")

for target in [0.180, 0.160, 0.140, 0.120, 0.110, 0.105, 0.100]:
    # Reset
    mujoco.mj_resetData(model, data)
    data.ctrl[f1_actuator_id] = target
    data.ctrl[f2_actuator_id] = target
    
    # Run to steady state
    for i in range(300):
        mujoco.mj_step(model, data)
    
    # Get final positions
    f1_palm = np.degrees(data.qpos[f1_palm_id])
    f1_tip = np.degrees(data.qpos[f1_tip_id])
    f2_palm = np.degrees(data.qpos[f2_palm_id])
    f2_tip = np.degrees(data.qpos[f2_tip_id])
    
    # Get tendon lengths
    f1_tendon = data.ten_length[0]
    f2_tendon = data.ten_length[1]
    
    # Check contacts
    contacts = data.ncon
    finger_contacts = 0
    
    if contacts > 0:
        for i in range(contacts):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                finger_contacts += 1
    
    print(f"Target {target:.3f}:")
    print(f"  F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°")
    print(f"  F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°")
    print(f"  Tendon: F1={f1_tendon:.6f}, F2={f2_tendon:.6f}")
    print(f"  Contacts: {contacts} total, {finger_contacts} finger-to-finger")
    
    if finger_contacts > 0:
        print(f"  *** FINGERS ARE TOUCHING! ***")
        break
    elif f1_palm >= 45.0 or f2_palm >= 45.0:
        print(f"  *** FINGERS AT JOINT LIMIT (~25°) ***")
        break

print("\n" + "="*80)
print("FINGER CONTACT TEST COMPLETE")
print("="*80)
