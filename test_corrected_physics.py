#!/usr/bin/env python3
"""
Test corrected gripper physics with proper force directions.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("CORRECTED GRIPPER PHYSICS TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

print(f"\nTesting with corrected force directions:")
print(f"Positive force = CLOSE, Negative force = OPEN")

# Start from neutral position
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.0
data.ctrl[f2_actuator_id] = 0.0

# Open the gripper (negative force)
print(f"\nOpening gripper with -1.5N force:")
for step in range(200):
    data.ctrl[f1_actuator_id] = -1.5  # Open
    data.ctrl[f2_actuator_id] = -1.5
    mujoco.mj_step(model, data)
    
    if step % 50 == 0:
        f1_palm = np.degrees(data.qpos[f1_palm_id])
        f1_tip = np.degrees(data.qpos[f1_tip_id])
        f2_palm = np.degrees(data.qpos[f2_palm_id])
        f2_tip = np.degrees(data.qpos[f2_tip_id])
        
        print(f"  Step {step:3d}: F1({f1_palm:5.1f}°,{f1_tip:5.1f}°) F2({f2_palm:5.1f}°,{f2_tip:5.1f}°) Contacts={data.ncon}")

# Close the gripper (positive force)
print(f"\nClosing gripper with +1.5N force:")
contact_detected = False
for step in range(400):
    data.ctrl[f1_actuator_id] = 1.5  # Close
    data.ctrl[f2_actuator_id] = 1.5
    mujoco.mj_step(model, data)
    
    if step % 25 == 0:
        f1_palm = np.degrees(data.qpos[f1_palm_id])
        f1_tip = np.degrees(data.qpos[f1_tip_id])
        f2_palm = np.degrees(data.qpos[f2_palm_id])
        f2_tip = np.degrees(data.qpos[f2_tip_id])
        
        print(f"  Step {step:3d}: F1({f1_palm:5.1f}°,{f1_tip:5.1f}°) F2({f2_palm:5.1f}°,{f2_tip:5.1f}°) Contacts={data.ncon}")
        
        if data.ncon > 0 and not contact_detected:
            contact_detected = True
            print(f"    ✓ CONTACT DETECTED!")
            for i in range(min(data.ncon, 3)):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                force = np.linalg.norm(data.efc_force[contact.efc_address:contact.efc_address+6])
                print(f"      {i}: {geom1_name} <-> {geom2_name}, force={force:.2f}N")

# Test force equilibrium after contact
if contact_detected:
    print(f"\nTesting force equilibrium (continued force application):")
    for step in range(200):
        data.ctrl[f1_actuator_id] = 1.5  # Continue closing force
        data.ctrl[f2_actuator_id] = 1.5
        mujoco.mj_step(model, data)
        
        if step % 25 == 0:
            f1_palm = np.degrees(data.qpos[f1_palm_id])
            f1_tip = np.degrees(data.qpos[f1_tip_id])
            f2_palm = np.degrees(data.qpos[f2_palm_id])
            f2_tip = np.degrees(data.qpos[f2_tip_id])
            
            print(f"  Step {step:3d}: F1({f1_palm:5.1f}°,{f1_tip:5.1f}°) F2({f2_palm:5.1f}°,{f2_tip:5.1f}°) Contacts={data.ncon}")

print("\n" + "="*80)
print("CORRECTED PHYSICS TEST COMPLETE")
print("="*80)
