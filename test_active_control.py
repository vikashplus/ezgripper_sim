#!/usr/bin/env python3
"""
Test active tendon control of the gripper.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("ACTIVE TENDON CONTROL TEST")
print("="*80)

# Get actuator IDs
f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

print(f"\nActuator IDs: F1={f1_actuator_id}, F2={f2_actuator_id}")
print(f"Control range: 0.100 to 0.200")

# Test sequence: open -> close -> open
test_sequence = [
    (0.150, "Neutral (middle)"),
    (0.200, "Open (max length)"),
    (0.100, "Close (min length)"),
    (0.200, "Open (max length)"),
    (0.150, "Neutral (middle)"),
]

for target_length, description in test_sequence:
    print(f"\n" + "="*50)
    print(f"TEST: {description}")
    print(f"Target tendon length: {target_length}")
    print("="*50)
    
    # Set actuator controls
    data.ctrl[f1_actuator_id] = target_length
    data.ctrl[f2_actuator_id] = target_length
    
    # Run simulation for 2 seconds
    for i in range(200):
        mujoco.mj_step(model, data)
        
        # Print status every 50 steps
        if i % 50 == 0:
            f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
            f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
            f2_palm_angle = np.degrees(data.qpos[f2_palm_id])
            f2_tip_angle = np.degrees(data.qpos[f2_tip_id])
            
            # Get tendon lengths
            f1_tendon_length = data.ten_length[0]  # finger1_tendon
            f2_tendon_length = data.ten_length[1]  # finger2_tendon
            
            # Get spring torques
            f1_palm_torque = data.qfrc_passive[f1_palm_id]
            f1_tip_torque = data.qfrc_passive[f1_tip_id]
            
            print(f"  [{i:3d}] F1: Palm={f1_palm_angle:6.1f}°, Tip={f1_tip_angle:6.1f}° | "
                  f"F2: Palm={f2_palm_angle:6.1f}°, Tip={f2_tip_angle:6.1f}°")
            print(f"        Tendon: F1={f1_tendon_length:.3f}, F2={f2_tendon_length:.3f} | "
                  f"Spring torque: Palm={f1_palm_torque:.4f} N·m")
    
    # Final status
    f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
    f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
    f2_palm_angle = np.degrees(data.qpos[f2_palm_id])
    f2_tip_angle = np.degrees(data.qpos[f2_tip_id])
    
    f1_tendon_length = data.ten_length[0]
    f2_tendon_length = data.ten_length[1]
    
    print(f"  Final: F1 Palm={f1_palm_angle:6.1f}°, Tip={f1_tip_angle:6.1f}° | "
          f"F2 Palm={f2_palm_angle:6.1f}°, Tip={f2_tip_angle:6.1f}°")
    print(f"  Final tendon: F1={f1_tendon_length:.3f}, F2={f2_tendon_length:.3f}")
    print(f"  Contacts: {data.ncon}")

print("\n" + "="*80)
print("ACTIVE CONTROL TEST COMPLETE")
print("="*80)
