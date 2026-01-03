#!/usr/bin/env python3
"""
Test gripper position with zero tendon force.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get actuator and joint IDs
gripper_f1_id = model.actuator('gripper_actuator_f1').id
gripper_f2_id = model.actuator('gripper_actuator_f2').id
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id

print("="*80)
print("ZERO TENDON FORCE TEST")
print("="*80)
print("\nTesting different tendon lengths to find zero force equilibrium")
print("Expected: Springs push gripper open when tendon force = 0")
print("="*80 + "\n")

# Test different tendon lengths
test_lengths = [0.100, 0.120, 0.140, 0.151, 0.160, 0.163, 0.170, 0.180]

for test_length in test_lengths:
    print(f"\nTesting tendon length: {test_length*1000:.1f}mm")
    print("-" * 50)
    
    # Reset simulation
    mujoco.mj_resetData(model, data)
    
    # Set tendon command
    data.ctrl[gripper_f1_id] = test_length
    data.ctrl[gripper_f2_id] = test_length
    
    # Run for 2 seconds to settle
    for _ in range(2000):
        mujoco.mj_step(model, data)
    
    # Check final state
    f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
    f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
    f2_palm_angle = np.degrees(data.qpos[f1_palm_id + 2])
    f2_tip_angle = np.degrees(data.qpos[f1_tip_id + 2])
    
    actual_tendon_f1 = data.ten_length[0] * 1000
    actual_tendon_f2 = data.ten_length[1] * 1000
    
    force_f1 = data.actuator_force[gripper_f1_id]
    force_f2 = data.actuator_force[gripper_f2_id]
    
    print(f"  Final position:")
    print(f"    F1_palm: {f1_palm_angle:6.1f}° | F1_tip: {f1_tip_angle:6.1f}°")
    print(f"    F2_palm: {f2_palm_angle:6.1f}° | F2_tip: {f2_tip_angle:6.1f}°")
    print(f"  Actual tendon: F1={actual_tendon_f1:.2f}mm, F2={actual_tendon_f2:.2f}mm")
    print(f"  Forces: F1={force_f1:.1f}N, F2={force_f2:.1f}N")
    
    # Check if force is near zero
    if abs(force_f1) < 1.0 and abs(force_f2) < 1.0:
        print(f"  >>> ZERO FORCE EQUILIBRIUM FOUND at {test_length*1000:.1f}mm <<<")
        break

print("\n" + "="*80)
print("CONCLUSION:")
print("The tendon force should be zero when the commanded length")
print("matches the actual physical path length at that position.")
print("At zero force, springs should push gripper to fully open position.")
print("="*80)
