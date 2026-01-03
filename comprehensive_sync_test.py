#!/usr/bin/env python3
"""
Comprehensive synchronization test across multiple positions.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("COMPREHENSIVE SYNCHRONIZATION TEST")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Test positions
test_positions = [0.100, 0.125, 0.150, 0.175, 0.200]

print(f"\nTesting synchronization across {len(test_positions)} positions:")
print("-" * 60)

for target in test_positions:
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
    
    # Calculate differences
    palm_diff = abs(f1_palm - f2_palm)
    tip_diff = abs(f1_tip - f2_tip)
    
    # Get tendon lengths
    f1_tendon = data.ten_length[0]
    f2_tendon = data.ten_length[1]
    tendon_diff = abs(f1_tendon - f2_tendon)
    
    print(f"Target {target:.3f}:")
    print(f"  F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°, Tendon={f1_tendon:.6f}")
    print(f"  F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°, Tendon={f2_tendon:.6f}")
    print(f"  Diff: Palm={palm_diff:6.3f}°, Tip={tip_diff:6.3f}°, Tendon={tendon_diff:.6f}")
    
    # Check if synchronized (within 0.001 degrees)
    if palm_diff < 0.001 and tip_diff < 0.001 and tendon_diff < 0.000001:
        print(f"  Status: ✓ SYNCHRONIZED")
    else:
        print(f"  Status: ⚠ NOT SYNCHRONIZED")
    print()

print("="*80)
print("SYNCHRONIZATION TEST COMPLETE")
print("="*80)
