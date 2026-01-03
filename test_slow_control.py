#!/usr/bin/env python3
"""
Test slow tendon control at 1mm/sec with damping=1.0.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("SLOW TENDON CONTROL TEST (1mm/sec, damping=1.0)")
print("="*80)

# Get actuator IDs
f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

print(f"\nControl range: 0.100 to 0.200 (100mm total range)")
print(f"Target velocity: 1mm/sec = 0.001/sec")
print(f"Expected time for full stroke: 100 seconds")

# Start at neutral position
current_target = 0.150
data.ctrl[f1_actuator_id] = current_target
data.ctrl[f2_actuator_id] = current_target

# Test slow opening and closing
test_sequence = [
    (0.200, "Slow open (0.150 -> 0.200)", 50),   # 50mm = 50 seconds
    (0.100, "Slow close (0.200 -> 0.100)", 100), # 100mm = 100 seconds  
    (0.150, "Return to neutral (0.100 -> 0.150)", 50), # 50mm = 50 seconds
]

for target_length, description, duration_sec in test_sequence:
    print(f"\n" + "="*60)
    print(f"TEST: {description}")
    print(f"Target: {current_target:.3f} -> {target_length:.3f}")
    print(f"Duration: {duration_sec} seconds @ 1mm/sec")
    print("="*60)
    
    steps = int(duration_sec * 100)  # 100 Hz timestep
    step_size = (target_length - current_target) / steps
    
    for i in range(steps):
        # Gradually change target for smooth motion
        new_target = current_target + step_size * i
        data.ctrl[f1_actuator_id] = new_target
        data.ctrl[f2_actuator_id] = new_target
        
        # Run simulation
        mujoco.mj_step(model, data)
        
        # Print status every 10% of the motion
        if i % (steps // 10) == 0:
            f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
            f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
            f2_palm_angle = np.degrees(data.qpos[f2_palm_id])
            f2_tip_angle = np.degrees(data.qpos[f2_tip_id])
            
            # Get tendon lengths
            f1_tendon_length = data.ten_length[0]
            f2_tendon_length = data.ten_length[1]
            
            # Get spring torques
            f1_palm_torque = data.qfrc_passive[f1_palm_id]
            
            progress = (i / steps) * 100
            print(f"  [{progress:5.1f}%] Target={new_target:.3f} | "
                  f"F1: Palm={f1_palm_angle:6.1f}°, Tip={f1_tip_angle:6.1f}° | "
                  f"Tendon={f1_tendon_length:.3f} | "
                  f"Spring={f1_palm_torque:.4f} N·m")
    
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
    
    current_target = target_length

print("\n" + "="*80)
print("SLOW CONTROL TEST COMPLETE")
print("="*80)
