#!/usr/bin/env python3
"""
Test gradual tendon lengthening from 151mm to full open at 1mm/sec.
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
print("GRADUAL TENDON SHORTENING TEST")
print("="*80)
print("\nStarting at 163mm (open), shortening at 1mm/sec to 100mm (closed)")
print("Expected: Tendon pulls gripper closed as it shortens")
print("="*80 + "\n")

# Start at 163mm (open)
start_length = 0.163  # meters (max path length - open)
end_length = 0.100    # meters (min - closed)
rate = -0.001         # -1mm/sec = -0.001 m/s (shortening)

last_print = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    start_time = data.time
    
    while viewer.is_running():
        step_start = time.time()
        
        # Calculate current tendon length based on time
        elapsed = data.time - start_time
        tendon_length = start_length + (rate * elapsed)
        tendon_length = max(tendon_length, end_length)  # Cap at min (closing)
        
        # Set actuator controls
        data.ctrl[gripper_f1_id] = tendon_length
        data.ctrl[gripper_f2_id] = tendon_length
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Print status every 0.5 seconds
        if data.time - last_print >= 0.5:
            f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
            f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
            f2_palm_angle = np.degrees(data.qpos[f1_palm_id + 2])
            f2_tip_angle = np.degrees(data.qpos[f1_tip_id + 2])
            
            actual_tendon_f1 = data.ten_length[0] * 1000
            actual_tendon_f2 = data.ten_length[1] * 1000
            
            force_f1 = data.actuator_force[gripper_f1_id]
            force_f2 = data.actuator_force[gripper_f2_id]
            
            print(f"[{data.time:5.2f}s] Tendon cmd: {tendon_length*1000:.1f}mm")
            print(f"  F1_palm: {f1_palm_angle:6.1f}° | F1_tip: {f1_tip_angle:6.1f}°")
            print(f"  F2_palm: {f2_palm_angle:6.1f}° | F2_tip: {f2_tip_angle:6.1f}°")
            print(f"  Actual tendon: F1={actual_tendon_f1:.2f}mm, F2={actual_tendon_f2:.2f}mm")
            print(f"  Forces: F1={force_f1:.1f}N, F2={force_f2:.1f}N")
            print()
            
            last_print = data.time
        
        # Stop when we reach min length (fully closed)
        if tendon_length <= end_length:
            print("="*80)
            print("REACHED MINIMUM TENDON LENGTH (100mm) - FULLY CLOSED")
            print("="*80)
            print(f"Final position:")
            print(f"  F1_palm: {np.degrees(data.qpos[f1_palm_id]):.1f}°")
            print(f"  F1_tip:  {np.degrees(data.qpos[f1_tip_id]):.1f}°")
            print(f"  Actual tendon: {data.ten_length[0]*1000:.2f}mm")
            print("="*80)
            break
        
        # Sync with real-time
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
