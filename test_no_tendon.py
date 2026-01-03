#!/usr/bin/env python3
"""
Test gripper behavior with tendon actuators disabled (control = 0).
This eliminates tendon force to see pure spring behavior.
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

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id

print("="*80)
print("NO TENDON FORCE TEST")
print("="*80)
print("\nTendon actuators DISABLED (control = 0)")
print("Expected: Springs should push gripper to natural open position")
print("="*80 + "\n")

# Disable tendon actuators completely
data.ctrl[:] = 0.0

last_print = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    while viewer.is_running():
        step_start = time.time()
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Print status every 0.5 seconds
        if data.time - last_print >= 0.5:
            f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
            f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
            f2_palm_angle = np.degrees(data.qpos[f1_palm_id + 2])
            f2_tip_angle = np.degrees(data.qpos[f1_tip_id + 2])
            
            # Check tendon forces (should be 0)
            force_f1 = data.actuator_force[0]
            force_f2 = data.actuator_force[1]
            
            # Check spring torques
            torque_f1_palm = data.qfrc_passive[f1_palm_id]
            torque_f1_tip = data.qfrc_passive[f1_tip_id]
            
            print(f"[{data.time:5.2f}s] NO TENDON FORCE")
            print(f"  F1_palm: {f1_palm_angle:6.1f}° | F1_tip: {f1_tip_angle:6.1f}°")
            print(f"  F2_palm: {f2_palm_angle:6.1f}° | F2_tip: {f2_tip_angle:6.1f}°")
            print(f"  Spring torques: Palm={torque_f1_palm:.4f} N·m, Tip={torque_f1_tip:.4f} N·m")
            print(f"  Tendon forces: F1={force_f1:.1f}N, F2={force_f2:.1f}N")
            
            # Check contacts
            print(f"  Contacts: {data.ncon}")
            if data.ncon > 0:
                print(f"  First contact: geom1={data.contact[0].geom1}, geom2={data.contact[0].geom2}")
            print()
            
            last_print = data.time
        
        # Stop after 10 seconds
        if data.time >= 10.0:
            print("="*80)
            print("FINAL POSITION (no tendon force):")
            print(f"  F1_palm: {np.degrees(data.qpos[f1_palm_id]):.1f}°")
            print(f"  F1_tip:  {np.degrees(data.qpos[f1_tip_id]):.1f}°")
            print(f"  Contacts: {data.ncon}")
            print("="*80)
            break
        
        # Sync with real-time
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
