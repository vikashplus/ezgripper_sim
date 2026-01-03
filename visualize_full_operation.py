#!/usr/bin/env python3
"""
Visualize full gripper operation from open to close with finger contact.
"""

import mujoco
import mujoco.viewer
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FULL GRIPPER OPERATION VISUALIZATION")
print("="*80)
print("The gripper will demonstrate:")
print("  1. Open position (0.200) - 3 seconds")
print("  2. Progressive closing (0.200 -> 0.100) - 7 seconds")
print("  3. Finger contact and force application - 5 seconds")
print("  4. Return to neutral (0.100 -> 0.150) - 4 seconds")
print("Total cycle: 19 seconds")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Control sequence (force commands)
sequence = [
    (-150.0, 3.0, "OPEN POSITION"),
    (100.0, 7.0, "CLOSING TO CONTACT"),
    (40.0, 5.0, "APPLYING FORCE"),
    (-50.0, 4.0, "RETURN TO NEUTRAL")
]

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.35
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -25
    viewer.cam.lookat = np.array([0.1, 0, 0.05])

    print("Viewer launched. Close window to exit.")
    
    phase_start_time = 0
    current_phase = 0
    start_target = 0.200
    
    while viewer.is_running():
        # Check phase transitions
        elapsed = data.time - phase_start_time
        if elapsed >= sequence[current_phase][1] and current_phase < len(sequence) - 1:
            current_phase += 1
            phase_start_time = data.time
            start_target = sequence[current_phase-1][0]
            print(f"\n[{data.time:6.1f}s] {sequence[current_phase][2]}")
        
        # Calculate current force
        if current_phase == 0:
            force = sequence[0][0]  # Opening force
        else:
            # Interpolate between phases
            phase_duration = sequence[current_phase][1]
            phase_progress = min(elapsed / phase_duration, 1.0)
            start_force = sequence[current_phase-1][0] if current_phase > 0 else sequence[0][0]
            end_force = sequence[current_phase][0]
            force = start_force + (end_force - start_force) * phase_progress
        
        # Apply force control
        data.ctrl[f1_actuator_id] = force
        data.ctrl[f2_actuator_id] = force
        
        # Get current state
        f1_palm = np.degrees(data.qpos[f1_palm_id])
        f1_tip = np.degrees(data.qpos[f1_tip_id])
        f2_palm = np.degrees(data.qpos[f2_palm_id])
        f2_tip = np.degrees(data.qpos[f2_tip_id])
        
        f1_tendon = data.ten_length[0]
        f1_spring = data.qfrc_passive[f1_palm_id]
        
        # Print status every second
        if int(data.time) % 1 == 0 and data.time - int(data.time) < model.opt.timestep:
            phase_name = sequence[current_phase][2]
            contacts = data.ncon
            
            # Check finger contacts
            finger_contacts = 0
            if contacts > 0:
                for i in range(contacts):
                    contact = data.contact[i]
                    geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                    geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                    
                    if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                        finger_contacts += 1
            
            contact_status = f"CONTACT: {finger_contacts}" if finger_contacts > 0 else "NO CONTACT"
            
            print(f"[{data.time:6.1f}s] {phase_name} | "
                  f"Force={force:.1f}N | "
                  f"F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}° | "
                  f"F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}° | "
                  f"Tendon={f1_tendon:.3f} | "
                  f"Spring={f1_spring:6.3f} N·m | "
                  f"{contact_status}")
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()

print("\nVisualization ended.")
