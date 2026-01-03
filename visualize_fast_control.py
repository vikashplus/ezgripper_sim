#!/usr/bin/env python3
"""
Visualize faster tendon control to see complete opening/closing cycle.
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
print("FAST TENDON CONTROL VISUALIZATION (10mm/sec, damping=1.0)")
print("="*80)
print("The gripper will automatically cycle through:")
print("  1. Open (0.150 -> 0.200) - 5 seconds")
print("  2. Close (0.200 -> 0.100) - 10 seconds") 
print("  3. Neutral (0.100 -> 0.150) - 5 seconds")
print("Total cycle: 20 seconds")
print("="*80)

# Get actuator IDs
f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

# Control variables
current_target = 0.150
phase = 0  # 0=open, 1=close, 2=neutral
phase_start_time = 0
phase_duration = [5, 10, 5]  # seconds for each phase
phase_targets = [0.200, 0.100, 0.150]  # target for each phase

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.1, 0, 0.1])

    print("Viewer launched. Close window to exit.")
    
    while viewer.is_running():
        # Calculate phase progress
        phase_progress = (data.time - phase_start_time) / phase_duration[phase]
        
        # Check if phase is complete
        if phase_progress >= 1.0:
            phase = (phase + 1) % 3
            phase_start_time = data.time
            phase_progress = 0.0
            phase_names = ["OPENING", "CLOSING", "NEUTRAL"]
            print(f"\n[{data.time:6.1f}s] Starting {phase_names[phase]}: "
                  f"Target {current_target:.3f} -> {phase_targets[phase]:.3f}")
        
        # Calculate current target based on phase progress
        start_target = current_target if phase == 0 else phase_targets[phase - 1]
        end_target = phase_targets[phase]
        current_target = start_target + (end_target - start_target) * phase_progress
        
        # Apply control
        data.ctrl[f1_actuator_id] = current_target
        data.ctrl[f2_actuator_id] = current_target
        
        # Get current state
        f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
        f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
        f2_palm_angle = np.degrees(data.qpos[f2_palm_id])
        f2_tip_angle = np.degrees(data.qpos[f2_tip_id])
        
        f1_tendon_length = data.ten_length[0]
        f1_spring_torque = data.qfrc_passive[f1_palm_id]
        
        # Print status every 1 second
        if int(data.time) % 1 == 0 and data.time - int(data.time) < model.opt.timestep:
            phase_names = ["OPENING", "CLOSING", "NEUTRAL"]
            print(f"[{data.time:6.1f}s] {phase_names[phase]} {phase_progress*100:5.1f}% | "
                  f"Target={current_target:.3f} | "
                  f"F1: Palm={f1_palm_angle:6.1f}°, Tip={f1_tip_angle:6.1f}° | "
                  f"Tendon={f1_tendon_length:.3f} | "
                  f"Spring={f1_spring_torque:6.3f} N·m | "
                  f"Contacts: {data.ncon}")
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()

print("\nVisualization ended.")
