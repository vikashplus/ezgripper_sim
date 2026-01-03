#!/usr/bin/env python3
"""
Diagnostic version of tennis ball demo - prints actual tendon lengths
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("=" * 60)
print("EZGripper Diagnostic Demo")
print("=" * 60)

# Get IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
f1_tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
f2_tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')

# Print actuator configuration
print("\nActuator Configuration:")
print(f"  F1 actuator: kp={model.actuator_gainprm[gripper_f1_id,0]:.1f}, kv={model.actuator_biasprm[gripper_f1_id,1]:.1f}")
print(f"  F2 actuator: kp={model.actuator_gainprm[gripper_f2_id,0]:.1f}, kv={model.actuator_biasprm[gripper_f2_id,1]:.1f}")
print(f"  Force range: [{model.actuator_forcerange[gripper_f1_id,0]:.1f}, {model.actuator_forcerange[gripper_f1_id,1]:.1f}]")

print("\nStarting simulation...")
print("Watch for TARGET vs ACTUAL tendon lengths")
print("=" * 60 + "\n")

phase_info = {"phase": "open", "last_print": 0}

def update_control(model, data, phase_info):
    """Update gripper control and print diagnostics"""
    current_time = data.time
    
    # Print every 1 second
    if current_time - phase_info['last_print'] >= 1.0:
        cycle_time = current_time % 10.0
        
        # Tendon length range: 0.135 (closed) to 0.180 (open - fully extended)
        open_length = 0.180
        closed_length = 0.135
        
        if cycle_time < 5.0:
            progress = cycle_time / 5.0
            target_length = open_length - (open_length - closed_length) * progress
            phase = "CLOSING"
        else:
            progress = (cycle_time - 5.0) / 5.0
            target_length = closed_length + (open_length - closed_length) * progress
            phase = "OPENING"
        
        # Set control
        data.ctrl[gripper_f1_id] = target_length
        data.ctrl[gripper_f2_id] = target_length
        
        # Get actual lengths
        actual_f1 = data.ten_length[f1_tendon_id]
        actual_f2 = data.ten_length[f2_tendon_id]
        
        # Print diagnostic
        print(f"[{current_time:5.1f}s] {phase:8s} | Target: {target_length*1000:6.2f}mm | Actual: F1={actual_f1*1000:6.2f}mm F2={actual_f2*1000:6.2f}mm | Error: {(actual_f1-target_length)*1000:+6.2f}mm")
        
        phase_info['last_print'] = current_time

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    while viewer.is_running():
        step_start = time.time()
        
        update_control(model, data, phase_info)
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nDemo finished!")
