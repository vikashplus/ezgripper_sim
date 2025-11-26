#!/usr/bin/env python3
"""
Monitor gripper closing and detect when joints hit limits
"""
import mujoco
import mujoco.viewer
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get IDs
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
f1_palm_l1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1_l2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_palm_l1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_l1_l2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

# Get qpos addresses
f1_palm_adr = model.jnt_qposadr[f1_palm_l1_id]
f1_l1l2_adr = model.jnt_qposadr[f1_l1_l2_id]
f2_palm_adr = model.jnt_qposadr[f2_palm_l1_id]
f2_l1l2_adr = model.jnt_qposadr[f2_l1_l2_id]

# Get joint limits
f1_palm_range = model.jnt_range[f1_palm_l1_id]
f1_l1l2_range = model.jnt_range[f1_l1_l2_id]

print("=" * 80)
print("GRIPPER CLOSING MONITOR")
print("=" * 80)
print(f"\nF1_palm_knuckle range: [{f1_palm_range[0]:.3f}, {f1_palm_range[1]:.3f}] rad")
print(f"                      = [{np.rad2deg(f1_palm_range[0]):.1f}°, {np.rad2deg(f1_palm_range[1]):.1f}°]")
print(f"\nF1_knuckle_tip range:  [{f1_l1l2_range[0]:.3f}, {f1_l1l2_range[1]:.3f}] rad")
print(f"                      = [{np.rad2deg(f1_l1l2_range[0]):.1f}°, {np.rad2deg(f1_l1l2_range[1]):.1f}°]")
print("\n" + "=" * 80)
print("Starting simulation... Watch for joint limit violations")
print("=" * 80)

# Simulation parameters
phase = "opening"
phase_time = 0
close_start_time = 1.0
close_duration = 3.0
hold_duration = 2.0
open_duration = 2.0

def check_limits(time):
    """Check if joints are at limits"""
    f1_palm_angle = data.qpos[f1_palm_adr]
    f1_l1l2_angle = data.qpos[f1_l1l2_adr]
    f2_palm_angle = data.qpos[f2_palm_adr]
    f2_l1l2_angle = data.qpos[f2_l1l2_adr]
    
    # Check if at limits (within 1 degree tolerance)
    tolerance = np.deg2rad(1.0)
    
    at_limit = False
    if abs(f1_palm_angle - f1_palm_range[0]) < tolerance:
        print(f"[{time:.2f}s] F1_palm_knuckle at MIN limit: {np.rad2deg(f1_palm_angle):.1f}°")
        at_limit = True
    if abs(f1_palm_angle - f1_palm_range[1]) < tolerance:
        print(f"[{time:.2f}s] F1_palm_knuckle at MAX limit: {np.rad2deg(f1_palm_angle):.1f}°")
        at_limit = True
    if abs(f1_l1l2_angle - f1_l1l2_range[0]) < tolerance:
        print(f"[{time:.2f}s] F1_knuckle_tip at MIN limit: {np.rad2deg(f1_l1l2_angle):.1f}°")
        at_limit = True
    if abs(f1_l1l2_angle - f1_l1l2_range[1]) < tolerance:
        print(f"[{time:.2f}s] F1_knuckle_tip at MAX limit: {np.rad2deg(f1_l1l2_angle):.1f}°")
        at_limit = True
    
    # Check asymmetry
    palm_diff = abs(f1_palm_angle - f2_palm_angle)
    l1l2_diff = abs(f1_l1l2_angle - f2_l1l2_angle)
    
    if palm_diff > np.deg2rad(5.0):
        print(f"[{time:.2f}s] ASYMMETRY in Palm-L1: F1={np.rad2deg(f1_palm_angle):.1f}°, F2={np.rad2deg(f2_palm_angle):.1f}°, diff={np.rad2deg(palm_diff):.1f}°")
    if l1l2_diff > np.deg2rad(5.0):
        print(f"[{time:.2f}s] ASYMMETRY in L1-L2: F1={np.rad2deg(f1_l1l2_angle):.1f}°, F2={np.rad2deg(f2_l1l2_angle):.1f}°, diff={np.rad2deg(l1l2_diff):.1f}°")

def update_control(time):
    global phase, phase_time
    
    if phase == "opening" and time >= close_start_time:
        phase = "closing"
        phase_time = time
        print(f"\n[{time:.2f}s] === CLOSING PHASE ===")
    
    elif phase == "closing" and time >= phase_time + close_duration:
        phase = "holding"
        phase_time = time
        print(f"\n[{time:.2f}s] === HOLDING PHASE ===")
    
    elif phase == "holding" and time >= phase_time + hold_duration:
        phase = "opening_final"
        phase_time = time
        print(f"\n[{time:.2f}s] === OPENING PHASE ===")
    
    # Set control
    if phase == "opening":
        data.ctrl[gripper_actuator_id] = 0.0
    elif phase == "closing":
        progress = (time - phase_time) / close_duration
        data.ctrl[gripper_actuator_id] = -1.0 * progress
    elif phase == "holding":
        data.ctrl[gripper_actuator_id] = -1.0
    elif phase == "opening_final":
        progress = (time - phase_time) / open_duration
        data.ctrl[gripper_actuator_id] = -1.0 * (1.0 - progress)

# Run with viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0.1, 0, 0.1]
    
    last_check_time = 0
    check_interval = 0.1  # Check every 100ms
    
    while viewer.is_running():
        step_start = data.time
        
        # Update control
        update_control(data.time)
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Check limits periodically
        if data.time - last_check_time >= check_interval:
            check_limits(data.time)
            last_check_time = data.time
        
        # Sync viewer
        viewer.sync()
        
        # Stop after full cycle
        if phase == "opening_final" and data.time >= phase_time + open_duration:
            print(f"\n[{data.time:.2f}s] === SIMULATION COMPLETE ===")
            break

print("\n" + "=" * 80)
print("Monitor complete")
print("=" * 80)
