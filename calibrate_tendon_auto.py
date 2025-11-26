#!/usr/bin/env python3
"""
Calibrate tendon length range - automatic version
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')

f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]

print("=" * 80)
print("TENDON LENGTH CALIBRATION - AUTO MODE")
print("=" * 80)
print("\nStarting in 2 seconds...")
print("=" * 80 + "\n")

# State tracking
calibration_state = {
    'tendon_max': None,
    'tendon_min': None,
    'start_time': 0,
    'target_length': 0.180,  # Start very long
    'pull_rate': -0.002,  # Retract at 2 mm/s (faster)
    'initial_palm_angle': None,
    'fingers_closing': False,
    'fingers_met': False,
    'phase': 'settling'  # settling, pulling, done
}

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Initial position - very open
    data.ctrl[gripper_f1_id] = 0.180
    data.ctrl[gripper_f2_id] = 0.180
    
    # Settle for 2 seconds
    print("Phase 1: Settling at open position...")
    for _ in range(2000):
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Store initial state
    calibration_state['initial_palm_angle'] = data.qpos[f1_palm_adr]
    calibration_state['start_time'] = data.time
    calibration_state['phase'] = 'pulling'
    
    print(f"Phase 2: Pulling tendon from {data.ten_length[f1_tendon]:.6f}m")
    print(f"Initial palm angle: {np.rad2deg(calibration_state['initial_palm_angle']):.2f}°\n")
    
    step_count = 0
    while viewer.is_running() and calibration_state['phase'] != 'done':
        # Update target length
        elapsed = data.time - calibration_state['start_time']
        calibration_state['target_length'] = 0.180 + calibration_state['pull_rate'] * elapsed
        data.ctrl[gripper_f1_id] = calibration_state['target_length']
        data.ctrl[gripper_f2_id] = calibration_state['target_length']
        
        # Get current state
        palm_angle = data.qpos[f1_palm_adr]
        l1l2_angle = data.qpos[f1_l1l2_adr]
        actual_tendon = data.ten_length[f1_tendon]
        force = abs(data.actuator_force[gripper_f1_id])
        
        # Detect when fingers start closing
        if not calibration_state['fingers_closing']:
            angle_change = abs(np.rad2deg(palm_angle - calibration_state['initial_palm_angle']))
            if angle_change > 1.0:
                calibration_state['fingers_closing'] = True
                calibration_state['tendon_max'] = actual_tendon
                print(f"\n{'='*80}")
                print(f"✓ FINGERS STARTED CLOSING at t={data.time:.2f}s")
                print(f"  Tendon MAX: {calibration_state['tendon_max']:.6f} m ({calibration_state['tendon_max']*1000:.2f} mm)")
                print(f"  Palm angle: {np.rad2deg(palm_angle):.2f}°")
                print(f"{'='*80}\n")
        
        # Detect when fingers meet
        if calibration_state['fingers_closing'] and not calibration_state['fingers_met']:
            # Check for high force or very closed angle
            if force > 40 or palm_angle < np.deg2rad(-85):
                calibration_state['fingers_met'] = True
                calibration_state['tendon_min'] = actual_tendon
                print(f"\n{'='*80}")
                print(f"✓ FINGERS MET at t={data.time:.2f}s")
                print(f"  Tendon MIN: {calibration_state['tendon_min']:.6f} m ({calibration_state['tendon_min']*1000:.2f} mm)")
                print(f"  Palm angle: {np.rad2deg(palm_angle):.2f}°")
                print(f"  Force: {force:.1f} N")
                print(f"{'='*80}\n")
                
                print(f"\n{'='*80}")
                print("CALIBRATION COMPLETE!")
                print(f"{'='*80}")
                print(f"  Tendon MAX (open):   {calibration_state['tendon_max']:.6f} m ({calibration_state['tendon_max']*1000:.2f} mm)")
                print(f"  Tendon MIN (closed): {calibration_state['tendon_min']:.6f} m ({calibration_state['tendon_min']*1000:.2f} mm)")
                print(f"  Total travel:        {(calibration_state['tendon_max'] - calibration_state['tendon_min'])*1000:.2f} mm")
                print(f"\n  Recommended ctrlrange: \"{calibration_state['tendon_min']:.3f} {calibration_state['tendon_max']:.3f}\"")
                print(f"{'='*80}\n")
                
                calibration_state['phase'] = 'done'
        
        # Print status every 200 steps
        if step_count % 200 == 0:
            print(f"t={data.time:5.2f}s  Tendon={actual_tendon:.6f}m  Palm={np.rad2deg(palm_angle):6.1f}°  L1L2={np.rad2deg(l1l2_angle):5.1f}°  Force={force:5.1f}N")
        
        step_count += 1
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Safety: stop if tendon gets too short
        if calibration_state['target_length'] < 0.120:
            print("\n⚠ Safety limit reached (tendon < 0.120m)")
            break

print("\nDone!")
