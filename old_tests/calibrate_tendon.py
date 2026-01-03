#!/usr/bin/env python3
"""
Calibrate tendon length range by slowly pulling and observing finger motion
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
f2_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
f2_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')

f1_palm_adr = model.jnt_qposadr[f1_palm]
f2_palm_adr = model.jnt_qposadr[f2_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]
f2_l1l2_adr = model.jnt_qposadr[f2_l1l2]

print("=" * 80)
print("TENDON LENGTH CALIBRATION")
print("=" * 80)
print("\nThis will:")
print("1. Start with excess tendon length (fully open)")
print("2. Slowly retract tendon")
print("3. Detect when fingers start closing (tendon_max)")
print("4. Continue until fingers meet (tendon_min)")
print("\nPress SPACE to start calibration")
print("Press ESC to exit")
print("=" * 80 + "\n")

# State tracking
calibration_state = {
    'started': False,
    'tendon_max': None,
    'tendon_min': None,
    'start_time': 0,
    'target_length': 0.180,  # Start very long
    'pull_rate': -0.001,  # Retract at 1 mm/s
    'initial_palm_angle': None,
    'fingers_closing': False,
    'fingers_met': False
}

def update_calibration(model, data, state):
    """Update calibration state"""
    if not state['started']:
        return
    
    current_time = data.time
    elapsed = current_time - state['start_time']
    
    # Slowly retract tendon
    state['target_length'] = 0.180 + state['pull_rate'] * elapsed
    data.ctrl[gripper_f1_id] = state['target_length']
    data.ctrl[gripper_f2_id] = state['target_length']
    
    # Get current state
    palm_angle = data.qpos[f1_palm_adr]
    actual_tendon = data.ten_length[f1_tendon]
    
    # Store initial angle
    if state['initial_palm_angle'] is None:
        state['initial_palm_angle'] = palm_angle
    
    # Detect when fingers start closing (palm angle changes by >1 degree)
    if not state['fingers_closing']:
        angle_change = abs(np.rad2deg(palm_angle - state['initial_palm_angle']))
        if angle_change > 1.0:
            state['fingers_closing'] = True
            state['tendon_max'] = actual_tendon
            print(f"\n{'='*80}")
            print(f"FINGERS STARTED CLOSING at t={current_time:.2f}s")
            print(f"  Tendon MAX length: {state['tendon_max']:.6f} m ({state['tendon_max']*1000:.2f} mm)")
            print(f"  Palm angle: {np.rad2deg(palm_angle):.2f}°")
            print(f"{'='*80}\n")
    
    # Detect when fingers meet (check for contact or force saturation)
    if state['fingers_closing'] and not state['fingers_met']:
        force = abs(data.actuator_force[gripper_f1_id])
        # If force is high (>40N) or palm angle is very negative, fingers have met
        if force > 40 or palm_angle < np.deg2rad(-80):
            state['fingers_met'] = True
            state['tendon_min'] = actual_tendon
            print(f"\n{'='*80}")
            print(f"FINGERS MET at t={current_time:.2f}s")
            print(f"  Tendon MIN length: {state['tendon_min']:.6f} m ({state['tendon_min']*1000:.2f} mm)")
            print(f"  Palm angle: {np.rad2deg(palm_angle):.2f}°")
            print(f"  Actuator force: {force:.1f} N")
            print(f"{'='*80}\n")
            print(f"\nCALIBRATION COMPLETE!")
            print(f"  Tendon range: {state['tendon_min']:.6f} to {state['tendon_max']:.6f} m")
            print(f"  Total travel: {(state['tendon_max'] - state['tendon_min'])*1000:.2f} mm")
            print(f"\nRecommended ctrlrange: \"{state['tendon_min']:.3f} {state['tendon_max']:.3f}\"")
            print(f"{'='*80}\n")

def key_callback(keycode):
    """Handle keyboard input"""
    if keycode == 32:  # SPACE
        if not calibration_state['started']:
            calibration_state['started'] = True
            calibration_state['start_time'] = data.time
            print("\nCalibration STARTED - slowly retracting tendon...")
            print("Watch the fingers and check console for events\n")

# Initial position - very open
data.ctrl[gripper_f1_id] = 0.180
data.ctrl[gripper_f2_id] = 0.180

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Run simulation
    step_count = 0
    while viewer.is_running():
        step_start = time.time()
        
        # Update calibration
        update_calibration(model, data, calibration_state)
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Print status every 100 steps
        if calibration_state['started'] and step_count % 100 == 0:
            palm_angle = data.qpos[f1_palm_adr]
            l1l2_angle = data.qpos[f1_l1l2_adr]
            tendon_len = data.ten_length[f1_tendon]
            force = data.actuator_force[gripper_f1_id]
            print(f"t={data.time:5.2f}s  Tendon={tendon_len:.6f}m  Palm={np.rad2deg(palm_angle):6.1f}°  L1L2={np.rad2deg(l1l2_angle):5.1f}°  Force={force:5.1f}N")
        
        step_count += 1
        
        # Update viewer
        viewer.sync()
        
        # Maintain real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
        # Stop if calibration complete
        if calibration_state['fingers_met']:
            print("\nCalibration complete. Press ESC to exit or continue observing.")

print("\nDone!")
