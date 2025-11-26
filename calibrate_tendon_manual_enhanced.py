#!/usr/bin/env python3
"""
Enhanced manual tendon calibration with stalling detection
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
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f1_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')

f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]

print("=" * 80)
print("ENHANCED MANUAL TENDON CALIBRATION")
print("=" * 80)
print("Controls:")
print("  UP:    Increase tendon length (open)")
print("  DOWN:  Decrease tendon length (close)")
print("  LEFT:  Step -0.1mm")
print("  RIGHT: Step +0.1mm")
print("  'S':   Mark start of closing (tendon_max)")
print("  'E':   Mark fingers met (tendon_min)")
print("  'F':   Mark force stall (tendon_stall)")
print("  ESC:   Exit")
print("=" * 80 + "\n")

# State
current_length = 0.180  # start open
step_size = 0.0001  # 0.1 mm
tendon_max = None
tendon_min = None
tendon_stall = None
prev_force = 0
prev_palm_angle = data.qpos[f1_palm_adr]
prev_l1l2_angle = data.qpos[f1_l1l2_adr]

# For stalling detection
stall_force_threshold = 0  # will be set after 'E' is pressed
stall_angle_change_threshold = 0.001  # radians (about 0.05 degrees)

# Flags
marking_stall = False

def key_callback(keycode):
    global current_length, tendon_max, tendon_min, tendon_stall, marking_stall
    if keycode == 265:  # UP arrow
        current_length += step_size
    elif keycode == 264:  # DOWN arrow
        current_length -= step_size
    elif keycode == 263:  # LEFT arrow
        current_length -= step_size
    elif keycode == 262:  # RIGHT arrow
        current_length += step_size
    elif keycode == 83:   # 'S'
        tendon_max = data.ten_length[f1_tendon]
        print(f"\nMARKED START OF CLOSING: {tendon_max:.6f} m")
    elif keycode == 69:   # 'E'
        tendon_min = data.ten_length[f1_tendon]
        print(f"\nMARKED FINGERS MET: {tendon_min:.6f} m")
        marking_stall = True
        print("Now monitoring for stall (force doubles without joint movement)")
    elif keycode == 70:   # 'F'
        tendon_stall = data.ten_length[f1_tendon]
        print(f"\nMARKED STALL: {tendon_stall:.6f} m")
    # Set control
data.ctrl[gripper_f1_id] = current_length
data.ctrl[gripper_f2_id] = current_length

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Initial control
    data.ctrl[gripper_f1_id] = current_length
    data.ctrl[gripper_f2_id] = current_length
    
    # Run simulation
    while viewer.is_running():
        # Get current state
        palm_angle = data.qpos[f1_palm_adr]
        l1l2_angle = data.qpos[f1_l1l2_adr]
        actual_tendon = data.ten_length[f1_tendon]
        force = abs(data.actuator_force[gripper_f1_id])
        
        # Print status
        status = f"Tendon: {current_length*1000:.2f} mm (Actual: {actual_tendon*1000:.2f} mm)  Palm: {np.rad2deg(palm_angle):.2f}°  L1L2: {np.rad2deg(l1l2_angle):.2f}°  Force: {force:.2f}N"
        if tendon_max:
            status += f"  MAX: {tendon_max*1000:.2f}mm"
        if tendon_min:
            status += f"  MIN: {tendon_min*1000:.2f}mm"
        if tendon_stall:
            status += f"  STALL: {tendon_stall*1000:.2f}mm"
        print(status, end='\r')
        
        # Stalling detection after 'E' is pressed
        if marking_stall and not tendon_stall:
            # Check if force has doubled since marking 'E'
            if force > 2 * stall_force_threshold:
                # Check if joint angles haven't changed much
                palm_moved = abs(palm_angle - prev_palm_angle) > stall_angle_change_threshold
                l1l2_moved = abs(l1l2_angle - prev_l1l2_angle) > stall_angle_change_threshold
                if not palm_moved and not l1l2_moved:
                    tendon_stall = actual_tendon
                    print(f"\nAUTO-DETECTED STALL: {tendon_stall:.6f} m")
            # Update previous angles
            prev_palm_angle = palm_angle
            prev_l1l2_angle = l1l2_angle
        
        # Step
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Final report
    print("\n" + "=" * 80)
    print("CALIBRATION RESULTS")
    if tendon_max is not None:
        print(f"  Tendon MAX (open):   {tendon_max:.6f} m ({tendon_max*1000:.2f} mm)")
    if tendon_min is not None:
        print(f"  Tendon MIN (closed): {tendon_min:.6f} m ({tendon_min*1000:.2f} mm)")
    if tendon_stall is not None:
        print(f"  Tendon STALL:        {tendon_stall:.6f} m ({tendon_stall*1000:.2f} mm)")
    if tendon_max is not None and tendon_min is not None:
        print(f"  Total travel:        {(tendon_max - tendon_min)*1000:.2f} mm")
    print("=" * 80)

print("\nDone!")
