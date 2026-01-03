#!/usr/bin/env python3
"""
Manual tendon calibration: User presses keys to mark events
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
print("MANUAL TENDON CALIBRATION")
print("=" * 80)
print("Controls:")
print("  UP:    Increase tendon length (open)")
print("  DOWN:  Decrease tendon length (close)")
print("  LEFT:  Step -0.1mm")
print("  RIGHT: Step +0.1mm")
print("  'S':   Mark start of closing (tendon_max)")
print("  'E':   Mark fingers met (tendon_min)")
print("  ESC:   Exit")
print("=" * 80 + "\n")

# State
current_length = 0.165  # initial guess
step_size = 0.0001  # 0.1 mm
tendon_max = None
tendon_min = None

def key_callback(keycode):
    global current_length, tendon_max, tendon_min
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
        # Print status
        print(f"Tendon: {current_length*1000:.2f} mm  Palm: {np.rad2deg(data.qpos[f1_palm_adr]):.2f}°  L1L2: {np.rad2deg(data.qpos[f1_l1l2_adr]):.2f}°", end='\r')
        # Step
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Final report
    if tendon_max is not None and tendon_min is not None:
        print("\n" + "=" * 80)
        print("CALIBRATION RESULTS")
        print(f"  Tendon MAX (open):   {tendon_max:.6f} m ({tendon_max*1000:.2f} mm)")
        print(f"  Tendon MIN (closed): {tendon_min:.6f} m ({tendon_min*1000:.2f} mm)")
        print(f"  Total travel:        {(tendon_max - tendon_min)*1000:.2f} mm")
        print("=" * 80)

print("\nDone!")
