#!/usr/bin/env python3
"""
Close gripper until fingers stop
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

# Get actuator IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')

print("=" * 60)
print("EZGripper - Close Until Stopped")
print("=" * 60)

# Command fully closed (minimum tendon length)
closed_length = 0.130  # meters - very short
data.ctrl[gripper_f1_id] = closed_length
data.ctrl[gripper_f2_id] = closed_length

print(f"Commanding tendon length: {closed_length:.4f} m (fully closed)")
print("\nLaunching viewer...")
print("Press ESC to exit")
print("=" * 60 + "\n")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Run simulation
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

print("\nDone!")
