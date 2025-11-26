#!/usr/bin/env python3
"""
Simplest tendon calibration: 1mm steps, console output
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

# Get IDs for finger2
f2_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
f2_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')

f2_palm_adr = model.jnt_qposadr[f2_palm]
f2_l1l2_adr = model.jnt_qposadr[f2_l1l2]

print("=" * 80)
print("NATURAL STATE OBSERVATION")
print("=" * 80)
print("Tendon fixed at 180mm - observing natural spring equilibrium")
print("=" * 80 + "\n")

# Parameters
current_length = 0.180  # meters - fixed at 180mm for natural state observation

# Initial control
data.ctrl[gripper_f1_id] = current_length
data.ctrl[gripper_f2_id] = current_length

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Settle briefly
    for _ in range(200):
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # Continuous observation of natural state
    while viewer.is_running():
        # Keep tendon at 180mm (no influence)
        data.ctrl[gripper_f1_id] = current_length
        data.ctrl[gripper_f2_id] = current_length
        
        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Print state every 100 steps (2 seconds at 50Hz)
        if int(data.time * 50) % 100 == 0:
            # Get state
            palm_angle = np.rad2deg(data.qpos[f1_palm_adr])
            l1l2_angle = np.rad2deg(data.qpos[f1_l1l2_adr])
            actual_tendon = data.ten_length[f1_tendon]
            force = abs(data.actuator_force[gripper_f1_id])
            
            # Finger2
            f2_palm_angle = np.rad2deg(data.qpos[f2_palm_adr])
            f2_l1l2_angle = np.rad2deg(data.qpos[f2_l1l2_adr])
            f2_actual_tendon = data.ten_length[f2_tendon]
            f2_force = abs(data.actuator_force[gripper_f2_id])
            
            # Calculate spring torques
            palm_angle_rad = np.deg2rad(palm_angle)
            l1l2_angle_rad = np.deg2rad(l1l2_angle)
            palm_spring_torque = -1.0 * (palm_angle_rad - 0.27)
            l1l2_spring_torque = -1.0 * (l1l2_angle_rad - 1.7)
            
            # Print
            print(f"t={data.time:5.1f}s  Tendon={actual_tendon*1000:5.1f}mm")
            print(f"  F1: Palm={palm_angle:6.1f}°  L1L2={l1l2_angle:5.1f}°  Force={force:5.1f}N")
            print(f"  F2: Palm={f2_palm_angle:6.1f}°  L1L2={f2_l1l2_angle:5.1f}°  Force={f2_force:5.1f}N")
            print(f"  Torques: Palm={palm_spring_torque:.3f}Nm  L1L2={l1l2_spring_torque:.3f}Nm")
            print()

print("\nDone!")
