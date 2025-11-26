#!/usr/bin/env python3
"""
Simple tendon actuation test
"""
import mujoco
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

# Load model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get actuator ID
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

print("Actuating tendon with current spring configuration:")
print("Palm joints: STRONG springs")
print("Tip joints: WEAK springs")
print()

print("Joint positions during actuation:")
print("Control | F1_Palm | F1_Tip | F2_Palm | F2_Tip")
print("--------|---------|--------|---------|--------")

for control in np.linspace(0, -0.5, 11):
    data.ctrl[actuator_id] = control
    mujoco.mj_step(model, data)

    f1_palm = np.degrees(data.qpos[0])
    f1_tip = np.degrees(data.qpos[1])
    f2_palm = np.degrees(data.qpos[2])
    f2_tip = np.degrees(data.qpos[3])

    print("5.2f")

print()
print("With STRONG palm springs and WEAK tip springs:")
print("- Palm joints should resist bending (stay close to 0°)")
print("- Tip joints should bend easily (move toward 90°)")
print("- This is the OPPOSITE of what we want for under-actuation!")
