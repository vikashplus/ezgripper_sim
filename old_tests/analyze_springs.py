#!/usr/bin/env python3
"""
Compare original vs corrected spring configurations
"""
import mujoco
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

# Load current model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get actuator ID
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

print("=" * 70)
print("SPRING CONFIGURATION ANALYSIS")
print("=" * 70)

print("\nCURRENT (CORRECTED) CONFIGURATION:")
print("- Palm-L1 joints: WEAK springs (0.024599)")
print("- L1-L2 joints: STRONG springs (0.051141)")
print("- Expected: L1-L2 resists bending → under-actuation")

print("\nORIGINAL (PROBLEMATIC) CONFIGURATION:")
print("- Palm-L1 joints: STRONG springs (0.051141)")
print("- L1-L2 joints: WEAK springs (0.024599)")
print("- Result: Fingers close straight together, no L1-L2 bending")

print("\nTESTING CURRENT CONFIGURATION:")
print("Control | F1_Palm | F1_Tip | F2_Palm | F2_Tip")
print("--------|---------|--------|---------|--------")

for control in np.linspace(0, -0.3, 7):
    data.ctrl[actuator_id] = control
    mujoco.mj_step(model, data)

    f1_palm = np.degrees(data.qpos[0])
    f1_tip = np.degrees(data.qpos[1])
    f2_palm = np.degrees(data.qpos[2])
    f2_tip = np.degrees(data.qpos[3])

    print("5.1f")

print("\n" + "=" * 70)
print("ANALYSIS:")
print("With STRONG L1-L2 springs:")
print("- L1-L2 joints resist bending first")
print("- Palm joints bend more easily")
print("- This enables under-actuation when contacting objects")
print()
print("With WEAK L1-L2 springs (original issue):")
print("- L1-L2 joints bend immediately")
print("- Fingers close like parallel jaws")
print("- No wrapping behavior around objects")
print("=" * 70)
