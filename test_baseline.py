#!/usr/bin/env python3
"""
Quick test of baseline EZGripper configuration
"""
import mujoco
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("=" * 60)
print("EZGripper Baseline Configuration Test")
print("=" * 60)

# Load model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("✓ Model loaded successfully!")
print()

# Check spring configurations
print("SPRING CONFIGURATIONS (Baseline):")
print("-" * 40)
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    stiffness = model.jnt_stiffness[i]
    springref = model.jnt_springref[i]
    range_min, range_max = model.jnt_range[i]

    print(f"{joint_name}:")
    print(f"  Stiffness: {stiffness:.6f}")
    if 'palm' in joint_name:
        print("  → WEAKER spring (Palm-L1 joint)")
    else:
        print("  → STRONGER spring (L1-L2 joint)")
    print(f"  SpringRef: {springref:.3f} rad ({np.degrees(springref):.1f}°)")
    print()

print("EXPECTED BEHAVIOR:")
print("- L1-L2 joints (stronger springs) should resist bending first")
print("- Palm joints (weaker springs) should bend more easily")
print("- Tendon pull affects both through pulley system")
print()

# Test tendon actuation
print("TESTING TENDON ACTUATION:")
print("-" * 40)

# Get actuator ID
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

print("Actuating tendon from 0 to -0.3 over 20 steps...")
print("Joint positions (degrees):")
print("Step | Palm L | Palm R | Tip L | Tip R")
print("-----|--------|--------|-------|-------")

for step in range(21):
    # Gradually actuate tendon
    control = -0.3 * (step / 20.0)
    data.ctrl[actuator_id] = control

    mujoco.mj_step(model, data)

    if step % 4 == 0:  # Print every 4 steps
        palm_l = np.degrees(data.qpos[0])
        tip_l = np.degrees(data.qpos[1])
        palm_r = np.degrees(data.qpos[2])
        tip_r = np.degrees(data.qpos[3])
        print("4d")

print()
print("✓ Baseline test complete!")
print("This shows how the weaker palm springs vs stronger tip springs behave.")
