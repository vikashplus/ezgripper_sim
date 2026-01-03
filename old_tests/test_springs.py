#!/usr/bin/env python3
"""
Simple test script to verify EZGripper spring configuration
"""
import mujoco
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("=" * 60)
print("EZGripper Spring Test")
print("=" * 60)

# Load model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("✓ Model loaded successfully!")
print(f"  - Joints: {model.njnt}")
print(f"  - Tendons: {model.ntendon}")
print()

# Check spring configurations
print("SPRING CONFIGURATIONS:")
print("-" * 40)
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    stiffness = model.jnt_stiffness[i]
    springref = model.jnt_springref[i]
    range_min, range_max = model.jnt_range[i]
    
    print(f"{joint_name}:")
    print(f"  Stiffness: {stiffness:.6f}")
    print(f"  SpringRef: {springref:.3f}")
    print(f"  Range: {np.degrees(range_min):.1f}° to {np.degrees(range_max):.1f}°")
    print()

# Test tendon actuation
print("TESTING TENDON ACTUATION:")
print("-" * 40)

# Set initial positions to spring references
for i in range(model.njnt):
    data.qpos[i] = model.jnt_springref[i]

mujoco.mj_forward(model, data)

# Get actuator ID
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

print("Starting tendon actuation test...")
print("Actuating from 0 to -0.5 over 50 steps")
print()

for step in range(51):
    # Gradually actuate tendon
    control = -0.5 * (step / 50.0)
    data.ctrl[actuator_id] = control
    
    mujoco.mj_step(model, data)
    
    if step % 10 == 0:
        print(f"Step {step:2d} | Control: {control:.3f} | Joint positions:")
        for i in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            pos_deg = np.degrees(data.qpos[i])
            print(f"  {joint_name}: {pos_deg:6.1f}°")
        print()

print("✓ Spring test complete!")
print("Expected behavior: Palm joints should bend first (weaker springs), then tip joints bend when palm joints reach limits.")
