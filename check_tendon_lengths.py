#!/usr/bin/env python3
"""
Check tendon lengths at different joint positions to find max path length.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("TENDON LENGTH ANALYSIS")
print("="*80)

# Get joint ranges
f1_palm_range = model.jnt_range[0]  # Palm-L1
f1_tip_range = model.jnt_range[1]   # L1-L2

print(f"\nJoint Ranges:")
print(f"  Palm-L1: {np.rad2deg(f1_palm_range[0]):.1f}° to {np.rad2deg(f1_palm_range[1]):.1f}°")
print(f"  L1-L2:   {np.rad2deg(f1_tip_range[0]):.1f}° to {np.rad2deg(f1_tip_range[1]):.1f}°")

print("\n" + "="*80)
print("Testing different joint configurations:")
print("="*80)

# Test configurations
configs = [
    ("Fully closed", 0.436, 0.0),  # Palm at +25°, L1-L2 at 0°
    ("Neutral", 0.0, 0.0),          # Both at 0°
    ("Palm open", -1.57, 0.0),      # Palm at -90°, L1-L2 at 0°
    ("L1-L2 open", 0.0, 1.7),       # Palm at 0°, L1-L2 at 97°
    ("Both open", -1.57, 1.7),      # Palm at -90°, L1-L2 at 97°
    ("Palm half", -0.785, 0.0),     # Palm at -45°, L1-L2 at 0°
    ("L1-L2 half", 0.0, 0.85),      # Palm at 0°, L1-L2 at 48.5°
]

max_length = 0
max_config = None

for name, palm_angle, tip_angle in configs:
    # Set joint positions
    data.qpos[0] = palm_angle  # F1_palm
    data.qpos[1] = tip_angle   # F1_tip
    data.qpos[2] = palm_angle  # F2_palm
    data.qpos[3] = tip_angle   # F2_tip
    
    # Update kinematics and tendon lengths
    mujoco.mj_kinematics(model, data)
    mujoco.mj_tendon(model, data)
    
    tendon1_length = data.ten_length[0] * 1000  # Convert to mm
    tendon2_length = data.ten_length[1] * 1000
    
    print(f"\n{name}:")
    print(f"  Palm-L1: {np.rad2deg(palm_angle):6.1f}° | L1-L2: {np.rad2deg(tip_angle):6.1f}°")
    print(f"  Tendon F1: {tendon1_length:.2f} mm")
    print(f"  Tendon F2: {tendon2_length:.2f} mm")
    
    if tendon1_length > max_length:
        max_length = tendon1_length
        max_config = (name, palm_angle, tip_angle)

print("\n" + "="*80)
print("MAXIMUM TENDON LENGTH:")
print("="*80)
print(f"Configuration: {max_config[0]}")
print(f"  Palm-L1: {np.rad2deg(max_config[1]):.1f}°")
print(f"  L1-L2:   {np.rad2deg(max_config[2]):.1f}°")
print(f"  Max length: {max_length:.2f} mm")
print("="*80)

# Now test the current "stuck" position
print("\nCurrent 'stuck' position (from test):")
data.qpos[0] = 0.0  # Palm at 0°
data.qpos[1] = -0.015  # L1-L2 at -0.8°
mujoco.mj_kinematics(model, data)
mujoco.mj_tendon(model, data)
print(f"  Palm-L1: 0.0° | L1-L2: -0.8°")
print(f"  Tendon length: {data.ten_length[0]*1000:.2f} mm")
