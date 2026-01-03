#!/usr/bin/env python3
"""
Debug script to check actual world positions of pulleys
"""
import mujoco
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Reset to open position
mujoco.mj_resetData(model, data)
data.ctrl[0] = 0.0
for _ in range(100):
    mujoco.mj_step(model, data)

print("=" * 80)
print("PULLEY WORLD POSITIONS (at open position)")
print("=" * 80)

# Get geom IDs
geoms = [
    'palm_split_pulley',
    'palm_pulley_f1',
    'palm_pulley_f2',
    'f1l1_pulley',
    'f2l1_pulley',
    'f1l2_pulley',
    'f2l2_pulley'
]

for geom_name in geoms:
    try:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        # Get world position
        pos = data.geom_xpos[geom_id]
        print(f"\n{geom_name}:")
        print(f"  World pos: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
    except:
        print(f"\n{geom_name}: NOT FOUND")

print("\n" + "=" * 80)
print("SITE WORLD POSITIONS")
print("=" * 80)

sites = [
    'palm_peg0',
    'palm_split',
    'palm_peg1',
    'f1l1_peg0',
    'f1l1_peg1',
    'f1l2_peg',
    'f2l1_peg0',
    'f2l1_peg1',
    'f2l2_peg'
]

for site_name in sites:
    try:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        pos = data.site_xpos[site_id]
        print(f"\n{site_name}:")
        print(f"  World pos: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
    except:
        print(f"\n{site_name}: NOT FOUND")

print("\n" + "=" * 80)
print("SYMMETRY CHECK")
print("=" * 80)

# Check if F1 and F2 pulleys are symmetric around Y=0
try:
    f1_pulley_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'f1l1_pulley')
    f2_pulley_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'f2l1_pulley')
    
    f1_pos = data.geom_xpos[f1_pulley_id]
    f2_pos = data.geom_xpos[f2_pulley_id]
    
    print(f"\nF1 L1 pulley: [{f1_pos[0]:.6f}, {f1_pos[1]:.6f}, {f1_pos[2]:.6f}]")
    print(f"F2 L1 pulley: [{f2_pos[0]:.6f}, {f2_pos[1]:.6f}, {f2_pos[2]:.6f}]")
    print(f"Y-axis mirror check: F1.y = {f1_pos[1]:.6f}, F2.y = {f2_pos[1]:.6f}")
    print(f"Should be opposite signs: {np.sign(f1_pos[1]) != np.sign(f2_pos[1])}")
    print(f"Should have same X,Z: X diff = {abs(f1_pos[0] - f2_pos[0]):.6f}, Z diff = {abs(f1_pos[2] - f2_pos[2]):.6f}")
except Exception as e:
    print(f"Error: {e}")
