#!/usr/bin/env python3
"""
Debug L1-L2 collision detection
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("DEBUGGING L1-L2 COLLISION DETECTION")
print("=" * 50)

# Load model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get geom IDs
geom_names = ['f1l1', 'f1l2', 'f2l1', 'f2l2']
geom_ids = {}
for name in geom_names:
    try:
        geom_ids[name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        print(f"✓ Found geom '{name}' at ID {geom_ids[name]}")
    except:
        print(f"✗ Geom '{name}' not found")

print()

# Check collision properties
print("COLLISION GEOM PROPERTIES:")
for name in geom_names:
    if name in geom_ids:
        gid = geom_ids[name]
        contype = model.geom_contype[gid]
        conaffinity = model.geom_conaffinity[gid]
        print(f"{name}: contype={contype}, conaffinity={conaffinity}")

print()

# Test if geoms can collide
print("COLLISION COMPATIBILITY TEST:")
test_pairs = [('f1l1', 'f1l2'), ('f2l1', 'f2l2')]
for pair in test_pairs:
    if pair[0] in geom_ids and pair[1] in geom_ids:
        g1, g2 = geom_ids[pair[0]], geom_ids[pair[1]]
        can_collide = (model.geom_contype[g1] & model.geom_conaffinity[g2]) and \
                     (model.geom_contype[g2] & model.geom_conaffinity[g1])
        print(f"{pair[0]} ↔ {pair[1]}: {'✓ CAN COLLIDE' if can_collide else '✗ CANNOT COLLIDE'}")

print()

# Check default collision settings
print("DEFAULT COLLISION SETTINGS:")
print(f"Default contype: {model.opt.contype}")
print(f"Default conaffinity: {model.opt.conaffinity}")
print(f"Default margin: {model.opt.margin}")
print(f"Default solref: {model.opt.solref}")
print(f"Default solimp: {model.opt.solimp}")

print()

# Check joint ranges
print("JOINT RANGES:")
joint_names = ['F1_palm_knuckle', 'F1_knuckle_tip', 'F2_palm_knuckle', 'F2_knuckle_tip']
for name in joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    range_min, range_max = model.jnt_range[jid]
    print(f"{name}: {np.degrees(range_min):.1f}° to {np.degrees(range_max):.1f}°")

print()

# Test collision detection manually
print("TESTING COLLISION DETECTION:")
data.ctrl[0] = -0.5  # Close gripper

for step in range(10):
    mujoco.mj_step(model, data)

    # Check for contacts
    ncon = data.ncon
    if ncon > 0:
        print(f"Step {step}: {ncon} contacts detected!")
        for i in range(ncon):
            geom1 = data.contact[i].geom1
            geom2 = data.contact[i].geom2
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
            print(f"  Contact {i}: {geom1_name} ↔ {geom2_name}")
    else:
        if step % 3 == 0:
            f1_palm = np.degrees(data.qpos[0])
            f1_tip = np.degrees(data.qpos[1])
            print(f"Step {step}: No contacts, joints at {f1_palm:.1f}°, {f1_tip:.1f}°")

print()
print("ANALYSIS:")
print("- If no L1-L2 contacts detected, they may not be intersecting")
print("- Check if joint ranges allow sufficient bending for collision")
print("- Mesh geometry might not intersect even at extreme joint angles")
