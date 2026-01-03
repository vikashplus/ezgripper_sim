#!/usr/bin/env python3
"""
Check collision box positions and test intersection
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("CHECKING COLLISION BOX POSITIONS")
print("=" * 40)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get collision geom IDs
collision_names = ['f1l1_collision', 'f1l2_collision', 'f2l1_collision', 'f2l2_collision']
collision_ids = {}

for name in collision_names:
    try:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        collision_ids[name] = gid
        print(f"✓ Found collision geom '{name}' at ID {gid}")
    except:
        print(f"✗ Collision geom '{name}' not found")

print()

# Check positions at zero joint angles (open position)
print("COLLISION BOX POSITIONS (open position):")
for name, gid in collision_ids.items():
    # Get body position that this geom belongs to
    body_id = model.geom_bodyid[gid]
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

    # Get geom position relative to body
    geom_pos = model.geom_pos[gid]

    print(f"{name}: body='{body_name}', relative_pos={geom_pos}")

print()

# Check positions at extreme angles
print("TESTING AT EXTREME ANGLES:")
data.qpos[0] = np.radians(-90)  # F1 palm fully open
data.qpos[1] = np.radians(90)   # F1 tip fully bent
mujoco.mj_step(model, data)

print("F1 joints at -90°, 90°:")
for name, gid in collision_ids.items():
    if 'f1' in name:
        # Get global position
        mujoco.mj_step(model, data)  # Update forward kinematics
        pos = data.geom_xpos[gid]
        print(f"  {name}: global_pos={pos}")

print()

# Check for contacts
ncon = data.ncon
print(f"Contacts detected: {ncon}")

if ncon > 0:
    print("CONTACT DETAILS:")
    for i in range(ncon):
        geom1 = data.contact[i].geom1
        geom2 = data.contact[i].geom2
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
        dist = data.contact[i].dist
        print(f"  {geom1_name} ↔ {geom2_name}: dist={dist:.6f}")
        if 'collision' in geom1_name and 'collision' in geom2_name:
            print("  🎯 L1-L2 COLLISION DETECTED!")
else:
    print("❌ NO CONTACTS - collision boxes may not be intersecting")

print()
print("COLLISION BOX SIZES: 0.003 x 0.003 x 0.003 (very small)")
print("If no collision, try:")
print("1. Larger collision boxes")
print("2. Different positions")
print("3. Force joints to more extreme angles")
