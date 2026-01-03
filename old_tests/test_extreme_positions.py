#!/usr/bin/env python3
"""
Force extreme joint positions to test L1-L2 collision
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("FORCING EXTREME JOINT POSITIONS FOR L1-L2 COLLISION TEST")
print("=" * 60)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Force joints to positions that should cause L1-L2 collision
print("Setting joints to extreme positions:")
print("- Palm joint: -90° (fully open)")
print("- Tip joint: 90° (fully bent)")
print()

# Convert to radians
palm_angle = np.radians(-90)  # Full open
tip_angle = np.radians(90)    # Full bend

# Check joint limits
palm_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
tip_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

palm_min, palm_max = model.jnt_range[palm_jid]
tip_min, tip_max = model.jnt_range[tip_jid]

print(f"Palm joint limits: {np.degrees(palm_min):.1f}° to {np.degrees(palm_max):.1f}°")
print(f"Tip joint limits: {np.degrees(tip_min):.1f}° to {np.degrees(tip_max):.1f}°")
print(f"Requested palm: {np.degrees(palm_angle):.1f}° (within limits: {palm_min <= palm_angle <= palm_max})")
print(f"Requested tip: {np.degrees(tip_angle):.1f}° (within limits: {tip_min <= tip_angle <= tip_max})")
print()

# Set joint positions directly
data.qpos[0] = palm_angle  # F1 palm
data.qpos[1] = tip_angle   # F1 tip

# Step to update
mujoco.mj_step(model, data)

print("After setting extreme positions:")
print(f"F1 Palm: {np.degrees(data.qpos[0]):.1f}°")
print(f"F1 Tip: {np.degrees(data.qpos[1]):.1f}°")
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
else:
    print("❌ NO CONTACTS DETECTED")
    print("This means L1 and L2 meshes do not intersect even at extreme joint positions!")

print()
print("CONCLUSION:")
if ncon == 0:
    print("- The STL meshes are not designed to collide with each other")
    print("- L1-L2 collision will never happen with current mesh geometry")
    print("- Need different approach: joint limits, contact pairs, or simplified collision geoms")
else:
    print("- L1-L2 collision is possible with current meshes")
    print("- Need to enable proper collision filtering")
