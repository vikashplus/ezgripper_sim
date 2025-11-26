#!/usr/bin/env python3
"""
Debug tendon routing and joint motion
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("DEBUGGING TENDON ROUTING AND L1-L2 JOINT MOTION")
print("=" * 60)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get joint IDs
f1_palm_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_tip_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

print("INITIAL JOINT POSITIONS:")
print(f"F1 Palm: {np.degrees(data.qpos[0]):.2f}°")
print(f"F1 Tip: {np.degrees(data.qpos[1]):.2f}°")
print()

# Test tendon actuation
print("TESTING TENDON ACTUATION (closing gripper):")
controls = [0, -0.2, -0.4, -0.6, -0.8]

for ctrl in controls:
    data.ctrl[actuator_id] = ctrl
    mujoco.mj_step(model, data)

    palm_angle = np.degrees(data.qpos[0])
    tip_angle = np.degrees(data.qpos[1])

    print("4.1f")

print()

# Check site positions
print("SITE POSITIONS FOR TENDON ROUTING:")
sites = ['f1l1_peg1', 'f1l2_pulley', 'f1l2_peg']
for site_name in sites:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    site_pos = model.site_pos[site_id]
    body_id = model.site_bodyid[site_id]
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    print(f"{site_name}: body={body_name}, pos={site_pos}")

print()

# Check if L1-L2 are overlapping
print("CHECKING FOR L1-L2 OVERLAP:")
# Reset to neutral
data.ctrl[actuator_id] = 0
mujoco.mj_step(model, data)

ncon = data.ncon
overlap_detected = False

if ncon > 0:
    for i in range(ncon):
        geom1 = data.contact[i].geom1
        geom2 = data.contact[i].geom2
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)

        if ('f1l1' in geom1_name and 'f1l2' in geom2_name) or \
           ('f1l1' in geom2_name and 'f1l2' in geom1_name):
            overlap_detected = True
            dist = data.contact[i].dist
            print(f"⚠️  L1-L2 OVERLAP: {geom1_name} ↔ {geom2_name}, dist={dist:.6f}")

if not overlap_detected:
    print("✅ No L1-L2 overlap detected")

print()

# Analyze tendon routing
print("TENDON ROUTING ANALYSIS:")
print("L1-L2 joint axis: [0, 1, 0] (y-axis)")
print("Tendon path: f1l1_peg1 (L1) → f1l2_pulley (L2) → f1l2_peg (L2)")
print()
print("For closing motion (positive L1-L2 angle):")
print("- Tendon should pull L2 clockwise around y-axis")
print("- Sites should be positioned for correct torque direction")
print()
print("ISSUE: If L1-L2 angle goes negative during closing,")
print("       the tendon routing may be on the wrong side of the joint!")
