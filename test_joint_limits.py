#!/usr/bin/env python3
"""
Test L1-L2 joint opening collision
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("TESTING L1-L2 JOINT OPENING COLLISION")
print("=" * 50)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get joint IDs
f1_tip_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_tip_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

print("Joint ranges:")
print(f"F1_knuckle_tip: {np.degrees(model.jnt_range[f1_tip_jid][0]):.1f}° to {np.degrees(model.jnt_range[f1_tip_jid][1]):.1f}°")
print(f"F2_knuckle_tip: {np.degrees(model.jnt_range[f2_tip_jid][0]):.1f}° to {np.degrees(model.jnt_range[f2_tip_jid][1]):.1f}°")
print()

# Test opening the L1-L2 joints
print("TESTING JOINT OPENING:")
opening_angles = [0, -0.2, -0.4, -0.6]  # Try to open beyond -0.5 limit

for angle in opening_angles:
    # Set joint angles
    data.qpos[1] = angle  # F1 L1-L2 joint
    data.qpos[3] = angle  # F2 L1-L2 joint

    mujoco.mj_step(model, data)

    # Check for contacts
    ncon = data.ncon
    l1_l2_contacts = 0

    if ncon > 0:
        for i in range(ncon):
            geom1 = data.contact[i].geom1
            geom2 = data.contact[i].geom2
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)

            if ('l1_collision' in geom1_name and 'l2_collision' in geom2_name) or \
               ('l1_collision' in geom2_name and 'l2_collision' in geom1_name):
                l1_l2_contacts += 1

    angle_deg = np.degrees(angle)
    within_limit = angle >= model.jnt_range[f1_tip_jid][0]

    print("3.1f")

print()
print("RESULTS:")
if l1_l2_contacts > 0:
    print("✅ L1-L2 COLLISION DETECTED! Joint opening is physically stopped.")
    print("This simulates the hard stops in the real EZGripper.")
else:
    print("❌ No L1-L2 collision detected. Collision boxes may not be positioned correctly.")
    print("The joint can open freely without physical constraints.")
