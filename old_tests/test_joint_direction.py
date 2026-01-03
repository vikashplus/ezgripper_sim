#!/usr/bin/env python3
"""
Test L1-L2 joint direction during tendon actuation
"""
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("TESTING L1-L2 JOINT DIRECTION DURING TENDON ACTUATION")
print("=" * 60)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')
f1_tip_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

print("Testing L1-L2 joint response to tendon actuation:")
print("Joint range: -28.6° to 97.4°")
print("Positive angles = closing, Negative angles = opening")
print()

# Start from neutral position (no tendon force)
data.ctrl[actuator_id] = 0
mujoco.mj_step(model, data)

initial_angle = np.degrees(data.qpos[1])
print(f"Initial L1-L2 angle: {initial_angle:.2f}°")
print()

# Apply increasing tendon force
print("Applying tendon force (closing gripper):")
print("Control | L1-L2 Angle | Direction")
print("--------|-------------|----------")

prev_angle = initial_angle
for ctrl in np.linspace(0, -0.5, 6):
    data.ctrl[actuator_id] = ctrl
    mujoco.mj_step(model, data)

    current_angle = np.degrees(data.qpos[1])
    direction = "CLOSING" if current_angle > prev_angle else "OPENING"

    print("6.2f")

    prev_angle = current_angle

print()
print("ANALYSIS:")
if prev_angle > initial_angle:
    print("✅ CORRECT: L1-L2 joint closes when tendon tightens")
    print("   Tendon routing creates proper torque for closing motion")
else:
    print("❌ PROBLEM: L1-L2 joint opens when tendon tightens")
    print("   Tendon may be on wrong side of joint or pulley positioned incorrectly")
    print("   This could cause L1-L2 separation and mesh overlap issues")

print()
print("If tendon is on wrong side:")
print("- Swap pulley position (left/right)")
print("- Or reverse tendon direction through pulley")
print("- Or adjust site positions for correct mechanical advantage")
