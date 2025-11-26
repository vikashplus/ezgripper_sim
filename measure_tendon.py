#!/usr/bin/env python3
"""
Measure tendon length at full open and full closed positions
"""
import mujoco
import numpy as np
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "demo_tennis_ball.xml")

print("=" * 60)
print("EZGripper Tendon Length Measurement")
print("=" * 60)

# Load the model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get IDs
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'flex_tendon')

print(f"\nModel loaded: {model_path}")
print(f"Actuator: gripper_actuator (ID: {gripper_actuator_id})")
print(f"Tendon: flex_tendon (ID: {tendon_id})")

# Measure FULL OPEN (actuator = 0, no tension)
print("\n" + "=" * 60)
print("FULL OPEN (actuator = 0, springs push fingers open)")
print("=" * 60)
data.ctrl[gripper_actuator_id] = 0.0
for _ in range(1000):  # Let it settle
    mujoco.mj_step(model, data)

tendon_length_open = data.ten_length[tendon_id]
print(f"Tendon length: {tendon_length_open:.6f} m ({tendon_length_open*1000:.3f} mm)")
print(f"Joint angles:")
print(f"  F1_palm_knuckle: {data.qpos[0]:.4f} rad ({np.degrees(data.qpos[0]):.2f}°)")
print(f"  F1_knuckle_tip:  {data.qpos[1]:.4f} rad ({np.degrees(data.qpos[1]):.2f}°)")

# Measure FULL CLOSED (actuator = -1, maximum tension)
print("\n" + "=" * 60)
print("FULL CLOSED (actuator = -1, maximum tension)")
print("=" * 60)
data.ctrl[gripper_actuator_id] = -1.0
for _ in range(1000):  # Let it settle
    mujoco.mj_step(model, data)

tendon_length_closed = data.ten_length[tendon_id]
print(f"Tendon length: {tendon_length_closed:.6f} m ({tendon_length_closed*1000:.3f} mm)")
print(f"Joint angles:")
print(f"  F1_palm_knuckle: {data.qpos[0]:.4f} rad ({np.degrees(data.qpos[0]):.2f}°)")
print(f"  F1_knuckle_tip:  {data.qpos[1]:.4f} rad ({np.degrees(data.qpos[1]):.2f}°)")

# Calculate delta
delta = tendon_length_open - tendon_length_closed
print("\n" + "=" * 60)
print("TENDON TRAVEL")
print("=" * 60)
print(f"Open length:   {tendon_length_open:.6f} m ({tendon_length_open*1000:.3f} mm)")
print(f"Closed length: {tendon_length_closed:.6f} m ({tendon_length_closed*1000:.3f} mm)")
print(f"Delta (travel): {delta:.6f} m ({delta*1000:.3f} mm)")
print(f"\nCurrent model range: 0.10 to 0.20 m (100 mm)")
print(f"Suggested range: {tendon_length_closed:.6f} to {tendon_length_open:.6f} m")
print("=" * 60)
