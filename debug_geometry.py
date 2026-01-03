#!/usr/bin/env python3
"""
Debug the finger geometry to understand why collision isn't working.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("GEOMETRY DEBUG")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get collision geom IDs
f1_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1_tip")
f2_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f2_tip")

print(f"\nFinger tip collision geom IDs: f1={f1_tip_geom_id}, f2={f2_tip_geom_id}")

# Reset to neutral position
mujoco.mj_resetData(model, data)

print(f"\nInitial positions (neutral):")
print(f"F1: Palm={np.degrees(data.qpos[f1_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f1_tip_id]):6.1f}°")
print(f"F2: Palm={np.degrees(data.qpos[f2_palm_id]):6.1f}°, Tip={np.degrees(data.qpos[f2_tip_id]):6.1f}°")

# Get initial geom positions
f1_tip_pos = data.geom_xpos[f1_tip_geom_id]
f2_tip_pos = data.geom_xpos[f2_tip_geom_id]

print(f"\nInitial collision box positions:")
print(f"F1 tip: [{f1_tip_pos[0]:.4f}, {f1_tip_pos[1]:.4f}, {f1_tip_pos[2]:.4f}]")
print(f"F2 tip: [{f2_tip_pos[0]:.4f}, {f2_tip_pos[1]:.4f}, {f2_tip_pos[2]:.4f}]")
print(f"Distance: {np.linalg.norm(f1_tip_pos - f2_tip_pos):.4f}")

# Test different joint angles
test_angles = [-30, -20, -10, 0, 10, 20, 30]

print(f"\nTesting different palm joint angles:")
print("Angle  F1_Palm  F1_Tip  F2_Palm  F2_Tip  F1_Pos[X,Y]  F2_Pos[X,Y]  Distance")

for angle in test_angles:
    # Reset and set angle
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = np.radians(angle)
    data.qpos[f2_palm_id] = np.radians(angle)
    
    # Forward kinematics
    mujoco.mj_forward(model, data)
    
    # Get positions
    f1_tip_pos = data.geom_xpos[f1_tip_geom_id]
    f2_tip_pos = data.geom_xpos[f2_tip_geom_id]
    distance = np.linalg.norm(f1_tip_pos - f2_tip_pos)
    
    print(f"{angle:4d}°  {np.degrees(data.qpos[f1_palm_id]):6.1f}°  {np.degrees(data.qpos[f1_tip_id]):6.1f}°  "
          f"{np.degrees(data.qpos[f2_palm_id]):6.1f}°  {np.degrees(data.qpos[f2_tip_id]):6.1f}°  "
          f"[{f1_tip_pos[0]:.3f},{f1_tip_pos[1]:.3f}]  [{f2_tip_pos[0]:.3f},{f2_tip_pos[1]:.3f}]  {distance:.4f}")

# Test with force control to see actual motion
print(f"\nTesting with force control:")
data.ctrl[f1_actuator_id] = 1.0
data.ctrl[f2_actuator_id] = 1.0

for step in range(100):
    mujoco.mj_step(model, data)
    
    if step % 20 == 0:
        f1_tip_pos = data.geom_xpos[f1_tip_geom_id]
        f2_tip_pos = data.geom_xpos[f2_tip_geom_id]
        distance = np.linalg.norm(f1_tip_pos - f2_tip_pos)
        
        print(f"Step {step:3d}: F1({np.degrees(data.qpos[f1_palm_id]):6.1f}°,{np.degrees(data.qpos[f1_tip_id]):6.1f}°) "
              f"F2({np.degrees(data.qpos[f2_palm_id]):6.1f}°,{np.degrees(data.qpos[f2_tip_id]):6.1f}°) "
              f"Dist={distance:.4f} Contacts={data.ncon}")

print("\n" + "="*80)
print("GEOMETRY DEBUG COMPLETE")
print("="*80)
