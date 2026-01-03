#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("=" * 60)
print("TESTING KEYFRAME APPLICATION")
print("=" * 60)

# Step 1: Check initial positions (default)
print("1. Default initial positions:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        angle_deg = np.degrees(data.qpos[i])
        print(f"   {name}: {angle_deg:7.1f}°")

# Step 2: Apply keyframe
print("\n2. Applying keyframe...")
mujoco.mj_resetDataKeyframe(model, data, 0)  # Apply first keyframe

# Step 3: Check positions after keyframe
print("3. Positions after keyframe application:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        angle_deg = np.degrees(data.qpos[i])
        print(f"   {name}: {angle_deg:7.1f}°")

# Step 4: Run one physics step
print("\n4. Running one physics step...")
mujoco.mj_step(model, data)

# Step 5: Check if positions changed
print("5. Positions after one physics step:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        angle_deg = np.degrees(data.qpos[i])
        print(f"   {name}: {angle_deg:7.1f}°")

print(f"\nTime after step: {data.time:.6f}s")
print("=" * 60)
