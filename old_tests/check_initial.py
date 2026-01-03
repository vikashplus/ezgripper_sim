#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Reset to keyframe "init" (if it exists)
mujoco.mj_resetDataKeyframe(model, data, 0)  # 0 = first keyframe

# Print initial positions BEFORE any simulation
print("=" * 60)
print("INITIAL POSITIONS AT TIME ZERO (Keyframe Applied)")
print("=" * 60)

# Get joint names
joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]

print("Joint positions from keyframe:")
for i, name in enumerate(joint_names):
    if name:  # Skip None entries
        angle_deg = np.degrees(data.qpos[i])
        print(f"  {name}: {angle_deg:7.1f}°")

print(f"\nKeyframe time: {data.time:.6f}s")
print("=" * 60)
