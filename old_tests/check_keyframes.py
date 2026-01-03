#!/usr/bin/env python3
import mujoco

# Load model and check keyframes
model = mujoco.MjModel.from_xml_path('ezgripper.xml')

print("Keyframe Information:")
print(f"Number of keyframes: {model.nkey}")
for i in range(model.nkey):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
    print(f"  Keyframe {i}: name='{name}'")

print("\nJoint Information:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        print(f"  Joint {i}: {name}")
