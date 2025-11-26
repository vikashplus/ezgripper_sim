#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("=" * 60)
print("MODEL ACTUATOR AND TENDON ANALYSIS")
print("=" * 60)

print(f"\nNumber of actuators: {model.nu}")
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"  Actuator {i}: {name}")

print(f"\nNumber of tendons: {model.ntendon}")
for i in range(model.ntendon):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
    print(f"  Tendon {i}: {name}")

print(f"\nNumber of joints: {model.njnt}")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        print(f"  Joint {i}: {name}")

# Check actuator types
print(f"\nActuator details:")
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"  {name}: trntype={model.actuator_trntype[i]}, gear={model.actuator_gainprm[i,0]:.3f}")

# Check tendon details
print(f"\nTendon details:")
for i in range(model.ntendon):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
    print(f"  {name}: length0={model.tendon_length0[i]:.3f}, stiffness={model.tendon_stiffness[i]:.3f}")

print("\n" + "=" * 60)
print("Analysis complete!")
print("=" * 60)
