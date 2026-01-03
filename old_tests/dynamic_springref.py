#!/usr/bin/env python3
import mujoco
import numpy as np
import time

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Apply initial keyframe for smooth startup
mujoco.mj_resetDataKeyframe(model, data, 0)

print("=" * 60)
print("DYNAMIC SPRINGREF CHANGE DEMONSTRATION")
print("=" * 60)

# Get joint indices
joint_names = []
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        joint_names.append(name)

print(f"Joints: {joint_names}")

# Initial springrefs (balanced)
initial_springrefs = {
    'F1_palm_knuckle': -2.0,
    'F2_palm_knuckle': -2.0, 
    'F1_knuckle_tip': -0.33,
    'F2_knuckle_tip': -0.33
}

# Target springrefs (with preload)
target_springrefs = {
    'F1_palm_knuckle': -4.71,  # Add 180° preload
    'F2_palm_knuckle': -4.71,  # Add 180° preload
    'F1_knuckle_tip': -0.33,   # Keep same
    'F2_knuckle_tip': -0.33    # Keep same
}

print(f"\nInitial springrefs: {initial_springrefs}")
print(f"Target springrefs: {target_springrefs}")

# Run simulation with initial springrefs for stabilization
print("\nPhase 1: Stabilization with balanced springrefs...")
for step in range(100):
    mujoco.mj_step(model, data)
    if step % 20 == 0:
        angles = [np.degrees(data.qpos[i]) for i in range(model.njnt)]
        print(f"  Step {step:3d}: {angles}")

# Change springrefs dynamically
print("\nPhase 2: Applying preload springrefs...")
for i, name in enumerate(joint_names):
    if name in target_springrefs:
        dof_adr = model.jnt_dofadr[i]  # Get DOF address for this joint
        model.qpos_spring[dof_adr] = target_springrefs[name]
        print(f"  Changed {name} springref to {target_springrefs[name]}")

# Continue simulation with new springrefs
print("\nPhase 3: Response to preload...")
for step in range(200):
    mujoco.mj_step(model, data)
    if step % 40 == 0:
        angles = [np.degrees(data.qpos[i]) for i in range(model.njnt)]
        torques = [data.qfrc_passive[i] for i in range(model.njnt)]
        print(f"  Step {step:3d}: Angles={angles}, Torques={[f'{t:.3f}' for t in torques]}")

print("\n" + "=" * 60)
print("Dynamic springref change complete!")
print("=" * 60)
