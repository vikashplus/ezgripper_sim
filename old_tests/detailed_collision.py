#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Apply keyframe for stable start
mujoco.mj_resetDataKeyframe(model, data, 0)

# Run a few steps to reach equilibrium
for _ in range(500):
    mujoco.mj_step(model, data)

print("=" * 60)
print("DETAILED COLLISION ANALYSIS")
print("=" * 60)

print(f"\nSimulation time: {data.time:.3f}s")
print(f"Number of contacts: {data.ncon}")

for i in range(data.ncon):
    contact = data.contact[i]
    geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
    geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
    
    print(f"\nContact {i}:")
    print(f"  Geoms: {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}")
    print(f"  Distance: {contact.dist:.6f}m")
    print(f"  Position: ({contact.pos[0]:.4f}, {contact.pos[1]:.4f}, {contact.pos[2]:.4f})")
    print(f"  Normal: ({contact.frame[0]:.3f}, {contact.frame[1]:.3f}, {contact.frame[2]:.3f})")
    
    # Get geom positions for reference
    if geom1_name:
        geom1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name)
        if geom1_id >= 0:
            geom1_pos = data.geom_xpos[geom1_id]
            print(f"  {geom1_name} pos: ({geom1_pos[0]:.4f}, {geom1_pos[1]:.4f}, {geom1_pos[2]:.4f})")
    
    if geom2_name:
        geom2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name)
        if geom2_id >= 0:
            geom2_pos = data.geom_xpos[geom2_id]
            print(f"  {geom2_name} pos: ({geom2_pos[0]:.4f}, {geom2_pos[1]:.4f}, {geom2_pos[2]:.4f})")

print("\n" + "=" * 60)
print("Analysis complete!")
print("=" * 60)
