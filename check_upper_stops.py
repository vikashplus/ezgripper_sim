#!/usr/bin/env python3
"""
Check why upper stops are colliding with finger mesh.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("UPPER STOP COLLISION ANALYSIS")
print("="*80)

f1_palm_id = model.joint('F1_palm_knuckle').id

# Check at different angles
angles = [-90, -45, -20, 0, 25]

print(f"\nUpper stop vs finger mesh collisions:")
print("="*50)

for angle in angles:
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_step(model, data)
    
    # Check for upper stop collisions
    palm_upper_id = model.geom('palm_stop_f1_upper').id
    finger_mesh_id = model.geom('f1l1').id
    
    collision_found = False
    for i in range(data.ncon):
        contact = data.contact[i]
        if (contact.geom1 == palm_upper_id and contact.geom2 == finger_mesh_id) or \
           (contact.geom1 == finger_mesh_id and contact.geom2 == palm_upper_id):
            print(f"  {angle:3.0f}°: COLLISION (dist={contact.dist:.6f})")
            collision_found = True
            break
    
    if not collision_found:
        print(f"  {angle:3.0f}°: No collision")

print(f"\n" + "="*50)
print("UPPER STOP POSITIONS:")
print("="*50)

# Get positions at different angles
for angle in [-90, 0, 25]:
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_kinematics(model, data)
    
    palm_upper_pos = data.geom_xpos[model.geom('palm_stop_f1_upper').id]
    finger_mesh_pos = data.geom_xpos[model.geom('f1l1').id]
    
    print(f"\nAt {angle:3.0f}°:")
    print(f"  Palm upper stop: [{palm_upper_pos[0]:.6f}, {palm_upper_pos[1]:.6f}, {palm_upper_pos[2]:.6f}]")
    print(f"  Finger mesh: [{finger_mesh_pos[0]:.6f}, {finger_mesh_pos[1]:.6f}, {finger_mesh_pos[2]:.6f}]")
    
    # Find closest point on finger mesh to upper stop
    dist = np.linalg.norm(palm_upper_pos - finger_mesh_pos)
    print(f"  Distance: {dist:.6f}")

print("\n" + "="*80)
print("ISSUE:")
print("Upper stops should only collide at +25°, not at negative angles")
print("The upper stop position might be interfering with finger mesh path")
print("="*80)
