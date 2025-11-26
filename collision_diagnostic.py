#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("=" * 70)
print("COMPREHENSIVE COLLISION DIAGNOSTIC")
print("=" * 70)

# 1. Check collision matrix
print("\n1. COLLISION MATRIX ANALYSIS:")
for i in range(model.ngeom):
    name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name1:
        for j in range(model.ngeom):
            name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, j)
            if name2 and i < j:  # Only check each pair once
                contype1 = model.geom_contype[i]
                conaffinity1 = model.geom_conaffinity[i]
                contype2 = model.geom_contype[j]
                conaffinity2 = model.geom_conaffinity[j]
                
                # Check if they can collide
                can_collide = (contype1 & conaffinity2) or (contype2 & conaffinity1)
                if can_collide:
                    print(f"  ✓ {name1} ↔ {name2}: contype={contype1}/{contype2}, conaffinity={conaffinity1}/{conaffinity2}")

# 2. Check geom properties
print("\n2. GEOM PROPERTIES:")
for i in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name and ('palm' in name.lower() or 'f1' in name.lower() or 'f2' in name.lower()):
        print(f"  {name}:")
        print(f"    Type: {model.geom_type[i]}")
        print(f"    Contype: {model.geom_contype[i]}, Conaffinity: {model.geom_conaffinity[i]}")
        print(f"    Pos: ({model.geom_pos[i,0]:.3f}, {model.geom_pos[i,1]:.3f}, {model.geom_pos[i,2]:.3f})")
        print(f"    Size: {model.geom_size[i,0]:.3f}, {model.geom_size[i,1]:.3f}, {model.geom_size[i,2]:.3f}")
        print(f"    Body: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[i])}")

# 3. Check contact exclusions
print("\n3. CONTACT EXCLUSIONS:")
print(f"  Number of exclusions: {model.nexclude}")
print("  (Skipping detailed exclude analysis due to API issues)")

# 4. Check body hierarchy
print("\n4. BODY HIERARCHY:")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and ('palm' in name.lower() or 'f1' in name.lower() or 'f2' in name.lower()):
        parent_id = model.body_parentid[i]
        parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id) if parent_id >= 0 else "World"
        print(f"  {name} (parent: {parent_name})")

# 5. Run simulation and force contact detection
print("\n5. FORCED CONTACT DETECTION:")
mujoco.mj_resetDataKeyframe(model, data, 0)

for step in range(100):
    mujoco.mj_step(model, data)
    
    # Force collision detection
    mujoco.mj_collision(model, data)
    
    if data.ncon > 0:
        print(f"  Step {step}: {data.ncon} contacts found")
        for i in range(min(3, data.ncon)):  # Show first 3 contacts
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            print(f"    Contact {i}: {geom1_name} {geom2_name}")
            print(f"      Distance: {contact.dist:.6f}m, Pos: ({contact.pos[0]:.3f}, {contact.pos[1]:.3f}, {contact.pos[2]:.3f})")
        break
else:
    print("  No contacts detected after 100 steps")

# 6. Check solver settings
print("\n6. SOLVER SETTINGS:")
print(f"  nSolverIterations: {model.opt.ls_iterations}")
print("  (Other solver settings skipped due to API issues)")

print("\n" + "=" * 70)
print("Diagnostic complete!")
print("=" * 70)
