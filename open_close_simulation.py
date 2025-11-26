#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("=" * 60)
print("GRIPPER OPEN/CLOSE SIMULATION")
print("=" * 60)

# Find joint indices
palm_joints = []
knuckle_joints = []
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        if 'palm_knuckle' in name:
            palm_joints.append((i, name))
        elif 'knuckle_tip' in name:
            knuckle_joints.append((i, name))

print(f"Found palm joints: {[name for _, name in palm_joints]}")
print(f"Found knuckle joints: {[name for _, name in knuckle_joints]}")

# Control function
def set_gripper_position(open_fraction):
    """Set gripper position: 0.0 = fully closed, 1.0 = fully open"""
    # Palm joints: -1.57 (closed) to 0.35 (open)
    palm_closed, palm_open = -1.57, 0.35
    palm_target = palm_closed + open_fraction * (palm_open - palm_closed)
    
    # Knuckle joints: 0 (closed) to 1.7 (open)  
    knuckle_closed, knuckle_open = 0, 1.7
    knuckle_target = knuckle_closed + open_fraction * (knuckle_open - knuckle_closed)
    
    # Set positions
    for i, name in palm_joints:
        data.qpos[i] = palm_target
    for i, name in knuckle_joints:
        data.qpos[i] = knuckle_target

# Test sequence: closed -> open -> closed
positions = [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0]

for step_idx, open_fraction in enumerate(positions):
    print(f"\n{'='*40}")
    print(f"Position {step_idx+1}/{len(positions)}: {open_fraction*100:.0f}% open")
    print(f"{'='*40}")
    
    # Set target position
    set_gripper_position(open_fraction)
    
    # Run simulation to settle
    for sim_step in range(200):
        mujoco.mj_step(model, data)
        
        # Check collisions at key points
        if sim_step % 50 == 0:
            ncon = data.ncon
            if ncon > 0:
                print(f"  Step {sim_step}: {ncon} contacts")
                for i in range(min(2, ncon)):
                    contact = data.contact[i]
                    geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                    geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                    dist = contact.dist
                    print(f"    Contact {i}: {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
    
    # Report final state
    print(f"\nFinal state at {open_fraction*100:.0f}% open:")
    angles = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            angle_deg = np.degrees(data.qpos[i])
            angles.append(f"{name}: {angle_deg:6.1f}°")
    print(f"  Joint angles: {', '.join(angles)}")
    
    # Check collisions
    ncon = data.ncon
    if ncon > 0:
        print(f"  COLLISIONS: {ncon} contacts")
        for i in range(ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            dist = contact.dist
            print(f"    Contact {i}: {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
    else:
        print(f"  No collisions")
    
    # Check joint limit violations
    violations = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            qpos = data.qpos[i]
            range_min, range_max = model.jnt_range[i]
            if qpos < range_min or qpos > range_max:
                violations.append(f"{name}: {np.degrees(qpos):.1f}°")
    
    if violations:
        print(f"  VIOLATIONS: {', '.join(violations)}")
    else:
        print(f"  All joints within limits")

print("\n" + "=" * 60)
print("Open/Close simulation complete!")
print("=" * 60)
