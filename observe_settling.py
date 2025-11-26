#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Apply keyframe for stable start
mujoco.mj_resetDataKeyframe(model, data, 0)

print("=" * 60)
print("GRIPPER SETTLING SIMULATION")
print("=" * 60)

# Run simulation and observe settling behavior
for step in range(1000):
    mujoco.mj_step(model, data)
    
    # Check collisions every 100 steps
    if step % 100 == 0:
        print(f"\nStep {step:3d} (t={data.time:.3f}s):")
        
        # Get current joint angles
        angles = []
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                angle_deg = np.degrees(data.qpos[i])
                angles.append(f"{name}: {angle_deg:6.1f}°")
        
        print(f"  Joint angles: {', '.join(angles)}")
        
        # Check for contacts
        ncon = data.ncon
        if ncon > 0:
            print(f"  COLLISIONS: {ncon} contacts")
            for i in range(min(3, ncon)):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                dist = contact.dist
                print(f"    Contact {i}: {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}")
                print(f"      Distance: {dist:.6f}m")
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
print("Simulation complete!")
print("=" * 60)
