#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Apply keyframe for stable start
mujoco.mj_resetDataKeyframe(model, data, 0)

print("=" * 70)
print("COLLISION ANALYSIS - PALM-L1 AND L1-L2 OVER-ROTATION")
print("=" * 70)

# Run simulation and check for collisions
for step in range(500):
    mujoco.mj_step(model, data)
    
    # Check collisions every 50 steps
    if step % 50 == 0:
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
            print(f"  COLLISIONS DETECTED ({ncon} contacts):")
            for i in range(ncon):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                dist = contact.dist
                # Use contact.efc_force for force if available, otherwise skip force display
                print(f"    Contact {i}: {geom1_name} ↔ {geom2_name}")
                print(f"      Distance: {dist:.6f}m")
        else:
            print(f"  No collisions detected")
        
        # Check if joints are beyond limits
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                qpos = data.qpos[i]
                range_min, range_max = model.jnt_range[i]
                if qpos < range_min or qpos > range_max:
                    print(f"  VIOLATION: {name} beyond limits!")
                    print(f"    Position: {np.degrees(qpos):.1f}°, Range: [{np.degrees(range_min):.1f}°, {np.degrees(range_max):.1f}°]")

print("\n" + "=" * 70)
print("Collision analysis complete!")
print("=" * 70)
