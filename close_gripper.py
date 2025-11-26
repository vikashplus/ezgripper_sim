#!/usr/bin/env python3
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Initialize viewer
try:
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    print("✅ MuJoCo viewer launched!")
    print("🎯 Closing gripper simulation")
    print("👀 Watch the gripper close with mesh collisions")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    exit(1)

print("\n" + "=" * 60)
print("CLOSING GRIPPER SIMULATION")
print("=" * 60)

# Start from current state (spring equilibrium)
mujoco.mj_resetDataKeyframe(model, data, 0)

# Let it settle first
print("Letting gripper settle to spring equilibrium...")
for step in range(300):
    mujoco.mj_step(model, data)
    viewer.sync()

print("Now closing gripper...")

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

# Close gripper gradually
step = 0
while viewer.is_running() and step < 1000:
    # Calculate target for closed position
    if step < 500:
        # Gradually close
        progress = step / 500.0
        # Palm joints: move toward closed position (-1.57)
        for i, name in palm_joints:
            current = data.qpos[i]
            target = -1.57
            data.qpos[i] = current + (target - current) * 0.02  # Smooth transition
        
        # Knuckle joints: move toward closed position (0.0)
        for i, name in knuckle_joints:
            current = data.qpos[i]
            target = 0.0
            data.qpos[i] = current + (target - current) * 0.02  # Smooth transition
    else:
        # Hold closed position
        pass
    
    # Step physics
    mujoco.mj_step(model, data)
    
    # Print status every 100 steps
    if step % 100 == 0:
        print(f"\nStep {step:3d} (t={data.time:.2f}s):")
        
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
                print(f"    {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
        
        # Check if fully closed
        if step >= 500:
            closed = True
            for i, name in palm_joints:
                if abs(data.qpos[i] - (-1.57)) > 0.1:
                    closed = False
                    break
            for i, name in knuckle_joints:
                if abs(data.qpos[i] - 0.0) > 0.1:
                    closed = False
                    break
            if closed:
                print("  ✅ Gripper is closed!")
    
    # Sync viewer
    viewer.sync()
    step += 1

print("\n🎬 Closing simulation complete!")
print("=" * 60)
