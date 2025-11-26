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
    print("🎯 Tendon-based gripper closing simulation")
    print("👀 Watch the gripper close via tendon actuation")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    exit(1)

print("\n" + "=" * 60)
print("TENDON-ACTUATED GRIPPER CLOSING")
print("=" * 60)

# Start from keyframe (open position)
mujoco.mj_resetDataKeyframe(model, data, 0)

print("Starting from open position...")
print("Pulling tendon to close gripper...")

# Find tendon actuator
tendon_actuator = None
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if name and 'tendon' in name.lower():
        tendon_actuator = i
        print(f"Found tendon actuator: {name}")
        break

if tendon_actuator is None:
    print("❌ No tendon actuator found!")
    exit(1)

# Simulation parameters
step = 0
max_force = 50.0  # Maximum tendon force
ramp_time = 500   # Steps to ramp up force

while viewer.is_running() and step < 1500:
    # Calculate tendon force
    if step < ramp_time:
        # Gradually increase force
        force = (step / ramp_time) * max_force
    elif step < 1000:
        # Hold maximum force
        force = max_force
    else:
        # Gradually release force
        release_progress = (step - 1000) / 500.0
        force = max_force * (1.0 - release_progress)
    
    # Apply tendon force
    data.ctrl[tendon_actuator] = force
    
    # Step physics
    mujoco.mj_step(model, data)
    
    # Print status every 100 steps
    if step % 100 == 0:
        print(f"\nStep {step:3d} (t={data.time:.2f}s):")
        print(f"  Tendon force: {force:.1f}N")
        
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
            for i in range(min(4, ncon)):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                dist = contact.dist
                print(f"    {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
        else:
            print(f"  No collisions")
        
        # Check tendon length
        for i in range(model.ntendon):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
            if name and 'finger' in name.lower():
                length = data.ten_length[i]
                print(f"  {name} length: {length:.6f}m")
        
        # Check if fingers are touching
        if ncon > 2:
            print("  ✅ Fingers are touching!")
        elif step > 800 and ncon <= 2:
            print("  ⚠️  Fingers not touching yet")
    
    # Sync viewer
    viewer.sync()
    step += 1

print("\n🎬 Tendon actuation simulation complete!")
print("=" * 60)
