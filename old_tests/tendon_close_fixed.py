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
    print("🔧 Actuating both finger tendons simultaneously")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    exit(1)

print("\n" + "=" * 60)
print("TENDON-ACTUATED GRIPPER CLOSING")
print("=" * 60)

# Start from keyframe (open position)
mujoco.mj_resetDataKeyframe(model, data, 0)

print("Starting from open position...")
print("Pulling tendons to close gripper...")

# Find tendon actuators
f1_actuator = 0  # gripper_actuator_f1
f2_actuator = 1  # gripper_actuator_f2

print(f"Using actuators: F1={f1_actuator}, F2={f2_actuator}")

# Simulation parameters
step = 0
max_force = 100.0  # Maximum tendon force
ramp_time = 300   # Steps to ramp up force
hold_time = 700   # Steps to hold maximum force

while viewer.is_running() and step < 1500:
    # Calculate tendon force
    if step < ramp_time:
        # Gradually increase force
        force = (step / ramp_time) * max_force
    elif step < ramp_time + hold_time:
        # Hold maximum force
        force = max_force
    else:
        # Gradually release force
        release_progress = (step - ramp_time - hold_time) / 500.0
        force = max_force * max(0, 1.0 - release_progress)
    
    # Apply tendon forces to both fingers
    data.ctrl[f1_actuator] = force
    data.ctrl[f2_actuator] = force
    
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
            for i in range(min(6, ncon)):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                dist = contact.dist
                print(f"    {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
        else:
            print(f"  No collisions")
        
        # Check tendon lengths
        for i in range(model.ntendon):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
            if name:
                length = data.ten_length[i]
                print(f"  {name} length: {length:.6f}m")
        
        # Check if fingertips are touching
        fingertip_contacts = 0
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if (geom1_name and 'tip' in geom1_name) or (geom2_name and 'tip' in geom2_name):
                fingertip_contacts += 1
        
        if fingertip_contacts > 0:
            print(f"  ✅ Fingertips touching! ({fingertip_contacts} fingertip contacts)")
        elif step > 600 and ncon > 2:
            print("  ✅ Fingers colliding (not fingertips)")
        elif step > 800:
            print("  ⚠️  Fingers not touching yet")
    
    # Sync viewer
    viewer.sync()
    step += 1

print("\n🎬 Tendon actuation simulation complete!")
print("=" * 60)
