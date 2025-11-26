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
    print("🎯 CORRECT tendon range: 160mm → 100mm")
    print("👀 Watch fingers close with proper tendon actuation")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    exit(1)

print("\n" + "=" * 60)
print("CORRECT TENDON RANGE - 160mm to 100mm")
print("=" * 60)

# Start from keyframe (open position)
mujoco.mj_resetDataKeyframe(model, data, 0)

print("Starting from open position...")
print("Pulling tendons from 160mm (open) to 100mm (closed)...")

# Find tendon actuators
f1_actuator = 0  # gripper_actuator_f1
f2_actuator = 1  # gripper_actuator_f2

# Simulation: proper tendon range
step = 0
max_steps = 2000

while viewer.is_running() and step < max_steps:
    # Calculate tendon position with CORRECT range
    if step < 600:
        # Phase 1: Close from 160mm to 100mm
        progress = step / 600.0
        target_length = 0.160 - progress * 0.060  # 160mm → 100mm
    elif step < 1200:
        # Phase 2: Hold closed at 100mm
        target_length = 0.100
    else:
        # Phase 3: Open from 100mm to 160mm
        progress = (step - 1200) / 800.0
        target_length = 0.100 + progress * 0.060  # 100mm → 160mm
    
    # Set tendon position control
    data.ctrl[f1_actuator] = target_length
    data.ctrl[f2_actuator] = target_length
    
    # Step physics
    mujoco.mj_step(model, data)
    
    # Print status every 100 steps
    if step % 100 == 0:
        print(f"\nStep {step:3d} (t={data.time:.2f}s):")
        print(f"  Target tendon length: {target_length*1000:.1f}mm")
        
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
            for i in range(min(12, ncon)):
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
                print(f"  {name} actual length: {length*1000:.1f}mm")
        
        # Check for finger-to-finger contacts
        finger_contacts = 0
        fingertip_contacts = 0
        l1_l2_contacts = 0
        
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            # Check for finger-to-finger contacts (not palm)
            if geom1_name and geom2_name:
                if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                    finger_contacts += 1
                    if 'tip' in geom1_name or 'tip' in geom2_name:
                        fingertip_contacts += 1
                    if ('l1' in geom1_name and 'l2' in geom2_name) or ('l2' in geom1_name and 'l1' in geom2_name):
                        l1_l2_contacts += 1
        
        if finger_contacts > 0:
            print(f"  ✅ FINGERS TOUCHING! ({finger_contacts} finger-finger contacts)")
            if fingertip_contacts > 0:
                print(f"    🎯 FINGERTIPS TOUCHING! ({fingertip_contacts} contacts)")
            if l1_l2_contacts > 0:
                print(f"    🔗 L1-L2 SEGMENTS TOUCHING! ({l1_l2_contacts} contacts)")
    
    # Sync viewer
    viewer.sync()
    step += 1

print("\n🎬 Correct tendon range simulation complete!")
print("=" * 60)
