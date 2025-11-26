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
    print("🎯 Tendon control with reduced springs")
    print("👀 Watch fingers close and touch via tendon actuation")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    exit(1)

print("\n" + "=" * 60)
print("TENDON CONTROL - REDUCED SPRINGS FOR MOVEMENT")
print("=" * 60)

# Store original springrefs
original_springrefs = {}
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        original_springrefs[i] = model.jnt_springref[i]

print("Reducing springrefs to enable tendon control...")
# Reduce springrefs temporarily to allow movement
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        model.jnt_springref[i] *= 0.1  # Reduce to 10% of original

# Start from keyframe (open position)
mujoco.mj_resetDataKeyframe(model, data, 0)

print("Starting from open position...")
print("Pulling tendons to close fingers until they touch...")

# Find tendon actuators
f1_actuator = 0  # gripper_actuator_f1
f2_actuator = 1  # gripper_actuator_f2

# Simulation: gradually close fingers
step = 0
max_steps = 1500

while viewer.is_running() and step < max_steps:
    # Calculate tendon position (shorter = more closed)
    if step < 600:
        # Phase 1: Gradually close
        progress = step / 600.0
        target_length = 0.168 - progress * 0.040  # Close more aggressively
    elif step < 1000:
        # Phase 2: Hold closed position
        target_length = 0.128
    else:
        # Phase 3: Open slightly
        progress = (step - 1000) / 500.0
        target_length = 0.128 + progress * 0.020
    
    # Set tendon position control
    data.ctrl[f1_actuator] = target_length
    data.ctrl[f2_actuator] = target_length
    
    # Step physics
    mujoco.mj_step(model, data)
    
    # Print status every 100 steps
    if step % 100 == 0:
        print(f"\nStep {step:3d} (t={data.time:.2f}s):")
        print(f"  Target tendon length: {target_length:.6f}m")
        
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
            for i in range(min(8, ncon)):
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
                print(f"    🎯 Fingertips touching! ({fingertip_contacts} contacts)")
            if l1_l2_contacts > 0:
                print(f"    🔗 L1-L2 segments touching! ({l1_l2_contacts} contacts)")
        elif step > 400:
            print(f"  ⚠️  Fingers not touching yet")
    
    # Sync viewer
    viewer.sync()
    step += 1

# Restore original springrefs
print("\nRestoring original springrefs...")
for i, original_value in original_springrefs.items():
    model.jnt_springref[i] = original_value

print("\n🎬 Tendon control simulation complete!")
print("=" * 60)
