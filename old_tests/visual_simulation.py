#!/usr/bin/env python3
import mujoco
import numpy as np
import time

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Initialize viewer
try:
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    print("✅ MuJoCo viewer launched successfully!")
    print("🎯 Watch the gripper settle with mesh collisions")
    print("⚡ Heavy damping (0.1) provides smooth, realistic motion")
    print("🔧 Springrefs create realistic preload")
    print("🚫 Mesh collisions provide physical hard stops")
except Exception as e:
    print(f"❌ Could not launch viewer: {e}")
    print("💡 Make sure you have a display available")
    exit(1)

# Apply keyframe for stable start
mujoco.mj_resetDataKeyframe(model, data, 0)

print("\n" + "=" * 60)
print("VISUAL GRIPPER SIMULATION")
print("=" * 60)
print("Watch the gripper fingers settle with realistic physics!")
print("Mesh collisions (palm ↔ fingers) provide the hard stops.")
print("\nClose the viewer window to stop the simulation.")

# Run visual simulation
step = 0
while viewer.is_running():
    # Step physics
    mujoco.mj_step(model, data)
    
    # Print status every 500 steps
    if step % 500 == 0:
        print(f"\nStep {step:4d} (t={data.time:.2f}s):")
        
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
            print(f"  COLLISIONS: {ncon} contacts active")
            for i in range(min(2, ncon)):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                dist = contact.dist
                print(f"    {geom1_name if geom1_name else 'Palm'} ↔ {geom2_name}: {dist:.6f}m")
        
        # Check if settled
        if step > 1000 and data.ncon > 0:
            settled = True
            for i in range(model.njnt):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if name and 'palm_knuckle' in name:
                    if abs(data.qpos[i] - (-1.72)) > 0.01:  # Check if near equilibrium
                        settled = False
                        break
            if settled:
                print("  ✅ Gripper has settled to equilibrium!")
    
    # Sync viewer
    viewer.sync()
    step += 1
    
    # Small delay for real-time viewing
    time.sleep(0.001)

print("\n🎬 Simulation ended!")
print("=" * 60)
