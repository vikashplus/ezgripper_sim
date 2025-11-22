#!/usr/bin/env python3
"""
Tennis Ball Grasp Demo for EZGripper

Demonstrates under-actuated grasping:
1. Gripper starts open above ball
2. Close gripper with moderate effort
3. L1 contacts ball first → stops
4. Tendon force overcomes L2 spring → L2 wraps
5. Adaptive conforming to spherical object
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("=" * 60)
print("EZGripper Tennis Ball Grasp Demo")
print("=" * 60)
print(f"Model: {model_path}")
print(f"MuJoCo version: {mujoco.__version__}")
print()

# Load the model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("✓ Model loaded successfully!")
print(f"  - Bodies: {model.nbody}")
print(f"  - Joints: {model.njnt}")
print(f"  - Actuators: {model.nu}")
print(f"  - Tendons: {model.ntendon}")
print()

# Get actuator ID
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')

# Demo sequence
print("Demo Sequence:")
print("  1. Gripper starts OPEN (springs extended)")
print("  2. Wait 2 seconds")
print("  3. CLOSE gripper slowly (watch L1 and L2 behavior)")
print("  4. L1 contacts ball → stops")
print("  5. L2 wraps around ball (tendon overcomes spring)")
print("  6. Hold grasp for 3 seconds")
print("  7. OPEN gripper (springs return)")
print()
print("Controls:")
print("  - Space: Pause/Resume")
print("  - ESC: Exit")
print("  - Double-click: Select body")
print("  - Right-drag: Rotate view")
print()
print("Starting in 2 seconds...")
time.sleep(2)

# Simulation parameters
sim_time = 0.0
phase = "open"
phase_start_time = 0.0

def update_control(sim_time):
    """Update gripper control based on demo phase"""
    global phase, phase_start_time
    
    elapsed = sim_time - phase_start_time
    
    if phase == "open":
        # Phase 1: Gripper open (2 seconds)
        data.ctrl[gripper_actuator_id] = 0.0  # Fully open
        if elapsed > 2.0:
            phase = "closing"
            phase_start_time = sim_time
            print(f"\n[{sim_time:.2f}s] CLOSING gripper...")
    
    elif phase == "closing":
        # Phase 2: Close gripper slowly (3 seconds)
        # Gradually increase closing force
        progress = min(elapsed / 3.0, 1.0)
        data.ctrl[gripper_actuator_id] = -1.0 * progress  # -1.0 = MAXIMUM effort
        
        if elapsed > 3.0:
            phase = "holding"
            phase_start_time = sim_time
            print(f"[{sim_time:.2f}s] HOLDING grasp at MAXIMUM strength...")
            print("  → Watch: L1 stopped by ball, L2 wrapped around!")
    
    elif phase == "holding":
        # Phase 3: Hold grasp (3 seconds)
        data.ctrl[gripper_actuator_id] = -1.0  # MAXIMUM grasp strength
        if elapsed > 3.0:
            phase = "opening"
            phase_start_time = sim_time
            print(f"\n[{sim_time:.2f}s] OPENING gripper...")
    
    elif phase == "opening":
        # Phase 4: Open gripper (2 seconds)
        data.ctrl[gripper_actuator_id] = 0.0  # Release
        if elapsed > 2.0:
            phase = "done"
            phase_start_time = sim_time
            print(f"[{sim_time:.2f}s] Demo complete!")
            print("\nYou can continue to interact with the simulation.")
            print("Close the viewer window to exit.")
    
    elif phase == "done":
        # Demo finished, keep gripper open
        data.ctrl[gripper_actuator_id] = 0.0

# Launch viewer with custom controller
print("\n" + "=" * 60)
print("Launching viewer...")
print("=" * 60 + "\n")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Initial setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Run simulation
    while viewer.is_running():
        step_start = time.time()
        
        # Update control
        update_control(sim_time)
        
        # Step simulation
        mujoco.mj_step(model, data)
        sim_time += model.opt.timestep
        
        # Update viewer
        viewer.sync()
        
        # Maintain real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nDemo finished!")
