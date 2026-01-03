#!/usr/bin/env python3
"""
Debug script to monitor joint positions and spring forces in real-time
"""
import mujoco
import numpy as np
import time

# Load model
model = mujoco.MjModel.from_xml_path('demo_tennis_ball.xml')
data = mujoco.MjData(model)

# Set initial positions to valid positions within ranges
data.qpos[0] = -0.52  # F1_palm_knuckle: -30° (within -60° to 15.5° range)
data.qpos[1] = 0.0    # F1_knuckle_tip: 0° (within 0° to 97.4° range)
data.qpos[2] = -0.52  # F2_palm_knuckle: -30° (within -60° to 15.5° range)
data.qpos[3] = 0.0    # F2_knuckle_tip: 0° (within 0° to 97.4° range)

mujoco.mj_forward(model, data)  # Update positions

# Get joint and actuator IDs
joints = {
    'F1_palm_knuckle': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle'),
    'F1_knuckle_tip': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip'),
}
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')

def print_joint_status():
    print(f"\n{'='*60}")
    print("JOINT STATUS:")
    print(f"{'='*60}")
    
    for name, jid in joints.items():
        pos = data.qpos[jid]
        vel = data.qvel[jid]
        force = data.qfrc_spring[jid]  # Spring force
        range_min, range_max = model.jnt_range[jid]
        
        print(f"{name}:")
        print(f"  Position: {np.degrees(pos):6.1f}° (Range: {np.degrees(range_min):.1f} to {np.degrees(range_max):.1f}°)")
        print(f"  Velocity: {np.degrees(vel):6.1f}°/s")
        print(f"  Spring Force: {force:8.3f} N·m")
        print(f"  Stiffness: {model.jnt_stiffness[jid]:.6f}")
        # Note: SpringRef not directly accessible, check XML for values
    
    # Actuator info
    print(f"\nTendon Actuator:")
    print(f"  Control: {data.ctrl[actuator_id]:.3f}")

# Test sequence
print("Starting joint debugging...")
print("Testing progressive wrap behavior...")

for step in range(100):
    # Gradually close gripper - single tendon control
    control = -0.5 * (step / 100.0)  # Gradual closing
    data.ctrl[actuator_id] = control
    
    mujoco.mj_step(model, data)
    
    if step % 10 == 0:  # Print every 10 steps
        print_joint_status()
    
    time.sleep(0.02)

print("\nDebug complete!")
