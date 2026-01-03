#!/usr/bin/env python3
"""
Test undriven gripper behavior - no tendon actuation, just springs and gravity.
The gripper should settle to its natural open position driven by springs.
"""

import mujoco
import mujoco.viewer
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("UNDRIVEN GRIPPER TEST - No Tendon Actuation")
print("="*80)
print("\nSpring Configuration:")
print(f"  F1_palm_knuckle: stiffness={model.jnt_stiffness[0]:.6f}, springref={model.qpos_spring[0]:.4f} rad ({np.rad2deg(model.qpos_spring[0]):.1f}°)")
print(f"  F1_knuckle_tip:  stiffness={model.jnt_stiffness[1]:.6f}, springref={model.qpos_spring[1]:.4f} rad ({np.rad2deg(model.qpos_spring[1]):.1f}°)")

print("\nJoint Ranges:")
print(f"  F1_palm_knuckle: {np.rad2deg(model.jnt_range[0,0]):.1f}° to {np.rad2deg(model.jnt_range[0,1]):.1f}°")
print(f"  F1_knuckle_tip:  {np.rad2deg(model.jnt_range[1,0]):.1f}° to {np.rad2deg(model.jnt_range[1,1]):.1f}°")

print("\n" + "="*80)
print("Expected: Springs should push fingers to open position (~-90° for Palm-L1)")
print("="*80)

# Set actuator controls to 163mm (at max path length of 162.86mm)
data.ctrl[:] = 0.163  # 163mm in meters

# Run simulation with viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        # Step simulation
        mujoco.mj_step(model, data)
        step += 1
        
        # Print status every 0.5 seconds
        if step % 500 == 0:
            time = data.time
            
            # Get joint angles
            f1_palm = data.qpos[0]
            f1_tip = data.qpos[1]
            f2_palm = data.qpos[2]
            f2_tip = data.qpos[3]
            
            # Get joint velocities
            f1_palm_vel = data.qvel[0]
            f1_tip_vel = data.qvel[1]
            
            # Get spring torques (passive forces)
            f1_palm_torque = -model.jnt_stiffness[0] * (f1_palm - model.qpos_spring[0])
            f1_tip_torque = -model.jnt_stiffness[1] * (f1_tip - model.qpos_spring[1])
            
            # Get tendon lengths
            mujoco.mj_tendon(model, data)
            tendon1_length = data.ten_length[0] * 1000  # Convert to mm
            tendon2_length = data.ten_length[1] * 1000
            
            print(f"[{time:5.2f}s]")
            print(f"  F1_palm: {np.rad2deg(f1_palm):6.1f}° | torque: {f1_palm_torque:+7.4f} N·m | vel: {f1_palm_vel:+7.4f} rad/s")
            print(f"  F1_tip:  {np.rad2deg(f1_tip):6.1f}° | torque: {f1_tip_torque:+7.4f} N·m | vel: {f1_tip_vel:+7.4f} rad/s")
            print(f"  F2_palm: {np.rad2deg(f2_palm):6.1f}° | F2_tip: {np.rad2deg(f2_tip):6.1f}°")
            print(f"  Tendon: F1={tendon1_length:.2f}mm, F2={tendon2_length:.2f}mm")
            print()
            
            # Check if settled (low velocity)
            if abs(f1_palm_vel) < 0.001 and abs(f1_tip_vel) < 0.001 and time > 5.0:
                print("="*80)
                print("STEADY STATE REACHED:")
                print(f"  F1_palm: {np.rad2deg(f1_palm):.2f}° (range: {np.rad2deg(model.jnt_range[0,0]):.0f}° to {np.rad2deg(model.jnt_range[0,1]):.0f}°)")
                print(f"  F1_tip:  {np.rad2deg(f1_tip):.2f}° (range: {np.rad2deg(model.jnt_range[1,0]):.0f}° to {np.rad2deg(model.jnt_range[1,1]):.0f}°)")
                print(f"  Spring torques: F1_palm={f1_palm_torque:.4f} N·m, F1_tip={f1_tip_torque:.4f} N·m")
                
                # Check for limit violations
                if f1_palm < model.jnt_range[0,0] or f1_palm > model.jnt_range[0,1]:
                    print(f"  ⚠️  WARNING: F1_palm outside limits!")
                if f1_tip < model.jnt_range[1,0] or f1_tip > model.jnt_range[1,1]:
                    print(f"  ⚠️  WARNING: F1_tip outside limits!")
                print("="*80)
                break
        
        # Update viewer
        viewer.sync()

print("\nTest finished!")
