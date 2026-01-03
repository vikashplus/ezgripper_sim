#!/usr/bin/env python3
"""
Simple visualization of force control gripper physics.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("SIMPLE FORCE CONTROL VISUALIZATION")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.35
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -25
    viewer.cam.lookat = np.array([0.1, 0, 0.05])

    print("Viewer launched. Close window to exit.")
    print("\nControl sequence:")
    print("1. OPEN (-150N force)")
    print("2. CLOSE (+100N force)")
    print("3. STOP (0N force)")
    
    step = 0
    phase = "OPEN"
    
    while viewer.is_running():
        # Phase 1: Open (steps 0-200)
        if step < 200:
            phase = "OPEN"
            force = -150.0
            
        # Phase 2: Close (steps 200-500)
        elif step < 500:
            phase = "CLOSE"
            force = 100.0
            
        # Phase 3: Stop (steps 500+)
        else:
            phase = "STOP"
            force = 0.0
        
        # Apply force control
        data.ctrl[f1_actuator_id] = force
        data.ctrl[f2_actuator_id] = force
        
        # Get current state
        f1_palm = np.degrees(data.qpos[f1_palm_id])
        f1_tip = np.degrees(data.qpos[f1_tip_id])
        f2_palm = np.degrees(data.qpos[f2_palm_id])
        f2_tip = np.degrees(data.qpos[f2_tip_id])
        
        # Check contacts
        contacts = data.ncon
        finger_contacts = 0
        if contacts > 0:
            for i in range(contacts):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                
                if ('f1' in geom1_name and 'f2' in geom2_name) or ('f2' in geom1_name and 'f1' in geom2_name):
                    finger_contacts += 1
        
        contact_status = f"CONTACT: {finger_contacts}" if finger_contacts > 0 else "NO CONTACT"
        
        # Print status every 50 steps
        if step % 50 == 0:
            print(f"[Step {step:4d}] {phase:5s} | Force={force:6.1f}N | "
                  f"F1({f1_palm:5.1f}°,{f1_tip:5.1f}°) F2({f2_palm:5.1f}°,{f2_tip:5.1f}°) | "
                  f"{contact_status}")
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()
        
        step += 1

print("\nVisualization ended.")
