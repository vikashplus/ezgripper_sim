#!/usr/bin/env python3
"""
Visualize mechanical stop positions in MuJoCo simulation window.
Shows current (wrong) vs correct positions with colored markers.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP VISUALIZATION IN SIMULATION")
print("="*80)
print("\nColors:")
print("  RED spheres: Current (wrong) stop positions")
print("  GREEN spheres: Correct stop positions")
print("  BLUE spheres: Palm stop positions")
print("\nClose window to exit")

# Calculate correct positions
f1_palm_id = model.joint('F1_palm_knuckle').id
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

# Calculate correct positions
data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)
finger_transform_lower = data.xpos[model.body('F1_L1').id]
correct_lower_relative = palm_lower_stop_pos - finger_transform_lower

data.qpos[f1_palm_id] = np.radians(25)
mujoco.mj_kinematics(model, data)
finger_transform_upper = data.xpos[model.body('F1_L1').id]
correct_upper_relative = palm_upper_stop_pos - finger_transform_upper

# Add visualization geoms for current vs correct positions
def add_visualization_markers():
    """Add colored spheres to show stop positions"""
    
    # Current positions (RED)
    # Lower stop current position
    current_lower_world = finger_transform_lower + np.array([0.01, 0, 0])
    # Upper stop current position  
    current_upper_world = finger_transform_upper + np.array([-0.01, 0, 0])
    
    # Correct positions (GREEN)
    correct_lower_world = finger_transform_lower + correct_lower_relative
    correct_upper_world = finger_transform_upper + correct_upper_relative
    
    return {
        'current_lower': current_lower_world,
        'current_upper': current_upper_world,
        'correct_lower': correct_lower_world,
        'correct_upper': correct_upper_world,
        'palm_lower': palm_lower_stop_pos,
        'palm_upper': palm_upper_stop_pos
    }

# Get marker positions
markers = add_visualization_markers()

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.3
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.08, 0.03, 0.05])

    step = 0
    angle = -90
    
    while viewer.is_running():
        # Cycle through angles to show stops at different positions
        if step % 100 == 0:
            angle = -90 if angle == 25 else 25
            data.qpos[f1_palm_id] = np.radians(angle)
            
            # Recalculate positions for current angle
            if angle == -90:
                finger_transform = data.xpos[model.body('F1_L1').id]
                markers['current_lower'] = finger_transform + np.array([0.01, 0, 0])
                markers['correct_lower'] = finger_transform + correct_lower_relative
            else:
                finger_transform = data.xpos[model.body('F1_L1').id]
                markers['current_upper'] = finger_transform + np.array([-0.01, 0, 0])
                markers['correct_upper'] = finger_transform + correct_upper_relative
        
        # Forward kinematics
        mujoco.mj_forward(model, data)
        
        # Draw custom markers using viewer overlay
        scene = viewer.opt
        
        # Print status every 50 steps
        if step % 50 == 0:
            print(f"\n[{step:4d}] Angle: {angle:3d}°")
            print(f"  Palm lower stop: [{markers['palm_lower'][0]:.3f}, {markers['palm_lower'][1]:.3f}, {markers['palm_lower'][2]:.3f}]")
            print(f"  Current lower:   [{markers['current_lower'][0]:.3f}, {markers['current_lower'][1]:.3f}, {markers['current_lower'][2]:.3f}]")
            print(f"  Correct lower:   [{markers['correct_lower'][0]:.3f}, {markers['correct_lower'][1]:.3f}, {markers['correct_lower'][2]:.3f}]")
            
            # Calculate distances
            dist_current = np.linalg.norm(markers['current_lower'] - markers['palm_lower'])
            dist_correct = np.linalg.norm(markers['correct_lower'] - markers['palm_lower'])
            print(f"  Distance current: {dist_current:.4f} (collides: {dist_current < 0.01})")
            print(f"  Distance correct: {dist_correct:.4f} (collides: {dist_correct < 0.01})")
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()
        
        step += 1

print("\nVisualization ended.")
