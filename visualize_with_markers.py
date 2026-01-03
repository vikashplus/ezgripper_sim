#!/usr/bin/env python3
"""
Visualize mechanical stop positions with colored markers in simulation.
"""

import mujoco
import numpy as np
import os

# Try different viewer import patterns
try:
    import mujoco.viewer as viewer
except ImportError:
    try:
        from mujoco import viewer
    except ImportError:
        try:
            import mujoco_py.viewer as viewer
        except ImportError:
            print("MuJoCo viewer not available. Using basic rendering...")
            viewer = None

# Load visualization model with markers
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_visualization.xml')
model = mujoco.MjModel.from_xml_path(viz_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP VISUALIZATION WITH MARKERS")
print("="*80)
print("\nMarker colors:")
print("  BLUE spheres: Palm stop positions (fixed at Z=0)")
print("  RED spheres: Current (wrong) finger stop positions (at Z=0.1)")
print("  GREEN spheres: Correct finger stop positions (at Z=0)")
print("\nThe RED and BLUE spheres should overlap for correct collision.")
print("Currently they're 100mm apart vertically!")
print("\nControls:")
print("  - Close window to exit")
print("  - Watch as finger cycles between -90° and +25°")

# Get joint ID
f1_palm_id = model.joint('F1_palm_knuckle').id

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view for good visibility
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.25
    viewer.cam.azimuth = 60
    viewer.cam.elevation = -15
    viewer.cam.lookat = np.array([0.075, 0.03, 0.05])

    step = 0
    angle = -90
    direction = 1
    
    print(f"\nVisualization started. Cycling finger angle...")
    
    while viewer.is_running():
        # Cycle angle between -90 and +25 degrees
        if step % 80 == 0:
            angle += direction * 5
            if angle >= 25:
                angle = 25
                direction = -1
            elif angle <= -90:
                angle = -90
                direction = 1
            
            data.qpos[f1_palm_id] = np.radians(angle)
            
            # Print status at key angles
            if angle in [-90, -45, 0, 25]:
                print(f"  Angle: {angle:3d}° - Check marker alignment")
        
        # Forward kinematics
        mujoco.mj_forward(model, data)
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()
        
        step += 1

print("\nVisualization ended.")
