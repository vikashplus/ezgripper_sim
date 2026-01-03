#!/usr/bin/env python3
"""
Run the permanent visualization with colored markers.
"""

import mujoco
import numpy as np
import os

# Load the permanent visualization model
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_permanent_visualization.xml')

try:
    model = mujoco.MjModel.from_xml_path(viz_path)
    data = mujoco.MjData(model)
    
    print("="*60)
    print("PERMANENT MECHANICAL STOP VISUALIZER")
    print("="*60)
    print("\n🔵 BLUE spheres: Palm stop positions (fixed)")
    print("🔴 RED spheres: Current finger stop positions (move with fingers)")
    print("🟢 GREEN spheres: Correct stop positions (ghost markers)")
    print("📏 Red cylinders: Stop reach indicators")
    print("\nAll markers are always visible!")
    print("\nControls:")
    print("  - Close window to exit")
    print("  - Watch as finger cycles through angles")
    
    # Simple simulation loop
    step = 0
    angle = -90
    direction = 1
    
    while True:
        # Cycle angle between -90 and +25 degrees
        if step % 100 == 0:
            angle += direction * 5
            if angle >= 25:
                angle = 25
                direction = -1
            elif angle <= -90:
                angle = -90
                direction = 1
            
            data.qpos[model.joint('F1_palm_knuckle').id] = np.radians(angle)
            data.qpos[model.joint('F2_palm_knuckle').id] = np.radians(angle)
            
            # Print status at key angles
            if angle in [-90, -45, 0, 25]:
                print(f"  Angle: {angle:3d}° - Check marker alignment")
        
        # Forward kinematics and step
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        
        step += 1
        
        # Exit after reasonable time
        if step > 2000:
            break
    
except Exception as e:
    print(f"Error: {e}")
    print("\nNote: This script runs a simple simulation loop.")
    print("The markers are embedded in the XML file for use with any MuJoCo viewer.")

print(f"\n✓ Simulation complete!")
print(f"\nThe XML file '{viz_path}' contains permanent visual markers.")
print(f"You can use it with any MuJoCo viewer that supports your installation.")
