#!/usr/bin/env python3
"""
Simple MuJoCo visualizer for the EZGripper with stop markers.
"""

import mujoco
import numpy as np
import os

# Load the visualization model with colored markers
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_visualization.xml')

try:
    model = mujoco.MjModel.from_xml_path(viz_path)
    data = mujoco.MjData(model)
    
    print("="*60)
    print("MUJOCO VISUALIZER - EZGripper with Stop Markers")
    print("="*60)
    print("\nMarker colors:")
    print("  BLUE spheres: Palm stop positions")
    print("  RED spheres: Current (wrong) finger stop positions")
    print("  GREEN spheres: Correct finger stop positions")
    print("\nControls:")
    print("  - Mouse: Rotate view")
    print("  - Scroll: Zoom in/out")
    print("  - Right-click drag: Pan")
    print("  - Close window to exit")
    print("\nStarting visualizer...")
    
    # Try different viewer launch methods
    viewer_launched = False
    
    # Method 1: mujoco.viewer.launch_passive
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer_launched = True
            print("✓ Viewer launched successfully!")
            
            # Set initial camera view
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            viewer.cam.distance = 0.3
            viewer.cam.azimuth = 60
            viewer.cam.elevation = -20
            viewer.cam.lookat = np.array([0.08, 0.03, 0.05])
            
            # Run simulation loop
            while viewer.is_running():
                # Simple animation - cycle finger angle
                data.time += model.opt.timestep
                angle = 25 * np.sin(data.time * 0.5)  # Oscillate between -25 and +25
                data.qpos[model.joint('F1_palm_knuckle').id] = np.radians(angle)
                
                mujoco.mj_step(model, data)
                viewer.sync()
                
    except Exception as e:
        print(f"✗ Method 1 failed: {e}")
    
    # Method 2: Try mujoco_py if available
    if not viewer_launched:
        try:
            import mujoco_py
            viewer = mujoco_py.MjViewer(model)
            print("✓ Using mujoco_py viewer!")
            
            while True:
                mujoco.mj_step(model, data)
                viewer.render()
                
        except ImportError:
            print("✗ mujoco_py not available")
        except Exception as e:
            print(f"✗ Method 2 failed: {e}")
    
    # Method 3: Try direct rendering
    if not viewer_launched:
        try:
            print("✓ Using basic rendering...")
            
            for step in range(1000):
                mujoco.mj_step(model, data)
                if step % 100 == 0:
                    print(f"Step {step}: Simulation running...")
                    
        except Exception as e:
            print(f"✗ Method 3 failed: {e}")
    
except Exception as e:
    print(f"✗ Failed to load model: {e}")
    print("\nTroubleshooting:")
    print("1. Ensure MuJoCo is installed: pip install mujoco")
    print("2. Check if ezgripper_visualization.xml exists")
    print("3. Verify mesh files are in the meshes/ directory")

print("\nVisualizer ended.")
