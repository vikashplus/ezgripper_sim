#!/usr/bin/env python3
"""
True visual simulation of EZGripper with mechanical stop markers.
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
    print("EZGRIPPER VISUAL SIMULATION")
    print("="*60)
    print("\n🔵 BLUE spheres: Palm stop positions (fixed)")
    print("🔴 RED spheres: Current finger stop positions (move with fingers)")
    print("🟢 GREEN spheres: Correct stop positions (ghost markers)")
    print("\n attempting to launch visual window...")
    
    # Try to use any available visualization method
    viewer_launched = False
    
    # Method 1: Try mujoco.viewer (most recent versions)
    try:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer_launched = True
            print("✓ MuJoCo viewer launched!")
            
            # Set camera for good view
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            viewer.cam.distance = 0.3
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20
            viewer.cam.lookat = np.array([0.08, 0.03, 0.05])
            
            # Run visual simulation
            step = 0
            phase = 0  # 0=open, 1=close, 2=hold
            
            while viewer.is_running():
                # Cycle through gripper motions
                if step % 200 == 0:
                    phase = (phase + 1) % 3
                    
                if phase == 0:  # Opening
                    target_angle = -60
                elif phase == 1:  # Closing
                    target_angle = 30
                else:  # Hold
                    target_angle = 0
                
                # Smooth motion
                current_angle = np.degrees(data.qpos[model.joint('F1_palm_knuckle').id])
                angle_diff = target_angle - current_angle
                data.qpos[model.joint('F1_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                data.qpos[model.joint('F2_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                
                # Step physics
                mujoco.mj_forward(model, data)
                mujoco.mj_step(model, data)
                
                # Update viewer
                viewer.sync()
                step += 1
                
    except Exception as e:
        print(f"✗ MuJoCo viewer failed: {e}")
    
    # Method 2: Try mujoco_py (older versions)
    if not viewer_launched:
        try:
            import mujoco_py
            viewer = mujoco_py.MjViewer(model)
            print("✓ MuJoCo_Py viewer launched!")
            
            step = 0
            phase = 0
            
            while True:
                if step % 200 == 0:
                    phase = (phase + 1) % 3
                    
                if phase == 0:
                    target_angle = -60
                elif phase == 1:
                    target_angle = 30
                else:
                    target_angle = 0
                
                current_angle = np.degrees(data.qpos[model.joint('F1_palm_knuckle').id])
                angle_diff = target_angle - current_angle
                data.qpos[model.joint('F1_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                data.qpos[model.joint('F2_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                
                mujoco.mj_step(model, data)
                viewer.render()
                step += 1
                
        except ImportError:
            print("✗ MuJoCo_Py not available")
        except Exception as e:
            print(f"✗ MuJoCo_Py failed: {e}")
    
    # Method 3: Try OpenGL rendering
    if not viewer_launched:
        try:
            print("✓ Trying OpenGL rendering...")
            
            # Create simple OpenGL context
            from OpenGL.GL import *
            from OpenGL.GLUT import *
            from OpenGL.GLU import *
            
            glutInit()
            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
            glutInitWindowSize(800, 600)
            glutCreateWindow("EZGripper Simulation")
            
            def display():
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
                
                # Simple rendering
                glLoadIdentity()
                gluLookAt(0.2, 0.1, 0.2, 0.08, 0.03, 0.05, 0, 0, 1)
                
                # Step simulation
                for _ in range(10):
                    mujoco.mj_step(model, data)
                
                glutSwapBuffers()
            
            def idle():
                glutPostRedisplay()
            
            glutDisplayFunc(display)
            glutIdleFunc(idle)
            glutMainLoop()
            
        except ImportError:
            print("✗ OpenGL not available")
        except Exception as e:
            print(f"✗ OpenGL failed: {e}")
    
    if not viewer_launched:
        print("\n❌ No visualization available in this environment.")
        print("To get visual simulation, you need:")
        print("  1. MuJoCo with viewer support: pip install mujoco[mujoco-viewer]")
        print("  2. Or MuJoCo-Py: pip install mujoco-py")
        print("  3. Or install system packages: sudo apt install python3-mujoco")
        
        print("\n🔄 Running text-based simulation instead...")
        
        # Text-based simulation as fallback
        step = 0
        phase = 0
        
        for step in range(1000):
            if step % 200 == 0:
                phase = (phase + 1) % 3
                phase_names = ["OPENING", "CLOSING", "HOLDING"]
                print(f"\n=== {phase_names[phase]} ===")
                
            if phase == 0:
                target_angle = -60
            elif phase == 1:
                target_angle = 30
            else:
                target_angle = 0
            
            current_angle = np.degrees(data.qpos[model.joint('F1_palm_knuckle').id])
            angle_diff = target_angle - current_angle
            data.qpos[model.joint('F1_palm_knuckle').id] += np.radians(angle_diff * 0.02)
            data.qpos[model.joint('F2_palm_knuckle').id] += np.radians(angle_diff * 0.02)
            
            mujoco.mj_forward(model, data)
            mujoco.mj_step(model, data)
            
            if step % 50 == 0:
                f1_angle = np.degrees(data.qpos[model.joint('F1_palm_knuckle').id])
                f2_angle = np.degrees(data.qpos[model.joint('F2_palm_knuckle').id])
                print(f"  Step {step:3d}: F1={f1_angle:5.1f}° F2={f2_angle:5.1f}° Contacts={data.ncon}")

except Exception as e:
    print(f"Error loading model: {e}")

print("\nSimulation ended.")
