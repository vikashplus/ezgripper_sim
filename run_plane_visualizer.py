#!/usr/bin/env python3
"""
Run the plane visualization showing 2D stop planes.
"""

import mujoco
import numpy as np
import os

# Load the plane visualization model
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_plane_visualization.xml')

try:
    model = mujoco.MjModel.from_xml_path(viz_path)
    data = mujoco.MjData(model)
    
    print("="*60)
    print("2D PLANE VISUALIZER - MECHANICAL STOPS")
    print("="*60)
    print("\n🟦 BLUE planes: Palm stop planes (fixed)")
    print("🟥 RED planes: Current finger stop planes (move with fingers)")
    print("🟩 GREEN planes: Correct stop positions (ghost planes)")
    print("\n✅ OVERLAP = STOP ENGAGEMENT")
    print("Watch as red planes rotate and overlap with blue planes!")
    
    # Try to launch visual window
    viewer_launched = False
    
    # Method 1: Try mujoco.viewer
    try:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer_launched = True
            print("✓ 3D Viewer launched with 2D planes!")
            
            # Set camera for good view of planes
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            viewer.cam.distance = 0.25
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -30
            viewer.cam.lookat = np.array([0.08, 0.03, 0.05])
            
            # Run simulation with plane overlap demonstration
            step = 0
            phase = 0  # 0=open, 1=close, 2=hold
            angle = 0
            
            print("\n🎬 Starting plane overlap demonstration...")
            print("Watch for RED and BLUE plane overlap at stop points!")
            
            while viewer.is_running():
                # Cycle through angles to show plane overlap
                if step % 150 == 0:
                    phase = (phase + 1) % 3
                    
                if phase == 0:  # Opening to -90°
                    target_angle = -90
                    phase_name = "OPENING TO LOWER STOP"
                elif phase == 1:  # Closing to +25°
                    target_angle = 25
                    phase_name = "CLOSING TO UPPER STOP"
                else:  # Hold at 0°
                    target_angle = 0
                    phase_name = "NEUTRAL POSITION"
                
                # Smooth motion
                current_angle = np.degrees(data.qpos[model.joint('F1_palm_knuckle').id])
                angle_diff = target_angle - current_angle
                data.qpos[model.joint('F1_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                data.qpos[model.joint('F2_palm_knuckle').id] += np.radians(angle_diff * 0.02)
                
                # Check plane overlap (stop engagement)
                if step % 30 == 0:
                    # Calculate distances between planes
                    palm_lower_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
                    finger_lower_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")]
                    dist_lower = np.linalg.norm(finger_lower_pos - palm_lower_pos)
                    
                    palm_upper_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")]
                    finger_upper_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_upper")]
                    dist_upper = np.linalg.norm(finger_upper_pos - palm_upper_pos)
                    
                    lower_engaged = dist_lower < 0.01
                    upper_engaged = dist_upper < 0.01
                    
                    print(f"  {phase_name}: Angle={current_angle:6.1f}° | Lower={dist_lower:.4f} {'✅' if lower_engaged else '❌'} | Upper={dist_upper:.4f} {'✅' if upper_engaged else '❌'}")
                
                # Step physics and update viewer
                mujoco.mj_forward(model, data)
                mujoco.mj_step(model, data)
                viewer.sync()
                step += 1
                
    except Exception as e:
        print(f"✗ Viewer failed: {e}")
    
    # Method 2: Try mujoco_py
    if not viewer_launched:
        try:
            import mujoco_py
            viewer = mujoco_py.MjViewer(model)
            print("✓ MuJoCo_Py viewer launched!")
            
            step = 0
            phase = 0
            
            while True:
                if step % 150 == 0:
                    phase = (phase + 1) % 3
                    
                if phase == 0:
                    target_angle = -90
                elif phase == 1:
                    target_angle = 25
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
    
    if not viewer_launched:
        print("\n❌ No visual viewer available in this environment.")
        print("The XML file contains 2D planes for use with any MuJoCo viewer.")
        print("\n📊 Running text-based plane overlap analysis...")
        
        # Text-based plane overlap demonstration
        test_angles = [-90, -45, 0, 25]
        
        print(f"\n{'Angle':>6} {'Lower_Dist':>11} {'Upper_Dist':>11} {'Planes_Overlap'}")
        print("-" * 50)
        
        for angle in test_angles:
            data.qpos[model.joint('F1_palm_knuckle').id] = np.radians(angle)
            data.qpos[model.joint('F2_palm_knuckle').id] = np.radians(angle)
            mujoco.mj_forward(model, data)
            
            palm_lower_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
            finger_lower_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")]
            dist_lower = np.linalg.norm(finger_lower_pos - palm_lower_pos)
            
            palm_upper_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")]
            finger_upper_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_upper")]
            dist_upper = np.linalg.norm(finger_upper_pos - palm_upper_pos)
            
            lower_overlap = dist_lower < 0.01
            upper_overlap = dist_upper < 0.01
            
            if lower_overlap and upper_overlap:
                overlap_status = "⚠️  BOTH"
            elif lower_overlap:
                overlap_status = "⬇️  LOWER"
            elif upper_overlap:
                overlap_status = "⬆️  UPPER"
            else:
                overlap_status = "  NONE"
            
            print(f"{angle:6d}° {dist_lower:11.4f} {dist_upper:11.4f} {overlap_status}")

except Exception as e:
    print(f"Error loading model: {e}")

print("\n" + "="*60)
print("PLANE VISUALIZATION COMPLETE")
print("="*60)
print("\n🎯 The 2D planes clearly show:")
print("  ✅ When RED and BLUE planes overlap = STOP ENGAGED")
print("  ❌ When planes are separate = NO STOP ENGAGEMENT")
print("  🟩 GREEN planes show where RED planes SHOULD be positioned")
print("\n🔧 To fix: Move RED planes to GREEN plane positions")
print("="*60)
