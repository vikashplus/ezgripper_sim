#!/usr/bin/env python3
"""
Visualize L1/L2 joint at its limit positions to validate the range.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("L1/L2 JOINT LIMIT VISUALIZATION")
    print("="*70)
    
    # Load model
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get joint IDs
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    
    # Get qpos addresses
    f1_tip_adr = model.jnt_qposadr[f1_tip_id]
    f2_tip_adr = model.jnt_qposadr[f2_tip_id]
    f1_palm_adr = model.jnt_qposadr[f1_palm_id]
    f2_palm_adr = model.jnt_qposadr[f2_palm_id]
    
    # Get limits
    tip_limit_low = model.jnt_range[f1_tip_id, 0]
    tip_limit_high = model.jnt_range[f1_tip_id, 1]
    
    print(f"\nL1/L2 Joint Limits:")
    print(f"  Lower limit: {np.rad2deg(tip_limit_low):.2f}°")
    print(f"  Upper limit: {np.rad2deg(tip_limit_high):.2f}°")
    print(f"\nVisualization sequence:")
    print(f"  1. Lower limit ({np.rad2deg(tip_limit_low):.0f}°) - STRAIGHT (L1/L2 aligned)")
    print(f"  2. Upper limit ({np.rad2deg(tip_limit_high):.0f}°) - FULLY CURLED")
    print(f"\nPress TAB to cycle through positions")
    print(f"Press ESC to exit")
    print("="*70)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        # Position 0: Lower limit (0°)
        position = 0
        positions = [
            (f"LOWER LIMIT ({np.rad2deg(tip_limit_low):.0f}°) - STRAIGHT (L1/L2 aligned)", tip_limit_low),
            (f"UPPER LIMIT ({np.rad2deg(tip_limit_high):.0f}°) - FULLY CURLED", tip_limit_high),
        ]
        
        last_tab_time = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # Check for TAB key press (cycle positions)
            current_time = time.time()
            if viewer.is_running() and current_time - last_tab_time > 0.5:
                # Auto-cycle every 3 seconds
                if current_time - last_tab_time > 3.0:
                    position = (position + 1) % len(positions)
                    last_tab_time = current_time
            
            # Set joint positions
            mujoco.mj_resetData(model, data)
            
            # Set palm to neutral
            data.qpos[f1_palm_adr] = 0.0
            data.qpos[f2_palm_adr] = 0.0
            
            # Set L1/L2 to current position
            pos_name, pos_angle = positions[position]
            data.qpos[f1_tip_adr] = pos_angle
            data.qpos[f2_tip_adr] = pos_angle
            
            # Forward kinematics
            mujoco.mj_forward(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Print current position
            if step_start - last_tab_time < 0.1:  # Just changed
                print(f"\n>>> {pos_name}")
                print(f"    L1/L2 angle: {np.rad2deg(pos_angle):.2f}°")
            
            # Maintain frame rate
            elapsed = time.time() - step_start
            if elapsed < 0.016:  # ~60 FPS
                time.sleep(0.016 - elapsed)
    
    print("\n" + "="*70)
    print("Visualization complete")
    print("="*70)

if __name__ == '__main__':
    main()
