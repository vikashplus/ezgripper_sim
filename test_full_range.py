#!/usr/bin/env python3
"""
Test full range of motion to understand the gripper kinematics.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("FULL RANGE OF MOTION TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    
    # Get joint info
    joint_range = model.jnt_range[f1_palm_id]
    spring_ref = model.jnt_stiffness[f1_palm_id]
    
    print(f"\nJoint Configuration:")
    print(f"  Range: {np.rad2deg(joint_range[0]):.1f}° to {np.rad2deg(joint_range[1]):.1f}°")
    print(f"  Spring reference: {model.qpos_spring[f1_palm_id]:.2f} rad = {np.rad2deg(model.qpos_spring[f1_palm_id]):.1f}°")
    
    print("\n" + "="*70)
    print("TEST 1: Manual position sweep (no control)")
    print("="*70)
    
    # Test different positions manually
    test_angles = [-90, -60, -30, 0, 25]
    
    for angle_deg in test_angles:
        angle_rad = np.deg2rad(angle_deg)
        
        mujoco.mj_resetData(model, data)
        data.qpos[f1_palm_id] = angle_rad
        data.qpos[f2_palm_id] = angle_rad
        data.ctrl[act_id] = 0.0
        mujoco.mj_forward(model, data)
        
        tendon_len = data.ten_length[t1_id]
        
        print(f"\nAngle: {angle_deg:4.0f}° | Tendon length: {tendon_len:.6f}")
        
        # Let it settle
        for _ in range(100):
            mujoco.mj_step(model, data)
        
        final_angle = np.rad2deg(data.qpos[f1_palm_id])
        final_tendon = data.ten_length[t1_id]
        print(f"  After settling: {final_angle:6.2f}° | Tendon: {final_tendon:.6f}")
    
    print("\n" + "="*70)
    print("TEST 2: Control sweep")
    print("="*70)
    
    # Test different control values
    test_controls = [-2.0, -1.0, 0.0, 1.0, 2.0]
    
    for ctrl_val in test_controls:
        mujoco.mj_resetData(model, data)
        data.qpos[f1_palm_id] = 0.0  # Start at neutral
        data.qpos[f2_palm_id] = 0.0
        data.ctrl[act_id] = ctrl_val
        
        # Simulate for 2 seconds
        for _ in range(2000):
            mujoco.mj_step(model, data)
        
        final_angle = np.rad2deg(data.qpos[f1_palm_id])
        final_tendon = data.ten_length[t1_id]
        
        print(f"\nControl: {ctrl_val:5.1f} | Final angle: {final_angle:7.2f}° | Tendon: {final_tendon:.6f}")
    
    print("\n" + "="*70)
    print("TEST 3: Interactive visualization")
    print("="*70)
    print("\nStarting from +25° (upper limit)")
    print("Will apply POSITIVE control to see direction of motion")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        # Start at upper limit
        mujoco.mj_resetData(model, data)
        data.qpos[f1_palm_id] = np.deg2rad(25)
        data.qpos[f2_palm_id] = np.deg2rad(25)
        mujoco.mj_forward(model, data)
        
        step = 0
        phase_duration = 3000
        
        while viewer.is_running():
            step_start = time.time()
            
            phase = (step // phase_duration) % 4
            
            if phase == 0:
                # Start position: +25° (upper limit)
                data.ctrl[act_id] = 0.0
                phase_name = "AT +25° (UPPER LIMIT)"
                
            elif phase == 1:
                # Apply POSITIVE control
                data.ctrl[act_id] = 2.0
                phase_name = "POSITIVE CTRL (+2.0)"
                
            elif phase == 2:
                # Apply NEGATIVE control
                data.ctrl[act_id] = -2.0
                phase_name = "NEGATIVE CTRL (-2.0)"
                
            else:
                # Oscillate
                t = (step % phase_duration) * model.opt.timestep
                data.ctrl[act_id] = 2.0 * np.sin(2 * np.pi * 0.3 * t)
                phase_name = "OSCILLATING"
            
            mujoco.mj_step(model, data)
            
            if step % 100 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                tendon = data.ten_length[t1_id]
                
                print(f"[{step:5d}] {phase_name:25s} | "
                      f"F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                      f"Tendon:{tendon:.6f} | Ctrl:{data.ctrl[act_id]:6.2f}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
