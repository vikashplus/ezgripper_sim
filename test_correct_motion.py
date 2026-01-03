#!/usr/bin/env python3
"""
Correct gripper motion test.

PHYSICS (confirmed by user):
- Springs at -3.14 rad (-180°) pull fingers CLOSED
- Joint limit at -90° prevents over-closing
- At -90°: Springs in tension, pushing against hard stops (OPEN position)
- Tendon pulls AGAINST springs to rotate toward +25° (CLOSE position)

CONTROL:
- Positive control = PULL tendon = CLOSE gripper (toward +25°)
- Negative/Zero control = RELEASE tendon = Springs close to -90° (OPEN)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("CORRECT GRIPPER MOTION TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    print("\nGripper Physics:")
    print("  -90° = OPEN (springs in tension against hard stops)")
    print("  +25° = CLOSED (fingers together)")
    print("  Springs pull toward -180° (closed)")
    print("  Tendon pulls AGAINST springs toward +25° (close)")
    print("\nControl:")
    print("  Positive (+) = PULL tendon = CLOSE (toward +25°)")
    print("  Negative (-) = RELEASE = Springs open to -90°")
    
    print("\n" + "="*70)
    print("INTERACTIVE VIEWER")
    print("="*70)
    print("\nStarting from CLOSED position (+20°)")
    print("Will cycle through:")
    print("  1. RELEASE (ctrl = -2.0) → Springs open to -90°")
    print("  2. CLOSE (ctrl = +2.0) → Tendon closes to +25°")
    print("  3. OSCILLATE between open and closed")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        # Start CLOSED
        mujoco.mj_resetData(model, data)
        data.qpos[f1_palm_id] = np.deg2rad(20)  # Start closed
        data.qpos[f2_palm_id] = np.deg2rad(20)
        mujoco.mj_forward(model, data)
        
        step = 0
        phase_duration = 3000  # 3 seconds per phase
        
        while viewer.is_running():
            step_start = time.time()
            
            phase = (step // phase_duration) % 3
            
            if phase == 0:
                # RELEASE - let springs open
                data.ctrl[act_id] = -2.0
                phase_name = "RELEASING (Springs open)"
                
            elif phase == 1:
                # CLOSE - pull tendon
                data.ctrl[act_id] = 2.0
                phase_name = "CLOSING (Tendon pulls)"
                
            else:
                # OSCILLATE
                t = (step % phase_duration) * model.opt.timestep
                data.ctrl[act_id] = 2.0 * np.sin(2 * np.pi * 0.3 * t)
                phase_name = "OSCILLATING"
            
            mujoco.mj_step(model, data)
            
            if step % 100 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                diff = abs(f1 - f2)
                
                # Determine state
                if f1 < -80:
                    state = "OPEN"
                elif f1 > 15:
                    state = "CLOSED"
                else:
                    state = "PARTIAL"
                
                print(f"[{step:5d}] {phase_name:25s} | "
                      f"{state:7s} | "
                      f"F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                      f"Diff:{diff:5.2f}° | Ctrl:{data.ctrl[act_id]:6.2f}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
