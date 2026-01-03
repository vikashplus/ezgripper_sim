import mujoco
import mujoco.viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# TEMPORARILY DISABLE SPRINGS - Set stiffness to zero
model.jnt_stiffness[:] = 0.0

print("=" * 80)
print("GRIPPER RANGE LIMITS DEMONSTRATION (SPRINGS DISABLED)")
print("=" * 80)
print("Showing fingers at opposite limits for photography")
print("Springs temporarily disabled to hold positions")
print("=" * 80 + "\n")

# Get joint IDs and addresses
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]
f2_palm_adr = model.jnt_qposadr[f2_palm]
f2_l1l2_adr = model.jnt_qposadr[f2_l1l2]

# Get actuator IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')

# Disable actuators
data.ctrl[gripper_f1_id] = 0.0
data.ctrl[gripper_f2_id] = 0.0

# Get joint ranges
f1_palm_range = model.jnt_range[f1_palm]
f1_l1l2_range = model.jnt_range[f1_l1l2]
f2_palm_range = model.jnt_range[f2_palm]
f2_l1l2_range = model.jnt_range[f2_l1l2]

print("JOINT RANGES (Hard Limits):")
print(f"F1 Palm:  {np.rad2deg(f1_palm_range[0]):6.1f}° to {np.rad2deg(f1_palm_range[1]):6.1f}°")
print(f"F1 L1L2:  {np.rad2deg(f1_l1l2_range[0]):6.1f}° to {np.rad2deg(f1_l1l2_range[1]):6.1f}°")
print(f"F2 Palm:  {np.rad2deg(f2_palm_range[0]):6.1f}° to {np.rad2deg(f2_palm_range[1]):6.1f}°")
print(f"F2 L1L2:  {np.rad2deg(f2_l1l2_range[0]):6.1f}° to {np.rad2deg(f2_l1l2_range[1]):6.1f}°")
print()

# Create only positions 2 and 3 for photography
positions = [
    {
        'name': 'POSITION 2: F1 Closed, F2 Open (TAKE PICTURE)',
        'f1_palm': f1_palm_range[1],  # Upper limit (+20°)
        'f1_l1l2': f1_l1l2_range[1],  # Upper limit (+30°)
        'f2_palm': f2_palm_range[0],  # Lower limit (-125°)
        'f2_l1l2': f2_l1l2_range[0],  # Lower limit (-97.4°)
        'duration': 10.0  # Long hold for photography
    },
    {
        'name': 'POSITION 3: F1 Open, F2 Closed (TAKE PICTURE)',
        'f1_palm': f1_palm_range[0],  # Lower limit (-125°)
        'f1_l1l2': f1_l1l2_range[0],  # Lower limit (-97.4°)
        'f2_palm': f2_palm_range[1],  # Upper limit (+20°)
        'f2_l1l2': f2_l1l2_range[1],  # Upper limit (+30°)
        'duration': 10.0  # Long hold for photography
    }
]

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Camera setup for good viewing angle
    viewer.cam.distance = 0.25
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -25
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    current_pos = 0
    pos_start_time = data.time
    target_set = False
    
    print("PHOTOGRAPHY MODE - Positions will hold for 10 seconds each")
    print("Take screenshots when position name appears")
    print("-" * 80)
    
    while viewer.is_running():
        # Keep actuators disabled
        data.ctrl[gripper_f1_id] = 0.0
        data.ctrl[gripper_f2_id] = 0.0
        
        # Check if it's time to switch positions
        if data.time - pos_start_time > positions[current_pos]['duration']:
            current_pos = (current_pos + 1) % len(positions)
            pos_start_time = data.time
            target_set = False
        
        # Set target joint positions (no spring forces)
        if not target_set:
            target = positions[current_pos]
            data.qpos[f1_palm_adr] = target['f1_palm']
            data.qpos[f1_l1l2_adr] = target['f1_l1l2']
            data.qpos[f2_palm_adr] = target['f2_palm']
            data.qpos[f2_l1l2_adr] = target['f2_l1l2']
            target_set = True
            
            print(f"\n*** {positions[current_pos]['name']} ***")
            print(f"  F1: Palm={np.rad2deg(target['f1_palm']):6.1f}°  L1L2={np.rad2deg(target['f1_l1l2']):6.1f}°")
            print(f"  F2: Palm={np.rad2deg(target['f2_palm']):6.1f}°  L1L2={np.rad2deg(target['f2_l1l2']):6.1f}°")
            print("  -> TAKE SCREENSHOT NOW <-")
        
        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()

print("\nDone! Springs will return to normal when script exits.")
