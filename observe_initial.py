import mujoco
import mujoco.viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("=" * 80)
print("INITIAL POSITION OBSERVATION")
print("=" * 80)
print("No actuator force - observing initial joint positions")
print("=" * 80 + "\n")

# Get joint IDs
f1_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_l1l2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

# Get joint addresses
f1_palm_adr = model.jnt_qposadr[f1_palm]
f1_l1l2_adr = model.jnt_qposadr[f1_l1l2]
f2_palm_adr = model.jnt_qposadr[f2_palm]
f2_l1l2_adr = model.jnt_qposadr[f2_l1l2]

# Get actuator IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')

# Disable all actuator forces
data.ctrl[gripper_f1_id] = 0.0
data.ctrl[gripper_f2_id] = 0.0

# Reset to initial positions (qpos0)
data.qpos[:] = model.qpos0

print("Initial joint positions (from qpos0):")
print(f"  F1 Palm: {np.rad2deg(data.qpos[f1_palm_adr]):6.1f}°")
print(f"  F1 L1L2: {np.rad2deg(data.qpos[f1_l1l2_adr]):6.1f}°")
print(f"  F2 Palm: {np.rad2deg(data.qpos[f2_palm_adr]):6.1f}°")
print(f"  F2 L1L2: {np.rad2deg(data.qpos[f2_l1l2_adr]):6.1f}°")
print()

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Camera setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Run simulation with no actuator force
    while viewer.is_running():
        # Keep actuators disabled
        data.ctrl[gripper_f1_id] = 0.0
        data.ctrl[gripper_f2_id] = 0.0
        
        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Print state every 2 seconds
        if int(data.time * 50) % 100 == 0:
            # Get current positions
            f1_palm_angle = np.rad2deg(data.qpos[f1_palm_adr])
            f1_l1l2_angle = np.rad2deg(data.qpos[f1_l1l2_adr])
            f2_palm_angle = np.rad2deg(data.qpos[f2_palm_adr])
            f2_l1l2_angle = np.rad2deg(data.qpos[f2_l1l2_adr])
            
            # Calculate spring torques
            palm_spring_torque_f1 = -0.05114105365 * (np.deg2rad(f1_palm_angle) - 0.27)
            l1l2_spring_torque_f1 = -0.02459949416 * (np.deg2rad(f1_l1l2_angle) - 1.7)
            palm_spring_torque_f2 = -0.05114105365 * (np.deg2rad(f2_palm_angle) - 0.27)
            l1l2_spring_torque_f2 = -0.02459949416 * (np.deg2rad(f2_l1l2_angle) - 1.7)
            
            print(f"t={data.time:5.1f}s")
            print(f"  F1: Palm={f1_palm_angle:6.1f}°  L1L2={f1_l1l2_angle:5.1f}°  Torques: Palm={palm_spring_torque_f1:.3f}Nm  L1L2={l1l2_spring_torque_f1:.3f}Nm")
            print(f"  F2: Palm={f2_palm_angle:6.1f}°  L1L2={f2_l1l2_angle:5.1f}°  Torques: Palm={palm_spring_torque_f2:.3f}Nm  L1L2={l1l2_spring_torque_f2:.3f}Nm")
            print()

print("\nDone!")
