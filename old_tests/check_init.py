import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print('Joint initialization positions:')
print('=' * 50)

# Get all joint names and their qpos0 values
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    joint_addr = model.jnt_qposadr[i]
    qpos0_value = model.qpos0[joint_addr]
    print(f'{joint_name:20s}: qpos0 = {qpos0_value:7.4f} rad ({np.rad2deg(qpos0_value):7.2f}°)')

print()
print('Body positions (quaternions):')
print('=' * 50)

# Check body positions/quaternions
for i in range(model.nbody):
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if body_name and body_name != 'world':
        # Bodies don't have direct qpos addresses - they inherit from parent joints
        # Let's check the initial body positions from the XML instead
        print(f'{body_name:20s}: (position from XML)')
