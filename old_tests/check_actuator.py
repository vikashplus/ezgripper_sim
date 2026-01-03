#!/usr/bin/env python3
"""
Check actual actuator force vs control signal
"""
import mujoco
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get IDs
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'flex_tendon')

print("=" * 80)
print("ACTUATOR CONFIGURATION")
print("=" * 80)

# Check actuator settings
print(f"\nActuator gear: {model.actuator_gear[gripper_actuator_id]}")
print(f"Actuator type: {model.actuator_dyntype[gripper_actuator_id]}")
print(f"Control range: {model.actuator_ctrlrange[gripper_actuator_id]}")

print("\n" + "=" * 80)
print("FORCE AT DIFFERENT CONTROL LEVELS")
print("=" * 80)

for ctrl_val in [-0.25, -0.5, -1.0]:
    mujoco.mj_resetData(model, data)
    data.ctrl[gripper_actuator_id] = ctrl_val
    
    # Step once to compute forces
    mujoco.mj_step(model, data)
    
    print(f"\nControl: {ctrl_val:.2f}")
    print(f"  data.ctrl: {data.ctrl[gripper_actuator_id]:.3f}")
    print(f"  data.actuator_force: {data.actuator_force[gripper_actuator_id]:.3f}")
    print(f"  data.ten_length: {data.ten_length[tendon_id]:.6f} m")
    
    # The actual force in the tendon
    # For motor actuators, force = gear * ctrl
    expected_force = model.actuator_gear[gripper_actuator_id, 0] * ctrl_val
    print(f"  Expected force (gear × ctrl): {expected_force:.3f} N")

print("\n" + "=" * 80)
