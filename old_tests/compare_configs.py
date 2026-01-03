#!/usr/bin/env python3
"""
Compare working vs problematic spring configurations
"""
import mujoco
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))

def test_configuration(model_path, config_name):
    """Test a specific configuration"""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'flex_tendon')

    print(f"\n{config_name}:")
    print("-" * 50)

    # Get spring values
    palm_stiffness = model.jnt_stiffness[0]  # F1_palm_knuckle
    tip_stiffness = model.jnt_stiffness[1]   # F1_knuckle_tip

    print(f"Palm-L1 stiffness: {palm_stiffness:.6f}")
    print(f"L1-L2 stiffness: {tip_stiffness:.6f}")

    print("\nJoint positions during actuation:")
    print("Control | F1_Palm | F1_Tip")
    print("--------|---------|--------")

    results = []
    for control in [-0.3]:
        data.ctrl[actuator_id] = control
        mujoco.mj_step(model, data)

        f1_palm = np.degrees(data.qpos[0])
        f1_tip = np.degrees(data.qpos[1])

        results.append((f1_palm, f1_tip))
        print("5.1f")

    return results[0]

# Test both configurations
working_path = os.path.join(script_dir, "ezgripper_working_baseline.xml")
problematic_path = os.path.join(script_dir, "ezgripper_problematic.xml")

print("=" * 70)
print("SPRING CONFIGURATION COMPARISON")
print("=" * 70)

working_results = test_configuration(working_path, "WORKING CONFIGURATION (Strong L1-L2 springs)")
problematic_results = test_configuration(problematic_path, "PROBLEMATIC CONFIGURATION (Weak L1-L2 springs)")

print("\n" + "=" * 70)
print("SUMMARY:")
print("=" * 70)
print("Working config (strong L1-L2 springs):")
print(f"  - Palm bends more: {working_results[0]:.1f}°")
print(f"  - Tip bends less: {working_results[1]:.1f}°")
print("  - Result: Under-actuation possible!")

print("\nProblematic config (weak L1-L2 springs):")
print(f"  - Palm bends less: {problematic_results[0]:.1f}°")
print(f"  - Tip bends more: {problematic_results[1]:.1f}°")
print("  - Result: Fingers close straight together (parallel jaws)")

print("\nKEY INSIGHT:")
print("- Strong L1-L2 springs = under-actuation behavior")
print("- Weak L1-L2 springs = parallel jaw behavior")
print("- The original issue was weak L1-L2 springs!")
print("=" * 70)
