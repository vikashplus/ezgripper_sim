#!/usr/bin/env python3
"""
Visual comparison of current vs correct mechanical stop positions.
"""

import mujoco
import numpy as np
import os

print("="*80)
print("VISUAL STOP POSITION COMPARISON")
print("="*80)

# Load both models
original_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
corrected_path = os.path.join(os.path.dirname(__file__), 'ezgripper_permanent_visualization.xml')

original_model = mujoco.MjModel.from_xml_path(original_path)
corrected_model = mujoco.MjModel.from_xml_path(corrected_path)

original_data = mujoco.MjData(original_model)
corrected_data = mujoco.MjData(corrected_model)

print("\n📊 COMPARISON AT KEY ANGLES:")
print("="*60)

test_angles = [-90, -45, 0, 25]

for angle in test_angles:
    print(f"\n🔍 Angle: {angle:3d}°")
    print("-" * 40)
    
    # Set angles
    original_data.qpos[original_model.joint('F1_palm_knuckle').id] = np.radians(angle)
    corrected_data.qpos[corrected_model.joint('F1_palm_knuckle').id] = np.radians(angle)
    
    # Forward kinematics
    mujoco.mj_forward(original_model, original_data)
    mujoco.mj_forward(corrected_model, corrected_data)
    
    # Get stop positions for original
    palm_lower_orig = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
    finger_lower_orig = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")]
    dist_lower_orig = np.linalg.norm(finger_lower_orig - palm_lower_orig)
    
    # Get stop positions for corrected (has visual markers)
    palm_lower_corr = corrected_data.geom_xpos[mujoco.mj_name2id(corrected_model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
    finger_lower_corr = corrected_data.geom_xpos[mujoco.mj_name2id(corrected_model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")]
    dist_lower_corr = np.linalg.norm(finger_lower_corr - palm_lower_corr)
    
    # Check contact
    contacts_orig = original_data.ncon
    contacts_corr = corrected_data.ncon
    
    print(f"  Original Model:")
    print(f"    Palm stop:  [{palm_lower_orig[0]:.3f}, {palm_lower_orig[1]:.3f}, {palm_lower_orig[2]:.3f}]")
    print(f"    Finger stop: [{finger_lower_orig[0]:.3f}, {finger_lower_orig[1]:.3f}, {finger_lower_orig[2]:.3f}]")
    print(f"    Distance: {dist_lower_orig:.4f}")
    print(f"    Contact: {'✅ YES' if contacts_orig > 0 else '❌ NO'}")
    
    print(f"  With Visual Markers:")
    print(f"    Palm stop:  [{palm_lower_corr[0]:.3f}, {palm_lower_corr[1]:.3f}, {palm_lower_corr[2]:.3f}]")
    print(f"    Finger stop: [{finger_lower_corr[0]:.3f}, {finger_lower_corr[1]:.3f}, {finger_lower_corr[2]:.3f}]")
    print(f"    Distance: {dist_lower_corr:.4f}")
    print(f"    Contact: {'✅ YES' if contacts_corr > 0 else '❌ NO'}")
    
    # Show improvement
    if dist_lower_corr < dist_lower_orig:
        improvement = dist_lower_orig - dist_lower_corr
        print(f"    📈 Improvement: {improvement:.4f} closer to contact")

print("\n" + "="*80)
print("🎯 KEY FINDINGS:")
print("="*80)

# Test at -90° (should engage lower stop)
original_data.qpos[original_model.joint('F1_palm_knuckle').id] = np.radians(-90)
mujoco.mj_forward(original_model, original_data)

palm_lower = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
finger_lower = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")]
dist_at_neg90 = np.linalg.norm(finger_lower - palm_lower)

print(f"\n📍 At -90° (lower limit):")
print(f"  Distance between stops: {dist_at_neg90:.4f}")
print(f"  Should be: < 0.01 for contact")
print(f"  Status: {'✅ WORKING' if dist_at_neg90 < 0.01 else '❌ NOT WORKING'}")

# Test at +25° (should engage upper stop)
original_data.qpos[original_model.joint('F1_palm_knuckle').id] = np.radians(25)
mujoco.mj_forward(original_model, original_data)

palm_upper = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")]
finger_upper = original_data.geom_xpos[mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_upper")]
dist_at_pos25 = np.linalg.norm(finger_upper - palm_upper)

print(f"\n📍 At +25° (upper limit):")
print(f"  Distance between stops: {dist_at_pos25:.4f}")
print(f"  Should be: < 0.01 for contact")
print(f"  Status: {'✅ WORKING' if dist_at_pos25 < 0.01 else '❌ NOT WORKING'}")

print("\n" + "="*80)
print("🔧 RECOMMENDATION:")
print("="*80)
if dist_at_neg90 > 0.01 or dist_at_pos25 > 0.01:
    print("❌ Mechanical stops are NOT positioned correctly!")
    print("📝 Update finger stop positions to:")
    print("   Lower stop: pos='0.0130 0.0000 -0.1000'")
    print("   Upper stop: pos='-0.0071 0.0000 -0.1000'")
    print("🎯 This will make stops engage at the correct joint limits")
else:
    print("✅ Mechanical stops are working correctly!")

print("\n💡 The visual markers in ezgripper_permanent_visualization.xml")
print("   show the current (RED) vs correct (GREEN) positions clearly.")
print("="*80)
