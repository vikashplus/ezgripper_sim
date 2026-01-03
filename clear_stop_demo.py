#!/usr/bin/env python3
"""
Clear demonstration of mechanical stop engagement points.
"""

import mujoco
import numpy as np
import os

# Load the permanent visualization model
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_permanent_visualization.xml')
model = mujoco.MjModel.from_xml_path(viz_path)
data = mujoco.MjData(model)

print("="*80)
print("MECHANICAL STOP ENGAGEMENT DEMONSTRATION")
print("="*80)
print("\nThis will show EXACTLY when mechanical stops engage:")
print("🔵 BLUE spheres = Palm stops (fixed reference)")
print("🔴 RED spheres = Finger stops (move with fingers)")
print("✅ COLLISION = When RED and BLUE spheres overlap")
print("\nWatch for contact detection messages...")

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f2_palm_id = model.joint('F2_palm_knuckle').id

# Get collision geometry IDs
palm_lower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")
palm_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")
finger_lower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")
finger_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_upper")

print(f"\nCollision geometry IDs:")
print(f"  Palm lower: {palm_lower_id}")
print(f"  Palm upper: {palm_upper_id}")
print(f"  Finger lower: {finger_lower_id}")
print(f"  Finger upper: {finger_upper_id}")

# Test specific angles where stops should engage
test_angles = [-90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 25, 30]

print(f"\n{'Angle':>6} {'Lower_Stop':>11} {'Upper_Stop':>11} {'Contacts':>9} {'Status'}")
print("-" * 50)

for angle in test_angles:
    # Set joint angle
    data.qpos[f1_palm_id] = np.radians(angle)
    data.qpos[f2_palm_id] = np.radians(angle)
    
    # Forward kinematics
    mujoco.mj_forward(model, data)
    
    # Check distances between stop pairs
    palm_lower_pos = data.geom_xpos[palm_lower_id]
    palm_upper_pos = data.geom_xpos[palm_upper_id]
    finger_lower_pos = data.geom_xpos[finger_lower_id]
    finger_upper_pos = data.geom_xpos[finger_upper_id]
    
    dist_lower = np.linalg.norm(finger_lower_pos - palm_lower_pos)
    dist_upper = np.linalg.norm(finger_upper_pos - palm_upper_pos)
    
    # Check for actual collisions
    lower_collides = dist_lower < 0.01
    upper_collides = dist_upper < 0.01
    
    # Count contacts
    contacts = data.ncon
    
    # Determine status
    if lower_collides and upper_collides:
        status = "⚠️  BOTH STOPS!"
    elif lower_collides:
        status = "⬇️  LOWER STOP"
    elif upper_collides:
        status = "⬆️  UPPER STOP"
    elif contacts > 0:
        status = "🔗 OTHER CONTACT"
    else:
        status = "  No contact"
    
    print(f"{angle:6d}° {dist_lower:11.4f} {dist_upper:11.4f} {contacts:9d} {status}")

print("\n" + "="*80)
print("STOP ENGAGEMENT ANALYSIS:")
print("="*80)

# Find actual engagement points
print("\nTesting fine-grained angles around expected limits:")

# Test lower limit (-90°)
print("\n🔍 Testing LOWER LIMIT (-90°):")
for angle in range(-95, -84):
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_forward(model, data)
    
    palm_lower_pos = data.geom_xpos[palm_lower_id]
    finger_lower_pos = data.geom_xpos[finger_lower_id]
    dist_lower = np.linalg.norm(finger_lower_pos - palm_lower_pos)
    
    if dist_lower < 0.01:
        print(f"  ✅ Lower stop engages at {angle}° (distance: {dist_lower:.4f})")
        break

# Test upper limit (+25°)
print("\n🔍 Testing UPPER LIMIT (+25°):")
for angle in range(20, 31):
    data.qpos[f1_palm_id] = np.radians(angle)
    mujoco.mj_forward(model, data)
    
    palm_upper_pos = data.geom_xpos[palm_upper_id]
    finger_upper_pos = data.geom_xpos[finger_upper_id]
    dist_upper = np.linalg.norm(finger_upper_pos - palm_upper_pos)
    
    if dist_upper < 0.01:
        print(f"  ✅ Upper stop engages at {angle}° (distance: {dist_upper:.4f})")
        break

print("\n" + "="*80)
print("EXPECTED vs ACTUAL STOP BEHAVIOR:")
print("="*80)
print("Expected:")
print("  Lower stop should engage at -90°")
print("  Upper stop should engage at +25°")
print("\nActual:")
print("  Based on the distances above, check if stops engage at these angles")
print("  If not, the stop positions need to be corrected")

print("\n🎯 TO FIX: Update finger stop positions to:")
print("  Lower: pos='0.0130 0.0000 -0.1000'")
print("  Upper: pos='-0.0071 0.0000 -0.1000'")

print("\n" + "="*80)
