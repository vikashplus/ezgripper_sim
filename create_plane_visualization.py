#!/usr/bin/env python3
"""
Create XML with 2D plane visualization for mechanical stops.
Shows finger stop plane and palm stop plane as 10cm x 10cm rectangles.
"""

import mujoco
import numpy as np
import os
import xml.etree.ElementTree as ET

# Load original XML
xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
tree = ET.parse(xml_path)
root = tree.getroot()

print("="*80)
print("CREATING 2D PLANE VISUALIZATION FOR MECHANICAL STOPS")
print("="*80)
print("\n🟦 Blue planes: Palm stop planes (fixed)")
print("🟥 Red planes: Finger stop planes (move with fingers)")
print("✅ Overlap: When planes intersect, stops are engaged")

# Load model to calculate positions
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Calculate positions for plane visualization
f1_palm_id = model.joint('F1_palm_knuckle').id
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

# Find the worldbody and mount body
worldbody = root.find('worldbody')
mount_body = None
for body in worldbody.findall('body'):
    if body.get('name') == 'mount':
        mount_body = body
        break

if mount_body is not None:
    print("Adding 2D plane visualizations...")
    
    # 1. Palm stop planes (BLUE) - fixed to mount body
    # Lower stop plane - oriented perpendicular to finger movement
    palm_lower_plane = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_lower_plane',
        'type': 'box',
        'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick (thin plane)
        'pos': f"{palm_lower_stop_pos[0]} {palm_lower_stop_pos[1]} {palm_lower_stop_pos[2]}",
        'euler': '0 0 0',  # Default orientation
        'contype': '0',  # No collision
        'conaffinity': '0',
        'rgba': '0 0 1 0.7',  # Blue, semi-transparent
        'group': '1'  # Visualization group
    })
    
    # Upper stop plane
    palm_upper_plane = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_upper_plane',
        'type': 'box',
        'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
        'pos': f"{palm_upper_stop_pos[0]} {palm_upper_stop_pos[1]} {palm_upper_stop_pos[2]}",
        'euler': '0 0 0',
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 0.7',  # Blue
        'group': '1'
    })
    
    # 2. Finger stop planes (RED) - attached to finger bodies
    # Find F1_L1 and F2_L1 bodies
    f1_l1_body = None
    f2_l1_body = None
    
    for body in mount_body.findall('.//body'):
        if body.get('name') == 'F1_L1':
            f1_l1_body = body
        elif body.get('name') == 'F2_L1':
            f2_l1_body = body
    
    if f1_l1_body is not None:
        # Current stop planes for finger 1 (RED)
        # Lower stop plane - positioned at current stop location
        finger_lower_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_finger_lower_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '0.015 0 0',  # Current lower stop position
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red, semi-transparent
            'group': '1'
        })
        
        # Upper stop plane
        finger_upper_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_finger_upper_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '-0.010 0 0',  # Current upper stop position
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
    
    if f2_l1_body is not None:
        # Current stop planes for finger 2 (RED)
        finger_lower_plane_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_finger_lower_plane_f2',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '0.015 0 0',
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
        
        finger_upper_plane_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_finger_upper_plane_f2',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '-0.010 0 0',
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
    
    # 3. Add "correct" position planes (GREEN) for comparison
    if f1_l1_body is not None:
        # Calculate correct positions
        data.qpos[f1_palm_id] = np.radians(-90)
        mujoco.mj_kinematics(model, data)
        finger_transform_lower = data.xpos[model.body('F1_L1').id]
        correct_lower_relative = palm_lower_stop_pos - finger_transform_lower
        
        data.qpos[f1_palm_id] = np.radians(25)
        mujoco.mj_kinematics(model, data)
        finger_transform_upper = data.xpos[model.body('F1_L1').id]
        correct_upper_relative = palm_upper_stop_pos - finger_transform_upper
        
        # Correct position planes (GREEN, semi-transparent)
        correct_lower_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_lower_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': f"{correct_lower_relative[0]} {correct_lower_relative[1]} {correct_lower_relative[2]}",
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 0.4',  # Green, more transparent
            'group': '1'
        })
        
        correct_upper_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_upper_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': f"{correct_upper_relative[0]} {correct_upper_relative[1]} {correct_upper_relative[2]}",
            'euler': '0 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 0.4',  # Green, more transparent
            'group': '1'
        })

# Save modified XML
viz_xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper_plane_visualization.xml')
tree.write(viz_xml_path, encoding='utf-8', xml_declaration=True)

print(f"\n✓ Plane visualization XML created: {viz_xml_path}")
print("\nPlane visualizations added:")
print("  🟦 BLUE planes: Palm stop planes (fixed)")
print("  🟥 RED planes: Current finger stop planes (move with fingers)")
print("  🟩 GREEN planes: Correct stop positions (ghost planes)")
print("\nAll planes are:")
print("  - 10cm x 10cm x 2mm thick")
print("  - Always visible (independent of collision)")
print("  - Semi-transparent for visibility")
print("  - Will clearly show overlap when stops engage")

print(f"\nTo use in simulation:")
print(f"  python3 run_plane_visualizer.py")
