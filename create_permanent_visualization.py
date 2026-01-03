#!/usr/bin/env python3
"""
Create XML with permanent visual markers for mechanical stops.
These markers are always visible regardless of collision state.
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
print("CREATING PERMANENT VISUAL MARKERS FOR MECHANICAL STOPS")
print("="*80)

# Load model to calculate positions
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Calculate positions for visual markers
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
    print("Adding permanent visual markers...")
    
    # 1. Palm stop markers (BLUE) - fixed to mount body
    palm_lower_marker = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_lower_permanent',
        'type': 'sphere',
        'size': '0.004',  # Slightly larger for visibility
        'pos': f"{palm_lower_stop_pos[0]} {palm_lower_stop_pos[1]} {palm_lower_stop_pos[2]}",
        'contype': '0',  # No collision
        'conaffinity': '0',
        'rgba': '0 0 1 0.8',  # Blue, slightly transparent
        'group': '1'  # Visualization group
    })
    
    palm_upper_marker = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_upper_permanent',
        'type': 'sphere',
        'size': '0.004',
        'pos': f"{palm_upper_stop_pos[0]} {palm_upper_stop_pos[1]} {palm_upper_stop_pos[2]}",
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 0.8',  # Blue
        'group': '1'
    })
    
    # 2. Current finger stop markers (RED) - attached to finger bodies
    # Find F1_L1 and F2_L1 bodies
    f1_l1_body = None
    f2_l1_body = None
    
    for body in mount_body.findall('.//body'):
        if body.get('name') == 'F1_L1':
            f1_l1_body = body
        elif body.get('name') == 'F2_L1':
            f2_l1_body = body
    
    if f1_l1_body is not None:
        # Current stop markers for finger 1 (RED)
        current_lower_marker_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_current_lower_f1_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': '0.015 0 0',  # Current lower stop position
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.8',  # Red
            'group': '1'
        })
        
        current_upper_marker_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_current_upper_f1_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': '-0.010 0 0',  # Current upper stop position
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.8',  # Red
            'group': '1'
        })
        
        # Add connecting lines to show stop "reach"
        lower_line_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_reach_lower_f1',
            'type': 'cylinder',
            'size': '0.001 0.020',  # Thin cylinder, 20mm long
            'pos': '0.015 0 0',
            'quat': '0 0 0 1',  # Default orientation
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0.5 0.5 0.5',  # Light red, transparent
            'group': '1'
        })
        
        upper_line_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_reach_upper_f1',
            'type': 'cylinder',
            'size': '0.001 0.015',  # Thin cylinder, 15mm long
            'pos': '-0.010 0 0',
            'quat': '0 0 0 1',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0.5 0.5 0.5',  # Light red, transparent
            'group': '1'
        })
    
    if f2_l1_body is not None:
        # Current stop markers for finger 2 (RED)
        current_lower_marker_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_current_lower_f2_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': '0.015 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.8',  # Red
            'group': '1'
        })
        
        current_upper_marker_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_current_upper_f2_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': '-0.010 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.8',  # Red
            'group': '1'
        })
        
        # Add connecting lines for finger 2
        lower_line_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_reach_lower_f2',
            'type': 'cylinder',
            'size': '0.001 0.020',
            'pos': '0.015 0 0',
            'quat': '0 0 0 1',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0.5 0.5 0.5',
            'group': '1'
        })
        
        upper_line_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_reach_upper_f2',
            'type': 'cylinder',
            'size': '0.001 0.015',
            'pos': '-0.010 0 0',
            'quat': '0 0 0 1',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0.5 0.5 0.5',
            'group': '1'
        })
    
    # 3. Add "ghost" markers showing where stops SHOULD be (GREEN)
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
        
        # Correct position markers (GREEN, semi-transparent)
        correct_lower_marker_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_lower_f1_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': f"{correct_lower_relative[0]} {correct_lower_relative[1]} {correct_lower_relative[2]}",
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 0.4',  # Green, more transparent
            'group': '1'
        })
        
        correct_upper_marker_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_upper_f1_permanent',
            'type': 'sphere',
            'size': '0.004',
            'pos': f"{correct_upper_relative[0]} {correct_upper_relative[1]} {correct_upper_relative[2]}",
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 0.4',  # Green, more transparent
            'group': '1'
        })

# Save modified XML
viz_xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper_permanent_visualization.xml')
tree.write(viz_xml_path, encoding='utf-8', xml_declaration=True)

print(f"\n✓ Permanent visualization XML created: {viz_xml_path}")
print("\nVisual markers added:")
print("  🔵 BLUE spheres: Palm stop positions (fixed)")
print("  🔴 RED spheres: Current finger stop positions (move with fingers)")
print("  🟢 GREEN spheres: Correct stop positions (ghost markers)")
print("  📏 Red cylinders: Stop reach indicators")
print("\nAll markers are:")
print("  - Always visible (independent of collision)")
print("  - Non-colliding (contype='0')")
print("  - In visualization group (group='1')")
print("  - Semi-transparent for visibility")

print(f"\nTo use in simulation:")
print(f"  python3 run_permanent_visualizer.py")
