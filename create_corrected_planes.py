#!/usr/bin/env python3
"""
Create XML with CORRECTLY oriented 2D plane visualization for mechanical stops.
Planes are perpendicular to finger movement for proper collision visualization.
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
print("CREATING CORRECTLY ORIENTED 2D PLANE VISUALIZATION")
print("="*80)
print("\n🟦 Blue planes: Palm stop planes (fixed, perpendicular)")
print("🟥 Red planes: Finger stop planes (move with fingers, perpendicular)")
print("🟩 Green planes: Correct stop positions (ghost planes, perpendicular)")
print("\n✅ OVERLAP: When planes intersect, stops are engaged")

# Load model to calculate positions
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Find the worldbody and mount body
worldbody = root.find('worldbody')
mount_body = None
for body in worldbody.findall('body'):
    if body.get('name') == 'mount':
        mount_body = body
        break

if mount_body is not None:
    print("Adding correctly oriented 2D plane visualizations...")
    
    # 1. Palm stop planes (BLUE) - fixed to mount body
    # These planes should be perpendicular to finger rotation axis (Y-axis)
    # So they should be oriented in XZ plane (euler = 0, 90, 0)
    
    # Lower stop plane
    palm_lower_plane = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_lower_plane',
        'type': 'box',
        'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
        'pos': '0.0855 0.03 0',  # Palm lower stop position
        'euler': '0 1.57 0',  # Rotate 90° around Y to be perpendicular to finger movement
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 0.7',  # Blue
        'group': '1'
    })
    
    # Upper stop plane
    palm_upper_plane = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_upper_plane',
        'type': 'box',
        'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
        'pos': '0.0655 0.03 0',  # Palm upper stop position
        'euler': '0 1.57 0',  # Rotate 90° around Y
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
        # These should also be perpendicular to finger movement
        finger_lower_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_finger_lower_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '0.015 0 0',  # Current lower stop position
            'euler': '0 1.57 0',  # Rotate 90° around Y
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
        
        finger_upper_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_finger_upper_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': '-0.010 0 0',  # Current upper stop position
            'euler': '0 1.57 0',  # Rotate 90° around Y
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
            'euler': '0 1.57 0',  # Rotate 90° around Y
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
            'euler': '0 1.57 0',  # Rotate 90° around Y
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
    
    # 3. Add "correct" position planes (GREEN) for comparison
    if f1_l1_body is not None:
        # Calculate correct positions (where planes SHOULD be for proper collision)
        f1_palm_id = model.joint('F1_palm_knuckle').id
        
        # At -90°, finger should engage with palm lower stop
        data.qpos[f1_palm_id] = np.radians(-90)
        mujoco.mj_kinematics(model, data)
        palm_lower_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")]
        finger_transform = data.xpos[model.body('F1_L1').id]
        correct_lower_relative = palm_lower_pos - finger_transform
        
        # At +25°, finger should engage with palm upper stop
        data.qpos[f1_palm_id] = np.radians(25)
        mujoco.mj_kinematics(model, data)
        palm_upper_pos = data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")]
        finger_transform = data.xpos[model.body('F1_L1').id]
        correct_upper_relative = palm_upper_pos - finger_transform
        
        # Correct position planes (GREEN)
        correct_lower_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_lower_plane_f1',
            'type': 'box',
            'size': '0.05 0.001 0.05',  # 10cm x 10cm x 2mm thick
            'pos': f"{correct_lower_relative[0]} {correct_lower_relative[1]} {correct_lower_relative[2]}",
            'euler': '0 1.57 0',  # Rotate 90° around Y
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
            'euler': '0 1.57 0',  # Rotate 90° around Y
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 0.4',  # Green
            'group': '1'
        })

# Save modified XML
viz_xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper_corrected_planes.xml')
tree.write(viz_xml_path, encoding='utf-8', xml_declaration=True)

print(f"\n✓ Corrected plane visualization XML created: {viz_xml_path}")
print("\nPlane orientations fixed:")
print("  🟦 BLUE planes: Palm stops (perpendicular to finger movement)")
print("  🟥 RED planes: Current finger stops (perpendicular, wrong position)")
print("  🟩 GREEN planes: Correct finger stops (perpendicular, right position)")
print("\nAll planes are now:")
print("  - Oriented perpendicular to finger rotation (euler='0 1.57 0')")
print("  - 10cm x 10cm x 2mm thick")
print("  - Will properly overlap when stops engage")

print(f"\nTo use in simulation:")
print(f"  python3 run_corrected_planes.py")
