#!/usr/bin/env python3
"""
Create XML with correct 2D plane visualization for mechanical stops.
- Blue planes: Fixed stop planes (in XY plane, don't rotate)
- Red planes: Finger planes (rotate with fingers, in XY plane)  
- Green planes: Reference showing correct red plane positions (fixed, don't rotate)
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
print("CREATING CORRECT 2D PLANE VISUALIZATION")
print("="*80)
print("\n🟦 Blue planes: Fixed stop planes (XY plane, don't rotate)")
print("🟥 Red planes: Finger planes (rotate with fingers, XY plane)")
print("🟩 Green planes: Reference showing correct red positions (fixed, XY plane)")
print("\n✅ OVERLAP: When red planes align with blue/green planes = STOP ENGAGED")

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
    print("Adding correct 2D plane visualizations...")
    
    # 1. Blue planes: Fixed stop planes (don't rotate with fingers)
    # These should be in XY plane and fixed to mount body
    
    # All planes should be parallel and overlapping at the L1-Palm joint axis
    # Finger 1 side (positive Y side)
    green_plane_f1 = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_green_plane_f1',
        'type': 'box',
        'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
        'pos': '0.07255 0.03 0',  # L1-Palm joint center for finger 1
        'euler': '0 0 1.57',  # Parallel to global Z axis
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 1 0 0.7',  # Green
        'group': '1'
    })
    
    blue_plane_f1 = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_blue_plane_f1',
        'type': 'box',
        'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
        'pos': '0.07255 0.03 0',  # L1-Palm joint center for finger 1
        'euler': '0 0 1.57',  # Parallel to global Z axis
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 0.7',  # Blue
        'group': '1'
    })
    
    # Finger 2 side (negative Y side) - mirrored
    green_plane_f2 = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_green_plane_f2',
        'type': 'box',
        'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
        'pos': '0.07255 -0.03 0',  # L1-Palm joint center for finger 2
        'euler': '0 0 1.57',  # Parallel to global Z axis
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 1 0 0.7',  # Green
        'group': '1'
    })
    
    blue_plane_f2 = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_blue_plane_f2',
        'type': 'box',
        'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
        'pos': '0.07255 -0.03 0',  # L1-Palm joint center for finger 2
        'euler': '0 0 1.57',  # Parallel to global Z axis
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 0.7',  # Blue
        'group': '1'
    })
    
    # 2. Red planes: Finger planes (rotate with fingers)
    # Find F1_L1 and F2_L1 bodies
    f1_l1_body = None
    f2_l1_body = None
    
    for body in mount_body.findall('.//body'):
        if body.get('name') == 'F1_L1':
            f1_l1_body = body
        elif body.get('name') == 'F2_L1':
            f2_l1_body = body
    
    if f1_l1_body is not None:
        # Red plane for finger 1 - moves with finger but stays at joint center, parallel to global Z
        finger_plane_f1 = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_finger_plane_f1',
            'type': 'box',
            'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
            'pos': '0 0 0',  # At finger joint center (relative to F1_L1 body)
            'euler': '0 0 1.57',  # Parallel to global Z axis
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
    
    if f2_l1_body is not None:
        # Red plane for finger 2 - moves with finger but stays at joint center, parallel to global Z
        finger_plane_f2 = ET.SubElement(f2_l1_body, 'geom', {
            'name': 'viz_finger_plane_f2',
            'type': 'box',
            'size': '0.001 0.05 0.05',  # 2mm x 10cm x 10cm thin plane parallel to global Z
            'pos': '0 0 0',  # At finger joint center (relative to F2_L1 body)
            'euler': '0 0 1.57',  # Parallel to global Z axis
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 0.7',  # Red
            'group': '1'
        })
    
    # 3. No reference planes needed - green is used for inner stop, blue for full open stop

# Save modified XML
viz_xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper_correct_plane_setup.xml')
tree.write(viz_xml_path, encoding='utf-8', xml_declaration=True)

print(f"\n✓ Correct plane visualization XML created: {viz_xml_path}")
print("\nPlane setup:")
print("  🟩 GREEN planes: Inner stop (L1 to palm) - when finger is closed/minimum angle")
print("  🟦 BLUE planes: Full open stop (L1 to palm) - when finger is fully open/maximum angle")
print("  🟥 RED planes: Finger plane - rotates from parallel to GREEN (closed) to parallel to BLUE (open)")
print("\nHow it works:")
print("  - Red planes rotate with finger joints")
print("  - When red plane aligns with GREEN plane = INNER STOP ENGAGED (finger closed)")
print("  - When red plane aligns with BLUE plane = FULL OPEN STOP ENGAGED (finger fully open)")
print("  - All planes are parallel to global Z axis")

print(f"\nTo use in simulation:")
print(f"  python3 run_correct_plane_setup.py")
