#!/usr/bin/env python3
"""
Check if XML spring definition is being parsed correctly.
"""

import mujoco
import numpy as np
import os

print("="*80)
print("XML PARSING CHECK")
print("="*80)

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)

# Check the raw XML content
print(f"\nChecking XML joint definition...")

# Read the XML file directly
with open(model_path, 'r') as f:
    xml_content = f.read()

# Find the joint definition
import re
joint_pattern = r'<joint[^>]*name="F1_palm_knuckle"[^>]*>.*?</joint>'
joint_match = re.search(joint_pattern, xml_content, re.DOTALL)

if joint_match:
    joint_xml = joint_match.group(0)
    print(f"Found joint definition:")
    print(joint_xml)
else:
    print("Joint definition not found!")

# Check all joint definitions
print(f"\n" + "="*50)
print("ALL JOINT DEFINITIONS:")
print("="*50)

all_joints = re.findall(r'<joint[^>]*name="[^"]*"[^>]*>', xml_content)
for joint in all_joints:
    if 'stiffness' in joint or 'springref' in joint:
        print(joint)

# Check if there are any actuator springs that might override
print(f"\n" + "="*50)
print("ACTUATOR DEFINITIONS:")
print("="*50)

actuator_pattern = r'<actuator>.*?</actuator>'
actuator_match = re.search(actuator_pattern, xml_content, re.DOTALL)

if actuator_match:
    actuators_xml = actuator_match.group(0)
    print(actuators_xml)
else:
    print("No actuators found (tendons disabled)")

print("="*80)
