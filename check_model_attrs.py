#!/usr/bin/env python3
"""
Check MuJoCo model attributes for spring configuration.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)

print("="*80)
print("MUJOCO MODEL ATTRIBUTES")
print("="*80)

# Get joint ID
f1_palm_id = model.joint('F1_palm_knuckle').id

print(f"\nJoint ID: {f1_palm_id}")
print(f"Model attributes containing 'spring':")
for attr in dir(model):
    if 'spring' in attr.lower():
        print(f"  {attr}: {getattr(model, attr)}")

print(f"\nModel attributes containing 'stiff':")
for attr in dir(model):
    if 'stiff' in attr.lower():
        print(f"  {attr}: {getattr(model, attr)}")

print(f"\nModel attributes containing 'jnt':")
for attr in dir(model):
    if 'jnt' in attr.lower():
        try:
            val = getattr(model, attr)
            if hasattr(val, '__len__') and len(val) > f1_palm_id:
                print(f"  {attr}[{f1_palm_id}]: {val[f1_palm_id]}")
        except:
            pass

# Check joint properties directly
print(f"\n" + "="*50)
print("JOINT PROPERTIES:")
print("="*50)

joint = model.joint(f1_palm_id)
print(f"Joint object attributes:")
for attr in dir(joint):
    if not attr.startswith('_'):
        try:
            val = getattr(joint, attr)
            print(f"  {attr}: {val}")
        except:
            print(f"  {attr}: <unable to access>")

print("="*80)
