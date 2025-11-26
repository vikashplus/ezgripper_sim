#!/usr/bin/env python3
import mujoco

# Load model and check available attributes
model = mujoco.MjModel.from_xml_path('ezgripper.xml')

print("Available joint-related attributes:")
for attr in dir(model):
    if 'spring' in attr.lower() or 'jnt' in attr.lower():
        print(f"  {attr}")

print("\nModel structure details:")
print(f"njnt: {model.njnt}")
print(f"Joint attributes: {[attr for attr in dir(model) if attr.startswith('jnt_')]}")
