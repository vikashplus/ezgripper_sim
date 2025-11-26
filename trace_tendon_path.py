#!/usr/bin/env python3
"""
Trace the complete tendon path and show world positions of all sites/geoms
"""
import mujoco
import numpy as np
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Reset to initial position
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)

print("=" * 80)
print("TENDON PATH TRACE - World Positions")
print("=" * 80)

# Read the tendon routing from the XML to understand the path
print("\nTENDON ROUTING (from ezgripper.xml):")
print("-" * 80)

# Main path before split
path = [
    ("SITE", "palm_peg0", "Main path start"),
    ("GEOM", "palm_split_pulley", "Split pulley"),
    ("SITE", "palm_split", "Split point"),
    ("PULLEY", "divisor=2", "--- SPLIT INTO TWO BRANCHES ---"),
]

# Branch 1 (Finger 1)
branch1 = [
    ("", "", ""),
    ("SITE", "palm_split", "Branch 1 start (F1)"),
    ("SITE", "palm_peg1", ""),
    ("GEOM", "palm_pulley_f1", "sidesite=palm_peg2"),
    ("SITE", "f1l1_peg0", ""),
    ("GEOM", "f1l1_pulley", "sidesite=f1l1_pulleyside"),
    ("SITE", "f1l1_peg1", ""),
    ("GEOM", "f1l2_pulley", ""),
    ("SITE", "f1l2_peg", ""),
    ("SITE", "f1l2_pin", "Branch 1 end"),
]

# Branch 2 (Finger 2)
branch2 = [
    ("PULLEY", "divisor=2", "--- SECOND BRANCH ---"),
    ("SITE", "palm_split", "Branch 2 start (F2)"),
    ("SITE", "palm_peg1", ""),
    ("GEOM", "palm_pulley_f2", "sidesite=palm_peg2"),
    ("SITE", "f2l1_peg0", ""),
    ("GEOM", "f2l1_pulley", "sidesite=f2l1_pulleyside"),
    ("SITE", "f2l1_peg1", ""),
    ("GEOM", "f2l2_pulley", ""),
    ("SITE", "f2l2_peg", ""),
    ("SITE", "f2l2_pin", "Branch 2 end"),
]

def get_site_world_pos(site_name):
    """Get world position of a site"""
    try:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        return data.site_xpos[site_id].copy()
    except:
        return None

def get_geom_world_pos(geom_name):
    """Get world position of a geom"""
    try:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        return data.geom_xpos[geom_id].copy()
    except:
        return None

def print_element(elem_type, elem_name, note, index):
    """Print element with world position"""
    if elem_type == "PULLEY":
        print(f"\n{index:2d}. {elem_type:6s} {elem_name:20s} {note}")
        return
    
    if elem_type == "":
        print()
        return
    
    if elem_type == "SITE":
        pos = get_site_world_pos(elem_name)
    elif elem_type == "GEOM":
        pos = get_geom_world_pos(elem_name)
    else:
        pos = None
    
    if pos is not None:
        print(f"{index:2d}. {elem_type:6s} {elem_name:20s} [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}]  {note}")
    else:
        print(f"{index:2d}. {elem_type:6s} {elem_name:20s} [NOT FOUND]  {note}")

# Print main path
index = 1
for elem in path:
    print_element(elem[0], elem[1], elem[2], index)
    if elem[0] != "PULLEY" and elem[0] != "":
        index += 1

# Print branch 1
for elem in branch1:
    print_element(elem[0], elem[1], elem[2], index)
    if elem[0] != "PULLEY" and elem[0] != "":
        index += 1

# Print branch 2
for elem in branch2:
    print_element(elem[0], elem[1], elem[2], index)
    if elem[0] != "PULLEY" and elem[0] != "":
        index += 1

# Calculate total path lengths
print("\n" + "=" * 80)
print("PATH LENGTH ANALYSIS")
print("=" * 80)

def calc_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    if pos1 is None or pos2 is None:
        return None
    return np.linalg.norm(pos2 - pos1)

# Branch 1 length
print("\nBranch 1 (Finger 1) segments:")
b1_sites = ["palm_split", "palm_peg1", "f1l1_peg0", "f1l1_peg1", "f1l2_peg", "f1l2_pin"]
b1_positions = [get_site_world_pos(s) for s in b1_sites]
b1_total = 0
for i in range(len(b1_sites)-1):
    dist = calc_distance(b1_positions[i], b1_positions[i+1])
    if dist is not None:
        print(f"  {b1_sites[i]:15s} -> {b1_sites[i+1]:15s}: {dist*1000:.2f} mm")
        b1_total += dist

print(f"\n  Total Branch 1 length: {b1_total*1000:.2f} mm")

# Branch 2 length
print("\nBranch 2 (Finger 2) segments:")
b2_sites = ["palm_split", "palm_peg1", "f2l1_peg0", "f2l1_peg1", "f2l2_peg", "f2l2_pin"]
b2_positions = [get_site_world_pos(s) for s in b2_sites]
b2_total = 0
for i in range(len(b2_sites)-1):
    dist = calc_distance(b2_positions[i], b2_positions[i+1])
    if dist is not None:
        print(f"  {b2_sites[i]:15s} -> {b2_sites[i+1]:15s}: {dist*1000:.2f} mm")
        b2_total += dist

print(f"\n  Total Branch 2 length: {b2_total*1000:.2f} mm")

# Symmetry check
print("\n" + "=" * 80)
print("SYMMETRY CHECK")
print("=" * 80)
print(f"Branch 1 length: {b1_total*1000:.2f} mm")
print(f"Branch 2 length: {b2_total*1000:.2f} mm")
print(f"Difference: {abs(b1_total - b2_total)*1000:.2f} mm")
if abs(b1_total - b2_total) < 0.0001:
    print("✓ Branches are SYMMETRIC")
else:
    print("✗ Branches are ASYMMETRIC")

print("\n" + "=" * 80)
