# EZGripper Integration Guide

This guide explains how to integrate the EZGripper MuJoCo model into your robot simulation.

## Quick Integration

### Step 1: Choose the Right Model File

- **`ezgripper.xml`** - Complete model with worldbody and floor (for standalone testing)
- **`ezgripper_only.xml`** - Gripper-only model (for robot integration)
- **`models/working/ezgripper_working.xml`** - Reference baseline model

### Step 2: Include the Gripper

For robot integration, use `ezgripper_only.xml`:

```xml
<mujoco model="your_robot">
  <compiler angle="radian"/>
  
  <!-- Your robot model -->
  <worldbody>
    <!-- Your robot base and links -->
    
    <!-- Attach EZGripper to end effector -->
    <body name="end_effector" pos="0 0 0.5">
      <body name="ezgripper_mount" pos="0 0 -0.05" quat="1 0 0 0">
        <include file="path/to/ezgripper_only.xml"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

### Step 3: Adjust Position and Orientation

The gripper defaults to:
- **Forward direction**: +X axis
- **Finger plane**: XY plane
- **Mount position**: Origin of gripper body

Common orientations:
```xml
<!-- Default: fingers forward -->
<quat>1 0 0 0</quat>

<!-- Rotate 90° around Z: fingers right -->
<quat>0.707 0 0 0.707</quat>

<!-- Rotate 90° around Y: fingers down -->
<quat>0.707 0 0.707 0</quat>
```

## Control Integration

### Actuator Control

The gripper has a single actuator: `gripper_actuator`

```python
import mujoco

# Load your integrated model
model = mujoco.MjModel.from_xml_path("your_robot_with_gripper.xml")
data = mujoco.MjData(model)

# Get actuator ID
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')

# Control the gripper
# Negative values = open (assist springs)
# Positive values = close (pull tendon)
data.ctrl[gripper_actuator_id] = 0.5  # Close with moderate force
```

### Joint Monitoring

Monitor gripper joint positions for feedback:

```python
# Get joint IDs
f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

# Read joint angles (in radians)
f1_palm_angle = data.qpos[f1_palm_id]
f2_palm_angle = data.qpos[f2_palm_id]
f1_tip_angle = data.qpos[f1_tip_id]
f2_tip_angle = data.qpos[f2_tip_id]

# Convert to degrees
import numpy as np
f1_palm_deg = np.rad2deg(f1_palm_angle)
f2_palm_deg = np.rad2deg(f2_palm_angle)
```

## Grasping Objects

### Adding Graspable Objects

```xml
<worldbody>
  <!-- Your robot with gripper -->
  
  <!-- Add object to grasp -->
  <body name="grasp_object" pos="0.3 0 0.1">
    <freejoint/>  <!-- Allow object to move -->
    <geom name="object_geom" type="box" size="0.02 0.02 0.02" 
          mass="0.1" friction="1.0 0.1 0.01"/>
  </body>
</worldbody>
```

### Contact Configuration

The gripper model includes contact pairs for finger-to-finger interaction. For object grasping, ensure:

```xml
<contact>
  <!-- Existing finger contacts -->
  
  <!-- Add object contacts if needed -->
  <pair geom1="f1_tip" geom2="object_geom" condim="6"/>
  <pair geom2="f2_tip" geom2="object_geom" condim="6"/>
</contact>
```

## Troubleshooting

### Gripper Not Closing

1. **Check actuator control**: Ensure positive values for closing
2. **Verify tendon routing**: Check tendon paths in XML
3. **Joint limits**: Ensure joints aren't at limits

### Asymmetric Finger Motion

This is **expected behavior** during contact:
- Small initial differences get amplified when fingers contact objects
- This mimics real hardware behavior
- The simulation is physically accurate

### Integration Issues

1. **Path problems**: Use absolute paths or correct relative paths
2. **Mesh loading**: Ensure mesh files are accessible
3. **Joint naming**: Check joint names match your control code

## Advanced Configuration

### Adjusting Spring Behavior

Modify joint spring parameters in the XML:

```xml
<joint name="F1_palm_knuckle" 
       stiffness="0.05" 
       springref="-0.52"/>
```

### Changing Damping

Adjust joint damping for different dynamics:

```xml
<joint name="F1_palm_knuckle" 
       damping="0.005"/>
```

### Modifying Joint Limits

Change joint ranges if needed:

```xml
<joint name="F1_palm_knuckle" 
       range="-1.57 0.27"/>
```

## Examples

See the test files for complete working examples:
- `tests/test_grasp_cylinder.py` - Wrapping grasp example
- `tests/test_pinch_cylinder.py` - Pinch grasp example
- `tests/test_active_closing.py` - Actuation example

## Support

For integration issues:
- Check the test files for reference implementations
- Review the main README for model specifications
- Contact SAKE Robotics support for hardware-specific questions