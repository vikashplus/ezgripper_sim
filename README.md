# EZGripper MuJoCo Simulation

High-fidelity [MuJoCo](https://mujoco.org) model for the **SAKE Robotics EZGripper** robotic gripper with realistic tendon-driven physics.

## Overview

This repository contains a physics-accurate MuJoCo model of the SAKE Robotics EZGripper Dual (Gen1/Gen2), featuring:

- ✅ **Tendon-driven mechanism** - Realistic cable actuation matching real hardware
- ✅ **Under-actuated fingers** - Single servo controls 2 opposing fingers
- ✅ **Adaptive grasping** - Fingers wrap around objects independently
- ✅ **Accurate meshes** - Real STL geometry from SAKE Robotics
- ✅ **MuJoCo 3.0+ compatible** - Works with latest MuJoCo versions

## Quick Start

### Installation

```bash
# Install MuJoCo (if not already installed)
pip3 install mujoco

# Clone this repository
git clone https://github.com/SAKErobotics/MuJoCo_ezgripper_sim.git
cd MuJoCo_ezgripper_sim
```

### Running Tests

```bash
# Test 1: Wrapping grasp demonstration
python3 tests/test_grasp_cylinder.py

# Test 2: Pinch grasp demonstration  
python3 tests/test_pinch_cylinder.py

# Test 3: Basic spring behavior verification
python3 tests/test_passive_springs.py

# Test 4: Tendon actuation demonstration
python3 tests/test_active_closing.py
```

## Files

### Model Files
- `ezgripper.xml` - Main working model with cylinder object for grasping demos
- `ezgripper_only.xml` - Gripper-only model for robot integration (no worldbody)
- `models/working/ezgripper_working.xml` - Reference baseline model

### Test Files
- `tests/test_grasp_cylinder.py` - Demonstrates wrapping grasp behavior
- `tests/test_pinch_cylinder.py` - Demonstrates pinch grasp behavior
- `tests/test_passive_springs.py` - Verifies spring behavior without actuation
- `tests/test_active_closing.py` - Demonstrates tendon-driven closing

### Mesh Files
- `meshes/` - All STL mesh files for gripper geometry

## EZGripper Specifications

### Mechanical Design

**Architecture**: Under-actuated, tendon-driven
- **Fingers**: 2 opposing fingers
- **Links per finger**: 2 (proximal + distal)
- **Joints per finger**: 2 (~110° range each)
- **Actuation**: Single Dynamixel MX-64AR servo

**Range of Motion**:
- **Full range**: 2500 servo units (0 = closed, 2500 = open)
- **Scaled range**: 0-100% (used in control interface)
- **Joint angles**: ~110° per joint

### Control Interface

The EZGripper uses two primary commands:

#### 1. Calibrate
```python
calibrate(servo)
```
- Moves gripper to closed position
- Resets zero position
- Should be run at startup

#### 2. Goto_Position(effort, position)
```python
goto_position(effort, position)
```

**Position Parameter** (0-100):
- `0`: Fully closed (continuous close mode)
- `1-100`: Specific position (% open)
- `100`: Fully open

**Effort Parameter** (0-100):
- `0`: Torque off (fingers can be back-driven)
- `1-100`: Torque level (% of max)

**Control Modes**:
- **Position = 0**: Torque control mode (best for grasping)
- **Position > 0**: Position control mode (best for positioning)

**Automatic Back-off Algorithm**:
- Closes until fingers stop moving
- Drops to 10% torque to maintain grasp
- Minimizes power and heat

### Physics Model

**Tendon System**:
- Spatial tendon routing through pulleys
- Realistic cable mechanics
- Spring-loaded opening (tendon closes, springs open)

**Contact Dynamics**:
- Finger-to-finger contact
- Finger-to-object contact
- Adaptive wrapping behavior

**Joint Properties**:
- Damping: 0.005
- Spring references for natural positions
- Limited ranges matching real hardware

## Integration with Robots

### Attaching to Robot Arm

To integrate the EZGripper into your robot model:

1. **Include the gripper body** (without worldbody):
```xml
<include file="ezgripper_only.xml"/>
```

2. **Attach to your end-effector**:
```xml
<body name="your_end_effector">
  <body name="ezgripper" pos="0 0 -0.05" quat="...">
    <!-- Include gripper here -->
  </body>
</body>
```

3. **Orient the gripper**:
   - Default: Gripper X-axis points forward
   - Rotate as needed for your application

## MuJoCo Version Compatibility

- ✅ **MuJoCo 3.0+**: Fully tested and supported
- ✅ **MuJoCo 2.x**: Should work (not actively tested)

**Tested with**:
- MuJoCo 3.3.7 on Ubuntu 24.04
- Python 3.12

## Credits

**Original Model**: [Vikash Kumar](https://github.com/vikashplus/ezgripper_sim) (2021)
**Hardware**: [SAKE Robotics](https://sakerobotics.com/)
**Maintained by**: SAKE Robotics

## License

Apache License 2.0 - See LICENSE file

## Resources

- **SAKE Robotics**: https://sakerobotics.com/
- **EZGripper Hardware**: https://sakerobotics.com/products
- **Python Drivers**: https://github.com/SAKErobotics/libezgripper
- **ROS 2 Drivers**: https://github.com/SAKErobotics/EZGripper_ros2
- **MuJoCo Documentation**: https://mujoco.readthedocs.io/

## Support

For questions or issues:
- GitHub Issues: https://github.com/SAKErobotics/MuJoCo_ezgripper_sim/issues
- Email: support@sakerobotics.com