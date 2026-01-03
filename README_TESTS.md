# EZGripper Test Benches

This directory contains standard test benches for the EZGripper simulation.

## Standard Test Benches

### 1. `test_passive_springs.py` - Passive Spring Behavior
**Purpose:** Verify spring preload and mechanical stops without tendon actuation.

**What it tests:**
- Springs push gripper to open position
- Joint limits are enforced by mechanical stops
- No oscillation (proper damping)
- Steady state at expected open position

**Expected Results:**
- Palm-L1: ~-90° (at lower limit)
- L1-L2: ~0° (at lower limit)
- No limit violations
- Stable (velocities → 0)

**Run:** `python3 test_passive_springs.py`

---

### 2. `test_active_closing.py` - Active Tendon Closing
**Purpose:** Verify tendon actuation and underactuated closing behavior.

**What it tests:**
- Tendon pulls gripper closed
- Underactuated joint sequencing (Palm-L1 then L1-L2)
- Joint limits enforced during closing
- Stable closed position

**Expected Results:**
- Phase 1: Stable open position
- Phase 2: Smooth closing motion
- Phase 3: Stable closed position
- All joints within limits

**Run:** `python3 test_active_closing.py`

---

## Test Philosophy

**DO NOT create new test files for every experiment.**

Instead:
1. Use these standard test benches to verify changes
2. If you need to test something specific, modify one of these temporarily
3. After testing, revert changes to keep test benches clean
4. Only create a new test bench if it's a fundamentally different test case

## Old Test Files

All experimental/one-off test files have been moved to `old_tests/` directory.
These are kept for reference but should not be used going forward.

## Adding New Standard Tests

Only add a new standard test bench if:
1. It tests a fundamentally different aspect of the gripper
2. It will be used repeatedly during development
3. It has clear pass/fail criteria
4. It's well-documented

Examples of valid new test benches:
- `test_object_grasp.py` - Test grasping various objects
- `test_force_limits.py` - Test actuator force limiting
- `test_collision_detection.py` - Test finger-to-finger collisions
