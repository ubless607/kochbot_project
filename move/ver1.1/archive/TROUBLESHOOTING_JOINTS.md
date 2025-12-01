# Troubleshooting: MuJoCo Joints Showing Zero

## The Problem

You're seeing **all MuJoCo joint values at zero** even when the robot moves. This can happen due to several reasons:

### Common Causes

1. **Joint Mapping Issue**: Motor IDs don't correspond 1:1 with MuJoCo joints
2. **Joint Direction Inverted**: Motors rotate opposite to MuJoCo's expected direction
3. **Axis Configuration**: Physical robot axes don't match MuJoCo XML axis definitions
4. **Assembly Differences**: Parts assembled at different angles (e.g., 90° rotated)
5. **PWM Conversion Bug**: Formula not correctly converting PWM → radians

## Quick Diagnostic

### Step 1: Check if PWM is being read

```bash
python diagnose_joints.py --quick-test --port YOUR_PORT
```

Move joints and verify PWM values change. If they don't, it's a hardware/communication issue.

### Step 2: Check the conversion formula

The current formula in `interface.py`:
```python
def _pwm2pos(self, pwm: np.ndarray) -> np.ndarray:
    return (pwm / 2048 - 1) * 3.14
```

Test manually:
```python
# At PWM=2048 (center)
pos = (2048 / 2048 - 1) * 3.14  # Should be 0.0 ✓

# At PWM=2148 (+100 from center)
pos = (2148 / 2048 - 1) * 3.14  # Should be ~0.15 rad ✓
```

If this looks correct, the issue is likely **joint mapping or direction**.

## The Root Cause: Joint Mapping

### MuJoCo Joint Structure (from XML)

```
joint1: axis=[0,0,1] (Z-axis, base rotation)
joint2: axis=[1,0,0] (X-axis, shoulder)
joint3: axis=[1,0,0] (X-axis, elbow)
joint4: axis=[1,0,0] (X-axis, wrist pitch)
joint5: axis=[0,1,0] (Y-axis, wrist roll)
joint6: axis=[0,0,1] (Z-axis, gripper)
```

### Physical Robot Motor IDs

Your robot has motors with IDs 1-6, but:
- They might not correspond to MuJoCo joints 1-6
- They might rotate in opposite directions
- They might be mounted at different angles

## Solution: Map Motors to MuJoCo Joints

### Method 1: Automatic Diagnostic (Recommended)

```bash
python diagnose_joints.py --diagnose --port YOUR_PORT
```

This interactive tool will:
1. Ask you to move each motor ONE AT A TIME
2. Record which MuJoCo joint should correspond
3. Ask if direction is inverted
4. Generate configuration file automatically

### Method 2: Manual Configuration

Create `joint_mapping.json`:

```json
{
  "motor_to_mujoco": [0, 1, 2, 3, 4, 5],
  "joint_inversions": [false, false, false, false, false, false],
  "joint_offsets_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

**motor_to_mujoco**: Maps motor index → MuJoCo joint index
- Example: `[1, 0, 2, 3, 4, 5]` means motor 0 → joint 1, motor 1 → joint 0

**joint_inversions**: Flip direction for each motor
- `true` = multiply by -1 (invert)
- `false` = use as-is

**joint_offsets_rad**: Additional offset in radians for each motor
- Useful for small adjustments
- Different from calibration (which centers at 2048)

### Common Mapping Examples

#### Example 1: Motors and Joints Match, But Some Inverted
```json
{
  "motor_to_mujoco": [0, 1, 2, 3, 4, 5],
  "joint_inversions": [false, true, true, false, false, false],
  "joint_offsets_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

#### Example 2: Motors Reordered
```json
{
  "motor_to_mujoco": [0, 2, 1, 3, 5, 4],
  "joint_inversions": [false, false, false, false, false, false],
  "joint_offsets_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

#### Example 3: Complex Mapping with Offsets
```json
{
  "motor_to_mujoco": [0, 1, 2, 3, 4, 5],
  "joint_inversions": [false, true, true, false, true, false],
  "joint_offsets_rad": [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]
}
```
(joint2 is inverted AND offset by π/2 radians, meaning it's mounted 90° rotated)

## Using the Enhanced System

### Step 1: Run Diagnostic
```bash
python diagnose_joints.py --diagnose
```

This creates `joint_mapping_diagnostic.json`.

### Step 2: Test the Mapping
```bash
python sync_enhanced.py --joint-config joint_mapping_diagnostic.json
```

### Step 3: Fine-tune if Needed

Edit the JSON file manually and re-test:
```bash
# Test again
python sync_enhanced.py --joint-config joint_mapping_diagnostic.json
```

### Step 4: Combine with Calibration

Once mapping is correct, use with calibration:
```bash
# First calibrate (if not done)
python calibration.py --calibrate

# Then sync with both calibration AND mapping
python sync_enhanced.py --calibration robot_calibration.json --joint-config joint_mapping_diagnostic.json
```

## Understanding the Full Pipeline

```
Raw Robot PWM
    ↓
[Apply Calibration] (center at 2048)
    ↓
Calibrated PWM
    ↓
[Convert to Radians] (pwm/2048-1)*3.14
    ↓
Motor Radians
    ↓
[Apply Inversions] (flip if needed)
    ↓
[Add Offsets] (rotate if needed)
    ↓
[Remap to MuJoCo] (motor[i]→joint[j])
    ↓
MuJoCo Joint Positions
    ↓
Simulation Updates
```

## Modifying the XML (Alternative Approach)

If you want to avoid configuration files and bake the mapping into MuJoCo:

### Option 1: Change Joint Ranges/Axes

Edit `low_cost_robot/low_cost_robot.xml`:

```xml
<!-- If joint moves opposite direction -->
<joint name="joint2" axis="-1 0 0" class="joint2"/>

<!-- If joint is rotated 90° -->
<joint name="joint2" axis="0 1 0" class="joint2"/>
```

### Option 2: Add Joint Offsets in XML

```xml
<joint name="joint2" axis="1 0 0" range="-3.14158 3.14158" ref="1.57"/>
```

The `ref` attribute shifts the zero position.

### Option 3: Reorder Actuators

```xml
<actuator>
    <position class="joint1" name="joint1" joint="joint1"/>
    <position class="joint3" name="joint2" joint="joint3"/>  <!-- Swapped -->
    <position class="joint2" name="joint3" joint="joint2"/>  <!-- Swapped -->
    ...
</actuator>
```

## Recommendations

1. **Start with diagnostic tool** - It's interactive and saves time
2. **Use JSON config first** - Easier to iterate than XML
3. **Test one joint at a time** - Move only one motor, verify MuJoCo response
4. **Only modify XML if needed** - For permanent/shared setups

## Quick Reference Commands

```bash
# Diagnose the problem
python diagnose_joints.py --quick-test        # See PWM values
python diagnose_joints.py --diagnose          # Full mapping diagnostic

# Test solutions
python sync_enhanced.py                       # Basic (no config)
python sync_enhanced.py --joint-config FILE   # With joint mapping
python sync_enhanced.py --calibration FILE --joint-config FILE  # Full setup

# Legacy (uses original interface.py)
python sync_simul_real.py                     # Original, no mapping
python sync_simul_real_calibrated.py          # With calibration only
```

## Still Not Working?

If joints still show zero:

1. **Check MuJoCo is actually updating**
   - Add print statements to see if `d.qpos` changes
   - Verify `mujoco.mj_step(m, d)` is being called

2. **Verify array slicing**
   - Code uses `d.qpos[:6]` - make sure model has 6+ joints
   - Check with `print(len(d.qpos))` and `print(m.nq)`

3. **Check for overwrites**
   - Make sure nothing else is setting `d.qpos` to zero
   - Look for `d.qpos = 0` or similar

4. **Test with known values**
   ```python
   # Force specific values
   d.qpos[:6] = np.array([0.5, 0.0, -0.5, 0.0, 0.0, 0.0])
   mujoco.mj_step(m, d)
   viewer.sync()
   ```
   If robot still doesn't move in MuJoCo, it's a MuJoCo setup issue.
