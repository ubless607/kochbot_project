# Robot Calibration System for MuJoCo Synchronization

This calibration system ensures that your real robot's joint positions properly align with the MuJoCo simulator's coordinate system **without requiring you to physically match the simulator's pose**.

## Overview

The calibration system solves a critical problem: **motors can rotate 360° and may be assembled at different absolute positions**. Trying to force the physical robot to match MuJoCo's zero pose could damage the robot or be physically impossible.

### The Solution

Instead of matching poses, we:
1. **Define a safe "home position"** on your physical robot
2. **Map this home position to MuJoCo's zero** (qpos = 0)
3. **Track relative movement** from there

This means:
- ✅ Your robot stays in a safe configuration
- ✅ No need to disassemble or adjust motor positions
- ✅ MuJoCo displays movement relative to your chosen home pose
- ✅ The visual appearance in MuJoCo represents your robot's movements accurately

## Quick Start

### 1. Run Calibration (First Time Setup)

```bash
python calibration.py --calibrate --port /dev/tty.usbmodem58760430141
```

**What happens during calibration:**
1. Robot torque is disabled (safe to move manually)
2. You position the robot in a **safe, comfortable home position** (your choice!)
3. System records this as the reference (does NOT need to match MuJoCo visually)
4. Offsets are calculated: `offset = 2048 - your_home_position`
5. Calibration saved to `robot_calibration.json`

**Choosing a Home Position:**
- Pick something safe and repeatable (e.g., arms straight down, T-pose, etc.)
- Should be comfortable for the robot's mechanical limits
- Easy to visually identify and return to
- Does NOT need to look like MuJoCo's default pose!

### 2. Test Calibration

```bash
python calibration.py --test --port /dev/tty.usbmodem58760430141
```

This displays real-time raw vs. calibrated values as you move the robot.

### 3. Use Calibration with MuJoCo

```bash
python calibration.py --demo --port /dev/tty.usbmodem58760430141
```

Or use the enhanced sync script:

```bash
python sync_simul_real_calibrated.py
```

## Using Calibration in Your Code

### Basic Usage

```python
from robot import Robot
from calibration import RobotCalibration

# Connect to robot
robot = Robot(device_name='/dev/tty.usbmodem58760430141')
robot._disable_torque()

# Load calibration
calibration = RobotCalibration()
calibration.load_calibration()

# Read and apply calibration
raw_pwm = robot.read_position()
calibrated_pwm = calibration.apply_calibration(raw_pwm)
```

### With MuJoCo Synchronization

```python
import mujoco
import numpy as np
from robot import Robot
from interface import SimulatedRobot
from calibration import RobotCalibration

# Setup
m = mujoco.MjModel.from_xml_path('low_cost_robot/scene.xml')
d = mujoco.MjData(m)
sim_robot = SimulatedRobot(m, d)

robot = Robot(device_name='/dev/tty.usbmodem58760430141')
robot._disable_torque()

calibration = RobotCalibration()
calibration.load_calibration()

# Sync loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        # Read and calibrate
        raw_pwm = np.array(robot.read_position())
        calibrated_pwm = calibration.apply_calibration(raw_pwm)
        
        # Update MuJoCo
        d.qpos[:6] = sim_robot._pwm2pos(calibrated_pwm)
        mujoco.mj_step(m, d)
        viewer.sync()
```

## How It Works

### The Math

```
Calibrated PWM = Raw PWM + Offset
where Offset = 2048 - Home Position PWM
```

**Example:**
- Your home position: Joint 1 is at PWM 2100
- Offset: 2048 - 2100 = -52
- When at home: Raw=2100 → Calibrated=2100+(-52)=2048 → MuJoCo shows 0 rad ✓
- Move +100 PWM: Raw=2200 → Calibrated=2148 → MuJoCo shows +0.157 rad (movement from home)

### What MuJoCo Displays

MuJoCo will show:
- **Zero position (qpos=0)** = Your chosen home position
- **Positive values** = Movement in one direction from home
- **Negative values** = Movement in opposite direction from home

The visual pose in MuJoCo represents **relative motion**, not absolute motor positions.

### Visual Example

```
BEFORE CALIBRATION:
Real Robot (home):  PWM [2100, 1950, 2048, ...]  
MuJoCo (at qpos=0): [0, 0, 0, ...]  
❌ Mismatch! Real robot at home ≠ MuJoCo at zero

AFTER CALIBRATION:
Real Robot (home):  PWM [2100, 1950, 2048, ...]
                      ↓ apply offset [-52, +98, 0, ...]
Calibrated:         PWM [2048, 2048, 2048, ...]
                      ↓ convert to radians
MuJoCo shows:       rad [0, 0, 0, ...]
✓ Match! Real robot home = MuJoCo zero

MOVEMENT TRACKING:
Real Robot moves:   PWM [2200, 2050, 2148, ...]  (moved +100, +100, +100)
                      ↓ apply same offsets
Calibrated:         PWM [2148, 2148, 2148, ...]
                      ↓ convert to radians  
MuJoCo shows:       rad [+0.157, +0.157, +0.157, ...]
✓ Movement tracked correctly!
```

## Calibration File Format

The calibration data is saved in `robot_calibration.json`:

```json
{
  "offsets": [10.5, -15.2, 5.0, 0.0, -8.3, 12.1],
  "reference_pose": [2037.5, 2063.2, 2043.0, 2048.0, 2056.3, 2035.9],
  "timestamp": "2025-11-10 14:30:00",
  "description": "PWM offsets to align real robot with MuJoCo simulator",
  "note": "reference_pose is the robot's physical home position..."
}
```

- **offsets**: PWM values to add to raw readings to center at 2048
- **reference_pose**: Raw PWM values at your chosen home position
- **timestamp**: When calibration was performed
- **note**: Explanation of what the calibration represents

## Recalibration

You should recalibrate if:
- You change your desired home position
- You reassemble the robot and motor positions shift
- Joint positions seem misaligned with simulation
- You want to redefine what "zero" means for your robot

Simply run the calibration process again - it will overwrite the previous calibration file.

**Note:** You don't need to worry about matching MuJoCo's visual appearance - just pick a safe, repeatable home position!

## Command Line Options

```bash
# Run calibration process
python calibration.py --calibrate [--port PORT]

# Test existing calibration
python calibration.py --test [--port PORT]

# Demo with MuJoCo
python calibration.py --demo [--port PORT] [--mujoco-xml PATH]

# Show help
python calibration.py --help
```

## Troubleshooting

**"Calibration file not found"**
- Run calibration first: `python calibration.py --calibrate`

**Robot position doesn't match simulation movements**
- Recalibrate and make sure robot is steady when recording home position
- Verify the correct port is being used
- Check that motors aren't slipping or have mechanical issues

**MuJoCo pose looks strange**
- This is expected! MuJoCo's visual zero is YOUR home position
- What matters is that MOVEMENTS match, not the absolute visual pose
- If you want MuJoCo to look different, you'd need to modify the XML (harder)

**Want to change MuJoCo's visual appearance?**
- Option 1 (Easy): Change your home position and recalibrate
- Option 2 (Hard): Modify `low_cost_robot/*.xml` joint offsets and URDF

**Values seem inverted or wrong direction**
- This suggests joint direction conventions differ
- May need to modify the `_pwm2pos` or `_pos2pwm` functions
- Check your MuJoCo model's joint axis definitions

## Integration with Existing Code

The calibration system is designed to be a drop-in addition to your existing workflow:

1. **Original workflow:**
   ```python
   raw_pwm = robot.read_position()
   d.qpos[:6] = sim_robot._pwm2pos(raw_pwm)
   ```

2. **With calibration:**
   ```python
   raw_pwm = robot.read_position()
   calibrated_pwm = calibration.apply_calibration(raw_pwm)
   d.qpos[:6] = sim_robot._pwm2pos(calibrated_pwm)
   ```

Just add one line to apply the calibration!
