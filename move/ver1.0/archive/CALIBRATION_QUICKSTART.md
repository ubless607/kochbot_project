# Robot Calibration System - Quick Reference

## 🎯 Problem Solved

Motors can be assembled at different 360° positions, making it dangerous or impossible to physically match MuJoCo's visual zero pose. This calibration system maps your robot's safe "home" position to MuJoCo's zero, enabling accurate motion tracking without requiring pose matching.

## 🚀 Quick Start

### 1️⃣ First Time Setup (5 minutes)

```bash
# Run calibration
python calibration.py --calibrate --port /dev/tty.usbmodem58760430141

# Follow prompts:
# - Robot torque will be disabled
# - Move robot to a safe, comfortable "home" position
# - System records this and calculates offsets
# - Calibration saved to robot_calibration.json
```

### 2️⃣ Daily Use

```bash
# Option A: Run the enhanced sync script
python sync_simul_real_calibrated.py

# Option B: Use in your own code
# (see code examples below)
```

### 3️⃣ Check Status Anytime

```bash
# Quick status check
python check_calibration.py

# Interactive test
python calibration.py --test
```

## 📁 Files Overview

| File | Purpose |
|------|---------|
| `calibration.py` | Main calibration system (run/test/demo) |
| `sync_simul_real_calibrated.py` | Enhanced sync script with calibration |
| `check_calibration.py` | Quick validation tool |
| `calibration_example.py` | Concept demonstration |
| `calibration_usage_examples.py` | Practical usage examples |
| `robot_calibration.json` | Your calibration data (created after setup) |
| `CALIBRATION_README.md` | Full documentation |
| `CALIBRATION_PHILOSOPHY.md` | Design rationale |

## 💻 Code Examples

### Basic Usage

```python
from robot import Robot
from calibration import RobotCalibration

# Setup
robot = Robot(device_name='/dev/tty.usbmodem58760430141')
robot._disable_torque()

calibration = RobotCalibration()
calibration.load_calibration()

# Read position with calibration
raw_pwm = robot.read_position()
calibrated_pwm = calibration.apply_calibration(raw_pwm)
```

### With MuJoCo Sync

```python
import mujoco
from interface import SimulatedRobot

m = mujoco.MjModel.from_xml_path('low_cost_robot/scene.xml')
d = mujoco.MjData(m)
sim = SimulatedRobot(m, d)

# In your loop
raw_pwm = np.array(robot.read_position())
calibrated_pwm = calibration.apply_calibration(raw_pwm)
d.qpos[:6] = sim._pwm2pos(calibrated_pwm)
```

### Reverse Calibration (Send Commands)

```python
# Want to move to specific MuJoCo angles
desired_angles_rad = np.array([0.0, 0.5, -0.3, 0.2, 0.0, 0.1])

# Convert to raw PWM for robot
calibrated_pwm = (desired_angles_rad / 3.14 + 1) * 2048
raw_pwm = calibrated_pwm - calibration.offsets

# Send to robot
robot.set_goal_pos(raw_pwm.astype(int))
```

## 🔧 Common Commands

```bash
# Calibration
python calibration.py --calibrate              # Run calibration
python calibration.py --test                   # Test calibration
python calibration.py --demo                   # Demo with MuJoCo
python check_calibration.py                    # Status check

# Examples
python calibration_example.py                  # See concept demo
python calibration_usage_examples.py           # Usage examples
python calibration_usage_examples.py --example teleoperation

# Production
python sync_simul_real_calibrated.py          # Run sync with calibration
```

## ❓ FAQ

**Q: Do I need to make my robot look like MuJoCo's zero pose?**  
A: No! That's the whole point. Choose any safe home position.

**Q: Will MuJoCo look wrong at zero?**  
A: MuJoCo's zero will show YOUR home position. Movement tracking is perfect.

**Q: When should I recalibrate?**  
A: After reassembly, if behavior seems off, or to change your home position.

**Q: Can I change MuJoCo's visual appearance?**  
A: Yes, either recalibrate with a different home, or modify the XML (harder).

**Q: What if I have multiple robots?**  
A: Each needs its own calibration. Use `RobotCalibration("robot1_cal.json")`.

## 🎓 Learn More

- `CALIBRATION_README.md` - Detailed documentation
- `CALIBRATION_PHILOSOPHY.md` - Why this approach works
- `calibration_example.py` - Run to see the math
- `calibration_usage_examples.py` - Practical integration patterns

## 🆘 Troubleshooting

```bash
# Calibration file not found?
python calibration.py --calibrate

# Values seem weird?
python check_calibration.py
python calibration.py --test

# Movement direction wrong?
# Check _pwm2pos conversion in interface.py

# Want to start over?
rm robot_calibration.json
python calibration.py --calibrate
```

## ✅ What You Get

- ✅ Safe calibration (no risk of robot damage)
- ✅ Perfect motion tracking
- ✅ Easy to use and recalibrate
- ✅ No XML modification needed
- ✅ Works with any home position

## 📊 How It Works (One Line)

```
Your Home Position → MuJoCo Zero | All movement tracked relative to this
```

---

**Ready to start?** Run `python calibration.py --calibrate` and follow the prompts!
