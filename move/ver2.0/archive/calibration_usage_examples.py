"""
Example: Using calibration in a robot control loop

This shows how to integrate calibration into various control scenarios:
1. Teleoperation (reading positions)
2. Position control (sending commands)
3. Trajectory following
"""

import numpy as np
import time
from robot import Robot
from calibration import RobotCalibration


def example_teleoperation(robot: Robot, calibration: RobotCalibration, duration=10.0):
    """
    Example: Read robot positions with calibration for teleoperation
    """
    print("\n" + "="*60)
    print("EXAMPLE 1: TELEOPERATION")
    print("="*60)
    print("Reading robot positions with calibration applied")
    print(f"Running for {duration} seconds...\n")
    
    robot._disable_torque()
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Read raw position
        raw_pwm = np.array(robot.read_position())
        
        # Apply calibration
        calibrated_pwm = calibration.apply_calibration(raw_pwm)
        
        # Convert to radians (MuJoCo format)
        joint_angles_rad = (calibrated_pwm / 2048 - 1) * 3.14
        
        # These joint angles can now be sent to MuJoCo or used for control
        print(f"\rJoint angles (rad): {np.round(joint_angles_rad, 3)}", end="")
        time.sleep(0.1)
    
    print("\n✓ Teleoperation example complete\n")


def example_position_control_with_calibration(robot: Robot, calibration: RobotCalibration):
    """
    Example: Sending position commands with reverse calibration
    
    If you want to command the robot to a specific MuJoCo angle,
    you need to reverse the calibration.
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: POSITION CONTROL")
    print("="*60)
    print("Sending position commands to robot using MuJoCo coordinates\n")
    
    # Desired joint angles in MuJoCo coordinates (radians)
    desired_angles_rad = np.array([0.0, 0.5, -0.3, 0.2, 0.0, 0.1])
    
    print(f"Target angles (rad): {desired_angles_rad}")
    
    # Convert to PWM (MuJoCo format)
    desired_pwm_calibrated = (desired_angles_rad / 3.14 + 1) * 2048
    
    # Reverse calibration to get raw PWM for robot
    raw_pwm_target = desired_pwm_calibrated - calibration.offsets
    
    print(f"Calibrated PWM: {desired_pwm_calibrated.astype(int)}")
    print(f"Raw PWM to send: {raw_pwm_target.astype(int)}")
    
    # Send to robot (uncomment to actually move)
    # robot._set_position_control()
    # robot.set_goal_pos(raw_pwm_target.astype(int))
    
    print("✓ (Command prepared - uncomment to execute)\n")


def example_record_and_replay(robot: Robot, calibration: RobotCalibration):
    """
    Example: Record trajectory and replay it
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: RECORD AND REPLAY TRAJECTORY")
    print("="*60)
    
    # Record phase
    print("\nRECORD PHASE:")
    print("Move the robot around (torque off)")
    robot._disable_torque()
    
    trajectory = []
    print("Recording for 5 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        raw_pwm = np.array(robot.read_position())
        calibrated_pwm = calibration.apply_calibration(raw_pwm)
        joint_angles = (calibrated_pwm / 2048 - 1) * 3.14
        
        trajectory.append({
            'time': time.time() - start_time,
            'angles': joint_angles,
            'raw_pwm': raw_pwm
        })
        time.sleep(0.05)
    
    print(f"✓ Recorded {len(trajectory)} waypoints")
    
    # Replay phase (simulation - not actually moving robot)
    print("\nREPLAY PHASE (simulated):")
    print("Would replay trajectory by converting back to raw PWM:")
    
    for i, waypoint in enumerate(trajectory[::20]):  # Show every 20th point
        angles = waypoint['angles']
        
        # Convert back to raw PWM for robot
        calibrated_pwm = (angles / 3.14 + 1) * 2048
        raw_pwm_replay = calibrated_pwm - calibration.offsets
        
        print(f"  Waypoint {i}: angles={np.round(angles, 2)} "
              f"→ raw_pwm={raw_pwm_replay.astype(int)}")
    
    print("\n✓ Trajectory processing complete\n")


def example_safety_check(robot: Robot, calibration: RobotCalibration):
    """
    Example: Monitor robot and warn if positions are unusual
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: SAFETY MONITORING")
    print("="*60)
    print("Checking if current position looks safe\n")
    
    robot._disable_torque()
    
    raw_pwm = np.array(robot.read_position())
    health = calibration.check_calibration_health(raw_pwm)
    
    print(f"Status: {health['status'].upper()}")
    print(f"Message: {health['message']}")
    print(f"Raw PWM: {health['current_pwm'].astype(int)}")
    print(f"Calibrated PWM: {health['calibrated_pwm'].astype(int)}")
    print(f"Joint angles (rad): {np.round(health['radians'], 3)}")
    
    if 'warnings' in health:
        print("\n⚠️  Warnings:")
        for warning in health['warnings']:
            print(f"  - {warning}")
    else:
        print("\n✓ All checks passed!")
    
    print()


def main():
    """Run all examples"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Calibration usage examples")
    parser.add_argument('--port', type=str, default='/dev/tty.usbmodem58760430141',
                       help='Robot serial port')
    parser.add_argument('--example', type=str, 
                       choices=['teleoperation', 'position', 'trajectory', 'safety', 'all'],
                       default='all',
                       help='Which example to run')
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("CALIBRATION USAGE EXAMPLES")
    print("="*60)
    
    # Setup
    print(f"\nConnecting to robot on {args.port}...")
    robot = Robot(device_name=args.port)
    
    print("Loading calibration...")
    calibration = RobotCalibration()
    try:
        calibration.load_calibration()
    except FileNotFoundError as e:
        print(f"\n❌ {e}")
        print("\nRun calibration first:")
        print("  python calibration.py --calibrate")
        return
    
    # Run examples
    if args.example in ['teleoperation', 'all']:
        example_teleoperation(robot, calibration, duration=3.0)
    
    if args.example in ['position', 'all']:
        example_position_control_with_calibration(robot, calibration)
    
    if args.example in ['trajectory', 'all']:
        example_record_and_replay(robot, calibration)
    
    if args.example in ['safety', 'all']:
        example_safety_check(robot, calibration)
    
    print("="*60)
    print("ALL EXAMPLES COMPLETE")
    print("="*60)


if __name__ == "__main__":
    main()
