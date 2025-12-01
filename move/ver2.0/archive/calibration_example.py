"""
Simple example demonstrating the calibration concept

This shows how calibration maps your robot's physical home position
to MuJoCo's zero position without requiring them to look the same.
"""

import numpy as np


def demonstrate_calibration():
    """
    Visual demonstration of how calibration works
    """
    print("="*70)
    print("CALIBRATION CONCEPT DEMONSTRATION")
    print("="*70)
    
    # Simulate a robot where motors are assembled at different positions
    print("\n1. PROBLEM: Motors assembled at different absolute positions")
    print("-" * 70)
    
    home_position_pwm = np.array([2100, 1950, 2048, 2200, 1900, 2050])
    print(f"Robot's comfortable home position (PWM): {home_position_pwm}")
    print(f"MuJoCo expects zero at (PWM):            [2048, 2048, 2048, ...]")
    print(f"❌ These don't match! But we CAN'T force the robot to 2048 everywhere")
    print(f"   (might break the robot or be physically impossible)")
    
    # Solution: Calculate offsets
    print("\n2. SOLUTION: Calculate calibration offsets")
    print("-" * 70)
    
    mujoco_zero = 2048.0
    offsets = mujoco_zero - home_position_pwm
    print(f"Offsets = {mujoco_zero} - home_position")
    print(f"Offsets = {offsets}")
    
    # Apply calibration at home
    print("\n3. AT HOME POSITION: Robot home → MuJoCo zero")
    print("-" * 70)
    
    calibrated_home = home_position_pwm + offsets
    mujoco_rad_home = (calibrated_home / 2048 - 1) * 3.14
    
    print(f"Raw PWM (at home):        {home_position_pwm}")
    print(f"Apply offsets:            + {offsets}")
    print(f"Calibrated PWM:           = {calibrated_home}")
    print(f"MuJoCo radians:           = {mujoco_rad_home}")
    print(f"✓ Success! Home position shows as zero in MuJoCo")
    
    # Apply calibration when moved
    print("\n4. AFTER MOVEMENT: Tracking relative motion")
    print("-" * 70)
    
    moved_position = home_position_pwm + np.array([100, -50, 150, 0, 75, -100])
    calibrated_moved = moved_position + offsets
    mujoco_rad_moved = (calibrated_moved / 2048 - 1) * 3.14
    
    print(f"Raw PWM (after moving):   {moved_position}")
    print(f"Apply same offsets:       + {offsets}")
    print(f"Calibrated PWM:           = {calibrated_moved}")
    print(f"MuJoCo radians:           = {np.round(mujoco_rad_moved, 3)}")
    print(f"✓ Movement correctly tracked relative to home!")
    
    # Summary
    print("\n5. KEY INSIGHT")
    print("="*70)
    print("• Your robot's HOME position → MuJoCo's ZERO (qpos=0)")
    print("• MuJoCo displays RELATIVE movement from your home")
    print("• No need to match absolute visual poses")
    print("• Safe for the robot, accurate motion tracking")
    print("="*70)
    
    # What this means in practice
    print("\n6. IN PRACTICE")
    print("-" * 70)
    print("When you run the sync:")
    print("  1. Move robot to home → MuJoCo shows neutral pose")
    print("  2. Move joint +30° → MuJoCo joint moves +30°")
    print("  3. Return to home → MuJoCo returns to neutral")
    print("\nThe MuJoCo visualization is 'centered' on YOUR home position!")
    print("="*70)


if __name__ == "__main__":
    demonstrate_calibration()
