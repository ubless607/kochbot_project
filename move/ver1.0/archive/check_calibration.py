#!/usr/bin/env python3
"""
Quick calibration status and validation script
Run this to check if calibration exists and looks reasonable
"""

import sys
from pathlib import Path
import json
import numpy as np
from calibration import RobotCalibration, CALIBRATION_FILE


def main():
    print("\n" + "="*70)
    print("CALIBRATION STATUS CHECK")
    print("="*70)
    
    # Check if file exists
    if not Path(CALIBRATION_FILE).exists():
        print(f"\n❌ Calibration file not found: {CALIBRATION_FILE}")
        print("\nTo create calibration:")
        print(f"  python calibration.py --calibrate --port YOUR_PORT")
        print("\nExample:")
        print(f"  python calibration.py --calibrate --port /dev/tty.usbmodem58760430141")
        sys.exit(1)
    
    # Load and display
    print(f"\n✓ Calibration file found: {CALIBRATION_FILE}")
    
    with open(CALIBRATION_FILE, 'r') as f:
        data = json.load(f)
    
    offsets = np.array(data['offsets'])
    reference = np.array(data['reference_pose'])
    timestamp = data.get('timestamp', 'unknown')
    
    print(f"\n📅 Created: {timestamp}")
    print(f"\n📊 Calibration Data:")
    print(f"   Reference pose (home): {reference.astype(int)}")
    print(f"   Offsets:              {offsets.astype(int)}")
    
    # Validate
    print(f"\n🔍 Validation:")
    warnings = []
    
    # Check offset magnitudes
    if np.any(np.abs(offsets) > 1000):
        warnings.append(f"⚠️  Very large offsets (>{1000}): {offsets[np.abs(offsets) > 1000]}")
    else:
        print(f"   ✓ Offsets in reasonable range (max: {np.abs(offsets).max():.1f})")
    
    # Check reference pose is reasonable
    if np.any(reference < 0) or np.any(reference > 4096):
        warnings.append(f"⚠️  Reference pose outside [0, 4096]: {reference}")
    else:
        print(f"   ✓ Reference pose in valid range [0, 4096]")
    
    # Check if reference is very far from center
    distance_from_center = np.abs(reference - 2048)
    if np.any(distance_from_center > 1500):
        warnings.append(f"⚠️  Some joints very far from center (2048): {reference[distance_from_center > 1500]}")
    else:
        print(f"   ✓ Reference pose reasonably centered (max offset: {distance_from_center.max():.1f})")
    
    # Display warnings
    if warnings:
        print(f"\n⚠️  WARNINGS:")
        for warning in warnings:
            print(f"   {warning}")
        print(f"\n   These might be OK depending on your robot, but consider recalibrating")
        print(f"   if behavior seems incorrect.")
    else:
        print(f"\n✅ Calibration looks healthy!")
    
    # Show what happens at home
    print(f"\n🏠 At Home Position:")
    print(f"   Raw PWM:        {reference.astype(int)}")
    calibrated_home = reference + offsets
    print(f"   Calibrated PWM: {calibrated_home.astype(int)}")
    radians_home = (calibrated_home / 2048 - 1) * 3.14
    print(f"   MuJoCo radians: {np.round(radians_home, 3)}")
    if np.allclose(radians_home, 0, atol=0.01):
        print(f"   ✓ Correctly maps to MuJoCo zero!")
    else:
        print(f"   ⚠️  Doesn't map to zero - calibration may be corrupted")
    
    # Usage suggestions
    print(f"\n📖 Usage:")
    print(f"   Test:  python calibration.py --test")
    print(f"   Demo:  python calibration.py --demo")
    print(f"   Sync:  python sync_simul_real_calibrated.py")
    print(f"   Redo:  python calibration.py --calibrate")
    
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
