"""
Synchronized Real Robot and MuJoCo Simulation with Calibration

This is an enhanced version of sync_simul_real.py that uses the calibration system
to ensure proper alignment between the real robot and MuJoCo simulator.
"""

import time
import numpy as np
import mujoco
import mujoco.viewer

from interface import SimulatedRobot
from robot import Robot
from calibration import RobotCalibration


def main():
    # Load MuJoCo model
    print("Loading MuJoCo model...")
    m = mujoco.MjModel.from_xml_path('low_cost_robot/scene.xml')
    d = mujoco.MjData(m)
    sim_robot = SimulatedRobot(m, d)
    
    # Connect to robot
    port = '/dev/tty.usbmodem58760430141'  # Update this to your port
    print(f"Connecting to robot on port: {port}")
    robot = Robot(device_name=port)
    robot._disable_torque()
    
    # Load calibration
    print("Loading calibration...")
    calibration = RobotCalibration()
    try:
        calibration.load_calibration()
    except FileNotFoundError as e:
        print(f"\n{e}")
        print("\nPlease run calibration first:")
        print("  python calibration.py --calibrate")
        return
    
    # Set initial position with calibration
    print("Initializing robot position...")
    raw_pwm = np.array(robot.read_position())
    calibrated_pwm = calibration.apply_calibration(raw_pwm)
    d.qpos[:6] = sim_robot._pwm2pos(calibrated_pwm)
    print(f"Initial position - Raw: {raw_pwm}")
    print(f"Initial position - Calibrated: {calibrated_pwm}")
    
    # Launch viewer
    print("\n" + "="*60)
    print("Starting synchronized visualization...")
    print("Move the real robot to control the simulation.")
    print("Press ESC in the viewer window to exit.")
    print("="*60 + "\n")
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        frame_count = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # Read robot position and apply calibration
            raw_pwm = np.array(robot.read_position())
            calibrated_pwm = calibration.apply_calibration(raw_pwm)
            d.qpos[:6] = sim_robot._pwm2pos(calibrated_pwm)
            
            # Step simulation
            mujoco.mj_step(m, d)
            viewer.sync()
            
            # Display stats every 100 frames
            frame_count += 1
            if frame_count % 100 == 0:
                elapsed = time.time() - start
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f} | Frames: {frame_count} | Time: {elapsed:.1f}s")
            
            # Time keeping
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
