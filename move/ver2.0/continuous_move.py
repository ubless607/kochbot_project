import sys
import select
import time
import numpy as np
import mujoco
import mujoco.viewer
import os

from interface import SimulatedRobot
from robot import Robot
# from dynamixel import Dynamixel

MODEL_PATH = 'low_cost_robot/scene.xml'

EE_SITE_NAME = 'joint6'        

GRIPPER_OPEN_PWM  = 2979     
GRIPPER_CLOSE_PWM = 1901 

sim_only_mode = False
home_mode = False
HOME_PWM = np.array([
    2048,  # joint1
    2048,  # joint2
    2048,  # joint3
    980,   # joint4
    2048,  # joint5
    GRIPPER_CLOSE_PWM  # joint6 (gripper)
], dtype=int)

# Joint 3: -90, Joint 5: 180
JOINT_OFFSETS = np.array([0, 0, -1.57, 0, 3.14, 0])

m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)
sim_robot = SimulatedRobot(m, d)

port = 'COM3'           
robot = Robot(device_name=port)

if os.name == 'nt':
    import msvcrt

robot._disable_torque()

pwm = np.array(robot.read_position())   
d.qpos[:6] = sim_robot._pwm2pos(pwm) + JOINT_OFFSETS     
mujoco.mj_forward(m, d)

ee_target_pos = sim_robot.read_ee_pos(EE_SITE_NAME).copy()

move_stride = 0.005   
ik_lr = 0.1          
dt = m.opt.timestep

control_joint_idx = [0, 1, 2, 3, 4]

teleop_active = True

curr_pwm = pwm.copy()
target_pwm = pwm.copy() 

def get_command_nonblocking():
    if os.name == 'nt':
        if msvcrt.kbhit(): 
            char = msvcrt.getwch()
            return char
        return None
    else:
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            return sys.stdin.readline().strip()
        return None

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        step_start = time.time()

        cmd = get_command_nonblocking()
        if cmd is not None:
            if isinstance(cmd, bytes):
                try:
                    key = cmd.decode('utf-8')
                except:
                    key = ''
            else:
                key = cmd[0] if len(cmd) > 0 else ''
            
            if key == 'x':
                robot._disable_torque()
                print("finish")
                break
            
            elif key == 'p' and not teleop_active:
                print("\n=== TELEOPERATION ===")
                pwm = np.array(robot.read_position())
                curr_pwm   = pwm.copy()
                target_pwm = pwm.copy()         
                # offset
                d.qpos[:6] = sim_robot._pwm2pos(pwm) + JOINT_OFFSETS
                mujoco.mj_forward(m, d)

                ee_target_pos = sim_robot.read_ee_pos(EE_SITE_NAME).copy()
                robot.set_goal_pos(curr_pwm)
                teleop_active = True
                print(f"현재 EE 위치: {ee_target_pos}")

            elif key == 'k' and not teleop_active:
                print("\n=== SIM ONLY ===")
                pwm = np.array(robot.read_position())
                curr_pwm   = pwm.copy()
                target_pwm = pwm.copy()
                # offset
                d.qpos[:6] = sim_robot._pwm2pos(pwm) + JOINT_OFFSETS
                mujoco.mj_forward(m, d)

                ee_target_pos = sim_robot.read_ee_pos(EE_SITE_NAME).copy()
                robot._disable_torque() # 실제 로봇 힘 빼기

                teleop_active = True
                home_mode = False
                sim_only_mode = True        
                print(f"[SIM ONLY] 시작 EE 위치: {ee_target_pos}")
            
            elif teleop_active and key == 'o':
                print("\n=== 홈 포즈 ===")
                target_pwm = HOME_PWM.copy()
                home_mode = True

            elif teleop_active:
                if key == 'w': ee_target_pos[1] += move_stride  # +Y
                elif key == 's': ee_target_pos[1] -= move_stride  # -Y
                elif key == 'a': ee_target_pos[0] -= move_stride  # -X
                elif key == 'd': ee_target_pos[0] += move_stride  # +X
                elif key == 'e': ee_target_pos[2] += move_stride  # +Z
                elif key == 'c' or key == 'q': ee_target_pos[2] -= move_stride  # -Z
                elif key == 'r':   # open
                    target_pwm[5] = GRIPPER_OPEN_PWM
                    print(f"[GRIPPER] target -> OPEN")
                elif key == 't':   # close
                    target_pwm[5] = GRIPPER_CLOSE_PWM
                    print(f"[GRIPPER] target -> CLOSE")

                print(f"[TELEOP] cmd={key}, new EE target = {ee_target_pos}")

        if not teleop_active:
            pwm = np.array(robot.read_position())
            curr_pwm   = pwm
            target_pwm = pwm.copy()
            d.qpos[:6] = sim_robot._pwm2pos(pwm) + JOINT_OFFSETS

        else:
            if not home_mode:
                # IK
                q_target = sim_robot.inverse_kinematics(
                    ee_target_pos,
                    lr=ik_lr,
                    joint_name=EE_SITE_NAME
                )                        

                real_q_target = q_target - JOINT_OFFSETS[:len(q_target)]
                
                pwm_target_arm = sim_robot._pos2pwm(real_q_target)
                pwm_target_arm = np.array([int(x) for x in pwm_target_arm])

                target_pwm[control_joint_idx] = pwm_target_arm

            # robot speed
            MAX_STEP = 10

            delta = target_pwm - curr_pwm
            step  = np.clip(delta, -MAX_STEP, MAX_STEP)
            curr_pwm = (curr_pwm + step).astype(int)
            
            if home_mode and np.all(np.abs(delta) <= MAX_STEP):
                home_mode = False

                mujoco.mj_forward(m, d)
                ee_target_pos = sim_robot.read_ee_pos(EE_SITE_NAME).copy()
                
            if not sim_only_mode:
                robot.set_goal_pos(curr_pwm)

            d.qpos[:6] = sim_robot._pwm2pos(curr_pwm) + JOINT_OFFSETS
            
        mujoco.mj_step(m, d)
        viewer.sync()

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)