import sys
import select
import time
import numpy as np
import mujoco
import mujoco.viewer
import os

from interface import SimulatedRobot
from robot import Robot

MODEL_PATH = 'low_cost_robot/scene.xml'
EE_SITE_NAME = 'joint6'

GRIPPER_OPEN_PWM  = 2979     
GRIPPER_CLOSE_PWM = 1901 

# --- [설정 1] 오프셋 (시뮬레이션 싱크용) ---
# 실제 로봇은 펴져 있는데 시뮬은 굽혀져 있으므로, 시뮬 보여줄 때만 더하기
JOINT_OFFSETS = np.array([0, 0, -1.57, 0, 3.14, 0])

# --- [설정 2] 모터 방향 설정 (중요!) ---
# 순서: [Joint1, Joint2, Joint3, Joint4, Joint5, Gripper]
# 예상: a/d(1번), e/c(2번), w/s(3번)
# Joint 1: -1 (왼쪽/오른쪽 방향 맞춤)
# Joint 2: 1 (허리 펴기/접기)
# Joint 3: -1 (팔 뻗기/접기)
# Joint 4: 1 (손목 끄덕)
# Joint 5: 1 (손목 회전)
DIR = np.array([1, 1, 1, 1, 1, 1])

m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)
sim_robot = SimulatedRobot(m, d)

port = 'COM3'           
robot = Robot(device_name=port)

if os.name == 'nt':
    import msvcrt

robot._disable_torque()

# 초기화
pwm = np.array(robot.read_position())   
# 시뮬레이션 싱크 맞추기 (오프셋 적용)
d.qpos[:6] = sim_robot._pwm2pos(pwm) + JOINT_OFFSETS     
mujoco.mj_forward(m, d)

teleop_active = True
curr_pwm = pwm.copy()
target_pwm = pwm.copy() 

# --- [설정 3] 속도 조절 ---
# 키를 한 번 누를 때(루프당) 모터 값이 얼마나 변할지 (클수록 빠름)
JOINT_SPEED = 15  

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

print("=== 관절 제어 모드 시작 ===")
print("W/S: 팔(Joint3) 펴기/접기")
print("E/C: 허리(Joint2) 펴기/접기")
print("A/D: 몸통(Joint1) 좌/우 회전")
print("U/J: 손목(Joint4) 펴기/접기")
print("I/K: 손목(Joint5) 회전")
print("R/T: 그리퍼")
print("X: 종료")

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        step_start = time.time()

        cmd = get_command_nonblocking()
        if cmd is not None:
            if isinstance(cmd, bytes):
                try: key = cmd.decode('utf-8')
                except: key = ''
            else:
                key = cmd[0] if len(cmd) > 0 else ''
            
            if key == 'x':
                robot._disable_torque()
                print("finish")
                break
            
            # --- [핵심] 관절 직접 제어 (Excavator Style) ---
            # 각 키가 특정 모터(Index)를 직접 건드립니다.
            
            # 1. 몸통 좌우 (Joint 1) - index 0
            if key == 'a':   target_pwm[0] += JOINT_SPEED * DIR[0] # 좌
            elif key == 'd': target_pwm[0] -= JOINT_SPEED * DIR[0] # 우
            
            # 2. 허리 상하 (Joint 2) - index 1
            elif key == 'e': target_pwm[1] += JOINT_SPEED * DIR[1] # 펴기(Up)
            elif key == 'c': target_pwm[1] -= JOINT_SPEED * DIR[1] # 접기(Down)
            
            # 3. 팔 상하 (Joint 3) - index 2
            elif key == 'w': target_pwm[2] += JOINT_SPEED * DIR[2] # 뻗기(Extend)
            elif key == 's': target_pwm[2] -= JOINT_SPEED * DIR[2] # 접기(Fold)
            
            # (옵션) 4. 손목 끄덕 (Joint 4) - index 3
            elif key == 'u': target_pwm[3] += JOINT_SPEED * DIR[3]
            elif key == 'j': target_pwm[3] -= JOINT_SPEED * DIR[3]
            
            # (옵션) 5. 손목 회전 (Joint 5) - index 4
            elif key == 'i': target_pwm[4] += JOINT_SPEED * DIR[4]
            elif key == 'k': target_pwm[4] -= JOINT_SPEED * DIR[4]

            # 6. 그리퍼 (Joint 6) - index 5
            elif key == 'r': # Open
                target_pwm[5] = GRIPPER_OPEN_PWM
                print(f"[GRIPPER] OPEN")
            elif key == 't': # Close
                target_pwm[5] = GRIPPER_CLOSE_PWM
                print(f"[GRIPPER] CLOSE")

        if teleop_active:
            # 1. 안전장치: PWM 값이 0~4095를 벗어나지 않게 자름
            target_pwm = np.clip(target_pwm, 0, 4095)
            
            # 2. 부드러운 움직임 (현재 값 -> 목표 값 서서히 이동)
            # 한 번에 변하는 양을 제한 (스무딩)
            MAX_CHANGE = 10 
            diff = target_pwm - curr_pwm
            step = np.clip(diff, -MAX_CHANGE, MAX_CHANGE)
            curr_pwm = (curr_pwm + step).astype(int)
            
            # 3. 로봇에 명령 전송
            robot.set_goal_pos(curr_pwm)

            # 4. 시뮬레이션 업데이트 (오프셋 더해서 보여줌)
            d.qpos[:6] = sim_robot._pwm2pos(curr_pwm) + JOINT_OFFSETS
            mujoco.mj_step(m, d)
            viewer.sync()

            # 디버깅: 현재 모터 값 출력 (30프레임마다 한 번씩만)
            # if int(time.time() * 10) % 5 == 0:
            #     print(f"PWM: {curr_pwm}")

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)