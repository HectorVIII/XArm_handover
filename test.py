#!/usr/bin/env python3
from xarm.wrapper import XArmAPI
import time

# === 连接机械臂 ===
ROBOT_IP = "192.168.1.225"   # 你的xArm IP
arm = XArmAPI(ROBOT_IP)

# 启用电机 & 设置模式
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

# === 打印当前位姿 ===
print("\n--- 当前位姿 ---")
pos_now = arm.get_position(is_radian=False)
angles_now = arm.get_servo_angle(is_radian=False)
print(f"笛卡尔位置 (x, y, z, roll, pitch, yaw): {pos_now[1]}")
print(f"关节角度 (°): {angles_now[1]}")

# === 移动到 Home ===
print("\n回到 Home 位置中...")
arm.move_gohome(wait=True)
time.sleep(1)

# === 打印 Home 位姿 ===
print("\n--- Home 位姿 ---")
pos_home = arm.get_position(is_radian=False)
angles_home = arm.get_servo_angle(is_radian=False)
print(f"笛卡尔位置 (x, y, z, roll, pitch, yaw): {pos_home[1]}")
print(f"关节角度 (°): {angles_home[1]}")

# === 对比结果 ===
print("\n--- 位姿变化对比 ---")
dx = [round(pos_home[1][i] - pos_now[1][i], 2) for i in range(6)]
print(f"位置变化 Δ(x,y,z,roll,pitch,yaw): {dx}")
