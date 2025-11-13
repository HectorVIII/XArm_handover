#!/usr/bin/env python3
from xarm.wrapper import XArmAPI
import time


ROBOT_IP = "192.168.1.225"


arm = XArmAPI(ROBOT_IP)
arm.connect()
arm.clean_error()
arm.clean_warn()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.5)


arm.set_gripper_enable(True)
arm.set_gripper_mode(0)      
arm.set_gripper_speed(2000)  

OPEN_POS = 800
CLOSE_POS = 50
MID_POS = 400


print("🟢 Opening gripper...")
arm.set_gripper_position(OPEN_POS, wait=True)
time.sleep(1)

print("🔴 Closing gripper...")
#arm.set_gripper_position(CLOSE_POS, wait=True)
time.sleep(1)

arm.disconnect()
print("✅ Done.")