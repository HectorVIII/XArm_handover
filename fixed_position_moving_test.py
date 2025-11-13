#!/usr/bin/env python3
"""
Simple xArm Move Example
Move the xArm to a fixed position in Cartesian space
"""

from xarm.wrapper import XArmAPI
import time

ROBOT_IP = "192.168.1.225"

arm = XArmAPI(ROBOT_IP)

arm.motion_enable(enable=True)
arm.set_mode(0)   # 0 -> Position control mode
arm.set_state(0)  # 0 -> Ready

# wait for arm initilization
time.sleep(1)

# move to fixed position
arm.set_position(x=141.3, y=583.5, z=70, roll=177, pitch=-8.7, yaw=96.4, speed=100, wait=True)
print("position done")

#joint_angles_1 = [55.9, 47.8, 22.8, 82.7, -31, 31.5, 0]  
#arm.set_servo_angle(angle=joint_angles_1, speed=50, wait=True)
#print("joint done")

print("wait for handover")
time.sleep(3) # wait for handover


"""x2, y2, z2 = 629.3, -36.6, 110.1  #back to above of instuments
arm.set_position(x=629.3, y=-36.6, z=110.1, roll=178.2, pitch=-0.8, yaw=6.8, speed=100, wait=True)
print("second position done")"""

#joint_angles_2 = [-12.9,53.1,12.2,95.2,-16.8,42.8,0]
#arm.set_servo_angle(angle=joint_angles_2, speed=50, wait=True)
#print("second jiont done")

arm.disconnect()
print("arm disconnect")