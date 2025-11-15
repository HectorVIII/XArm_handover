#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ZED (Left Hand Stable Frame Detection) + xArm7
Workflow:
P0 → P1 (grip) → P0 → P2 (obtained from ZED left hand) → wait for pull → release → disable FT + recover → lift → back to P0
"""

import time
from xarm.wrapper import XArmAPI

from config import (
    ROBOT_IP,
    GRIPPER_SPEED,
    P0, P1,
    APPROACH_Z_UP, SAFE_Z_MAX,
    CLOSE_APPROACH_SPEED, CLOSE_APPROACH_ACC,
)
from arm_utils import recover, move, gripper_open, gripper_close
from zed_left_hand import detect_left_hand_stable_then_map_to_P2
from pull_release import detect_pull_then_release


def main():
    # Connect to robot
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    arm.connect()
    recover(arm)

    # Gripper init
    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_speed(GRIPPER_SPEED)

    print("Move to P0")
    move(arm, P0)

    print("Open GRipper")
    gripper_open(arm)

    print("MOve to P1 to grip tool")
    move(arm, P1)

    print("Close gripper")
    gripper_close(arm)

    print("Back to P0")
    move(arm, P0)

    # Left hand stable position -> P2
    P2 = detect_left_hand_stable_then_map_to_P2()
    P2_UP = dict(**P2)
    P2_UP["z"] = min(P2["z"] + APPROACH_Z_UP, SAFE_Z_MAX)

    print("Move to P2_UP")
    move(arm, P2_UP)

    print("MOve to P2")
    move(arm, P2, speed=CLOSE_APPROACH_SPEED, acc=CLOSE_APPROACH_ACC)

    # wait for pull → release
    if detect_pull_then_release(arm):
        # Disable FT after use (optional)
        try:
            arm.ft_sensor_enable(False)
        except Exception:
            pass

        time.sleep(0.2)
        recover(arm)

        # Lift + return to P0
        print("Lift")
        ret = move(arm, P2_UP, speed=160, acc=5000)
        if ret != 0:
            print(f"Lift failed (code={ret}), fallback with relative lift")
            code, cur = arm.get_position(is_radian=False)
            if code == 0 and cur:
                x, y, z, r, p, yaw = cur[:6]
                z2 = min(z + 60, SAFE_Z_MAX)
                arm.set_position(
                    x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                    speed=120, mvacc=4000, wait=True
                )
                recover(arm)
                move(arm, P2_UP, speed=160, acc=5000)

        print("Go back to P0")
        ret = move(arm, P0)
        if ret != 0:
            print(f"Return to P0 failed (code={ret}), fallback via lift")
            code, cur = arm.get_position(is_radian=False)
            if code == 0 and cur:
                x, y, z, r, p, yaw = cur[:6]
                z2 = min(z + 80, SAFE_Z_MAX)
                arm.set_position(
                    x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                    speed=120, mvacc=4000, wait=True
                )
                recover(arm)
                move(arm, P0)

    arm.disconnect()
    print("Process completed.")


if __name__ == "__main__":
    main()
