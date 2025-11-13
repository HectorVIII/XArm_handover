#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ZED (Left Hand Stable Frame Detection) + xArm7
Workflow:
P0 → P1 (grip) → P0 → P2 (obtained from ZED left hand) → wait for pull → release → disable FT + recover → lift → back to P0
"""

import time, cv2, numpy as np
from statistics import mean
import pyzed.sl as sl
from xarm.wrapper import XArmAPI

# ================= xArm Basic Parameters =================
ROBOT_IP = "192.168.1.225"
MOVE_SPEED = 240
MOVE_ACC   = 6000
CLOSE_APPROACH_SPEED = 120  # slower speed for close approach
CLOSE_APPROACH_ACC   = 4000
GRIPPER_SPEED = 1500
OPEN_POS   = 850    # 85mm
CLOSE_POS  = 50

# Poses (mm/deg)
P0 = dict(x=629.3, y=-29.1, z=36.6, roll=178.2, pitch=-0.8, yaw=6.8)    # above tool
P1 = dict(x=632.0, y=-29.1, z=1.0,  roll=178.2, pitch=-0.8, yaw=6.8)   # grip tool
P2_ORI = dict(roll=177.0, pitch=-8.7, yaw=96.4) # P2 is the detected position by camera, orientation fixed
SAFE_Z_MIN, SAFE_Z_MAX = 0.0, 600.0
APPROACH_Z_UP = 50.0    # approach height before/after P2

# ================= Calibration Extrinsics (m) =================
R_cb = np.array( [[0.14401772498071977, -0.37152242415526016, 0.9171859044060677], [0.9895153863357063, 0.06424884057811589, -0.1293498615742083], [-0.010871756816483053, 0.9261982373305225, 0.3768801269230795]] )
t_cb = np.array( [1.4735924130368008, 0.4533308765337746, 0.38943522249067103] )
"""R_cb = np.array([
    [ 0.00671984, -0.36547256,  0.93079786],
    [ 0.99966612, -0.02076919, -0.01537194],
    [ 0.02494994,  0.93059037,  0.36521097]
])  # Rotation from camera to base

t_cb = np.array([1.44587977, 0.37286003, 0.32676220])   # Translation from camera to base"""

# ================= Left Hand Detection Parameters =================
CONF_THR = 0.60          # Confidence threshold
EMA_ALPHA = 0.7         # Smoothing factor
POS_TOL = 0.012          # Stability threshold(meter)
STABLE_FRAMES_REQUIRED = 120  # Consecutive stable frames

# ================= Pull-and-Release Detection Parameters  =================
FT_FORCE_RELEASE_N   = 7.0      # N: Force threshold to trigger release
CHECK_PERIOD         = 0.03    # seconds
DEBOUNCE_COUNT       = 4       # number of consecutive triggers to confirm
ALLOW_TRIGGER_AFTER  = 0.5   # seconds to wait before allowing trigger

# ------------------ xArm Control ------------------
# Recover from error/warn states
def recover(arm):
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.2)

# Move to pose   
def move(arm, pose, speed=MOVE_SPEED, acc=MOVE_ACC):
    return arm.set_position(
        x=pose["x"], y=pose["y"], z=pose["z"],
        roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"],
        speed=speed, mvacc=acc, wait=True)

# Gripper control
def gripper_open(arm):
    arm.set_gripper_speed(GRIPPER_SPEED)
    arm.set_gripper_position(OPEN_POS, wait=True)
def gripper_close(arm):
    arm.set_gripper_speed(GRIPPER_SPEED)
    arm.set_gripper_position(CLOSE_POS, wait=True)

# FT read with error handling
def read_ft_wrench(arm):
    """Safe FT read, returns (code, [Fx,Fy,Fz,Tx,Ty,Tz]) or (1,None)"""
    try:
        code, data = arm.get_ft_sensor_data()
        if code == 0 and data and len(data) >= 6:
            return 0, data
    except Exception:
        pass    # ignore
    return 1, None


def detect_pull_then_release(arm):
    print("[INFO] Waiting for pull trigger (FT mode)...")

    # Enable FT and zero if available
    try:
        arm.ft_sensor_enable(True)
        if hasattr(arm, "ft_sensor_set_zero"):
            arm.ft_sensor_set_zero()
    except Exception as e:
        print(f"[WARN] Failed to enable/zero FT: {e}")

    # Allow settle time after reaching target
    t0 = time.time()
    while time.time() - t0 < ALLOW_TRIGGER_AFTER:
        time.sleep(0.01)

    hits = 0
    while True:
        time.sleep(CHECK_PERIOD)
        code, wrench = read_ft_wrench(arm)
        if code != 0 or wrench is None:
            print("[WARN] FT read failed, retrying...")
            hits = 0
            continue

        Fx, Fy, Fz = wrench[:3]
        # NOTE: Units are assumed in Newtons; if your SDK returns N*mm for torque,
        # force should still be N. If values are strangely large, print and verify.
        F_total = (Fx*Fx + Fy*Fy + Fz*Fz) ** 0.5

        # Debug print (optional):
        # print(f"[FT] |F|={F_total:.2f} N (Fx={Fx:.2f}, Fy={Fy:.2f}, Fz={Fz:.2f}) hits={hits}")

        if F_total >= FT_FORCE_RELEASE_N:
            hits += 1
        else:
            hits = 0

        if hits >= DEBOUNCE_COUNT:
            print(f"[TRIGGERED] |F|={F_total:.2f} N ≥ {FT_FORCE_RELEASE_N:.1f} N → opening gripper.")
            arm.set_gripper_speed(GRIPPER_SPEED)
            arm.set_gripper_position(OPEN_POS, wait=True)
            return True

# ------------------ ZED Left Hand Detection ------------------
def detect_left_hand_stable_then_map_to_P2():
    """Detect left hand stable for ≥N frames, then map to base coordinates (mm)"""
    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.camera_resolution = sl.RESOLUTION.HD720
    ip.camera_fps = 60
    ip.depth_mode = sl.DEPTH_MODE.NEURAL
    ip.coordinate_units = sl.UNIT.METER
    ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    err = zed.open(ip)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(err)

    ptp = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(ptp)

    btp = sl.BodyTrackingParameters()
    btp.enable_tracking = True
    btp.enable_body_fitting = False            
    btp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    btp.body_format = sl.BODY_FORMAT.BODY_34   # Compatible: BODY_34 keeps left hand index=8
    zed.enable_body_tracking(btp)

    bodies = sl.Bodies()
    brt = sl.BodyTrackingRuntimeParameters()
    brt.detection_confidence_threshold = 40
    rtp = sl.RuntimeParameters()
    image = sl.Mat()

    LH_IDX = 8          # Left hand keypoint index in BODY_34
    ema = None
    last_ema = None
    stable_frames = 0

    print(f"Please extend your left hand and keep it stable for ~{STABLE_FRAMES_REQUIRED/60:.1f} seconds …")
    try:
        while True:
            if zed.grab(rtp) != sl.ERROR_CODE.SUCCESS:
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            zed.retrieve_bodies(bodies, brt)
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()

            if bodies.is_new:
                for body in bodies.body_list:
                    kc = body.keypoint_confidence
                    if len(kc) > LH_IDX and kc[LH_IDX] > CONF_THR:
                        lh = np.array(body.keypoint[LH_IDX], dtype=float)  # m
                        if np.any(np.isnan(lh)):
                            continue

                        # EMA smoothing
                        ema = lh if ema is None else EMA_ALPHA * lh + (1 - EMA_ALPHA) * ema

                        if last_ema is None:
                            last_ema = ema.copy()
                            stable_frames = 1
                        else:
                            diff = float(np.linalg.norm(ema - last_ema))
                            last_ema = ema.copy()
                            if diff <= POS_TOL:
                                stable_frames += 1
                            else:
                                stable_frames = 1  # Reset counter

                        # Overlay text
                        cv2.putText(frame,
                                    f"StableFrames: {stable_frames}/{STABLE_FRAMES_REQUIRED}",
                                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.imshow("ZED Left Hand Fast (BODY_34)", frame)
                        cv2.waitKey(1)

                        # Trigger when stable frames reach threshold
                        if stable_frames >= STABLE_FRAMES_REQUIRED:
                            print(f"Left hand stable ：{ema}")
                            # Camera(m) -> Base(m) -> (mm)
                            p_base_m = R_cb @ ema + t_cb
                            x_mm, y_mm, z_mm = 1000 * p_base_m[0], 1000 * p_base_m[1], 1000 * p_base_m[2]
                            z_mm = max(z_mm, SAFE_Z_MIN)
                            P2 = dict(x=x_mm, y=y_mm, z=z_mm, **P2_ORI)
                            print(f"→ converted to base frame P2={P2}")
                            return P2

            cv2.imshow("ZED Left Hand Fast (BODY_34)", frame)
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        try:
            zed.disable_body_tracking()
            zed.disable_positional_tracking()
        except Exception:
            pass
        zed.close()
        cv2.destroyAllWindows()

# ------------------ Main Workflow ------------------
def main():
    arm = XArmAPI(ROBOT_IP, is_radian=False)
    arm.connect()
    recover(arm)

    # Gripper init
    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_speed(GRIPPER_SPEED)

    print("Move to P0"); move(arm, P0)
    print("Open GRipper"); gripper_open(arm)
    print("MOve to P1 to grip tool"); move(arm, P1)
    print("Close gripper"); gripper_close(arm)
    print("Back to P0"); move(arm, P0)

    # Left hand stable position -> P2
    P2 = detect_left_hand_stable_then_map_to_P2()
    P2_UP = dict(**P2); P2_UP["z"] = min(P2["z"] + APPROACH_Z_UP, SAFE_Z_MAX)

    print("Move to P2_UP"); move(arm, P2_UP)
    print("MOve to P2"); move(arm, P2, speed=CLOSE_APPROACH_SPEED, acc=CLOSE_APPROACH_ACC)

    # wait for pull→ release
    if detect_pull_then_release(arm):
        # Disable FT after use (optional)
        try:
            arm.ft_sensor_enable(False)
        except Exception:
            pass
        time.sleep(0.2)
        recover(arm)

        # Lift + return to P0
        print("Lift"); ret = move(arm, P2_UP, speed=160, acc=5000)
        if ret != 0:
            print(f"RLift failed (code={ret}), fallback with relative lift")
            code, cur = arm.get_position(is_radian=False)
            if code==0 and cur:
                x,y,z,r,p,yaw = cur[:6]
                z2 = min(z+60, SAFE_Z_MAX)
                arm.set_position(x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                                 speed=120, mvacc=4000, wait=True)
                recover(arm); move(arm, P2_UP, speed=160, acc=5000)

        print("⬅️ 回 P0"); ret = move(arm, P0)
        if ret != 0:
            print(f"Return to P0 failed (code={ret}), fallback via lift")
            code, cur = arm.get_position(is_radian=False)
            if code==0 and cur:
                x,y,z,r,p,yaw = cur[:6]
                z2 = min(z+80, SAFE_Z_MAX)
                arm.set_position(x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                                 speed=120, mvacc=4000, wait=True)
                recover(arm); move(arm, P0)

    arm.disconnect(); print("Process completed.")

if __name__ == "__main__":
    main()
