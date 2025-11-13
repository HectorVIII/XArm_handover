#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ZED 左手“稳定2秒” => 自动与 xArm 同步采样，保存CSV，并可一键标定

import pyzed.sl as sl
import cv2, time, csv, os
import numpy as np
from xarm.wrapper import XArmAPI

# ====== 参数 ======
ROBOT_IP = "192.168.1.225"
CSV_PATH = "hand_arm_pairs.csv"
CONF_THR = 0.60
EMA_ALPHA = 0.40
POS_TOL   = 0.010   # 1cm
STABLE_T  = 2.00    # 2s

# ====== 工具函数 ======
def rigid_transform_3D(A, B):
    assert A.shape == B.shape and A.shape[1] == 3 and A.shape[0] >= 3
    CA, CB = A.mean(axis=0), B.mean(axis=0)
    AA, BB = A-CA, B-CB
    H = AA.T @ BB
    U,S,Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1,:] *= -1
        R = Vt.T @ U.T
    t = CB - R @ CA
    return R, t

def rmse_Rt(R, t, A, B):
    pred = (A @ R.T) + t
    e = np.linalg.norm(pred - B, axis=1)
    return float(np.sqrt((e**2).mean())), e

def pick_best_left_hand(body_list, conf_thr=CONF_THR):
    best, best_conf = None, -1.0
    for body in body_list:
        kp = body.keypoint
        kc = body.keypoint_confidence
        if len(kc) > 8 and kc[8] > conf_thr:
            p = np.array(kp[8], dtype=float)
            if not np.any(np.isnan(p)):
                if kc[8] > best_conf:
                    best, best_conf = p, kc[8]
    return best, best_conf

# ====== 连接机械臂 ======
arm = XArmAPI(ROBOT_IP, is_radian=False)
arm.connect()

# ====== 打开ZED ======
zed = sl.Camera()
ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.HD720
ip.camera_fps = 60
ip.depth_mode = sl.DEPTH_MODE.NEURAL
ip.coordinate_units = sl.UNIT.METER
ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
ec = zed.open(ip)
if ec != sl.ERROR_CODE.SUCCESS:
    raise RuntimeError(ec)
ptp = sl.PositionalTrackingParameters()
zed.enable_positional_tracking(ptp)
btp = sl.BodyTrackingParameters()
btp.enable_tracking = True
btp.enable_body_fitting = True
btp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
btp.body_format = sl.BODY_FORMAT.BODY_34
zed.enable_body_tracking(btp)

bodies = sl.Bodies()
rtp = sl.RuntimeParameters()
brt = sl.BodyTrackingRuntimeParameters()
brt.detection_confidence_threshold = 40
img = sl.Mat()

# ====== 稳定检测状态 ======
ema = None
anchor = None
t_anchor = None
printed = False

# ====== CSV 准备 ======
newfile = not os.path.exists(CSV_PATH)
f = open(CSV_PATH, "a", newline="")
w = csv.writer(f)
if newfile:
    w.writerow(["cam_x","cam_y","cam_z","base_x","base_y","base_z"])  # 米

print("[提示] 手稳定2秒会自动采样并保存到 CSV。按 'c' 计算外参；'q' 退出。")

try:
    while True:
        if zed.grab(rtp) != sl.ERROR_CODE.SUCCESS:
            if cv2.waitKey(1) == ord('q'): break
            continue

        zed.retrieve_bodies(bodies, brt)
        zed.retrieve_image(img, sl.VIEW.LEFT)
        frame = img.get_data()
        now = time.time()

        lh, conf = (None, None)
        p2 = None
        if bodies.is_new:
            lh, conf = pick_best_left_hand(bodies.body_list, CONF_THR)
            if bodies.body_list:
                p2_2d = bodies.body_list[0].keypoint_2d[8]
                if not np.isnan(p2_2d[0]): p2 = (int(p2_2d[0]), int(p2_2d[1]))

        if lh is not None:
            # EMA
            ema = lh if ema is None else EMA_ALPHA*lh + (1-EMA_ALPHA)*ema

            if anchor is None:
                anchor = ema.copy()
                t_anchor = now
                printed = False
            else:
                if np.linalg.norm(ema - anchor) <= POS_TOL:
                    pass
                else:
                    anchor = ema.copy()
                    t_anchor = now
                    printed = False

            stable_elapsed = now - t_anchor if t_anchor else 0.0

            # 叠字
            cv2.putText(frame, f"conf={conf:.2f} stable={stable_elapsed:.1f}s tol={POS_TOL*100:.1f}cm",
                        (30,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50,220,50), 2)
            if p2:
                color = (0,200,0) if stable_elapsed >= STABLE_T else (0,165,255)
                cv2.putText(frame, "STABLE" if stable_elapsed>=STABLE_T else "HOLD",
                            (p2[0]+12,p2[1]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(frame, f"[{ema[0]:.2f},{ema[1]:.2f},{ema[2]:.2f}] m",
                            (p2[0]+12,p2[1]+12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)

            # 满足稳定 => 自动采样一次（相机+机械臂 同步）
            if (stable_elapsed >= STABLE_T) and (not printed):
                code, pose = arm.get_position(is_radian=False)  # mm
                if code == 0 and pose:
                    base_m = (np.array(pose[:3], dtype=float) / 1000.0).tolist()
                    cam_m  = [float(ema[0]), float(ema[1]), float(ema[2])]
                    w.writerow(cam_m + base_m); f.flush()
                    print(f"[Saved] cam={cam_m}  base={base_m}")
                    printed = True  # 本段稳定只存一次
        else:
            ema = None; anchor = None; t_anchor = None; printed = False
            cv2.putText(frame, "No LEFT_HAND detected",
                        (30,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        cv2.imshow("ZED Hand Calib (Stable>=2s -> Autosave)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # 读取CSV做一键标定
            f.flush()
            data = np.loadtxt(CSV_PATH, delimiter=",", skiprows=1)
            if data.ndim == 1 or data.shape[0] < 3:
                print("样本不足（至少3条）"); continue
            A = data[:, :3]   # cam (m)
            B = data[:, 3:6]  # base (m)
            R, t = rigid_transform_3D(A, B)
            rmse, per = rmse_Rt(R, t, A, B)
            print("\n=== Calib Result ===")
            print("R_cb =\n", R)
            print("t_cb (m) =", t)
            print(f"RMSE = {rmse*1000:.2f} mm")
            print("---- 粘贴到主脚本 ----")
            print("R_cb = np.array(", repr(R.tolist()), ")")
            print("t_cb = np.array(", repr(t.tolist()), ")")
            print("====================\n")

finally:
    try: f.close()
    except: pass
    zed.disable_body_tracking(); zed.disable_positional_tracking(); zed.close()
    cv2.destroyAllWindows()
    arm.disconnect()

