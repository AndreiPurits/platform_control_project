#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import glob
import cv2
import numpy as np

# --------- ПАРАМЕТРЫ (под твою шахматку) ----------
INNER_CORNERS = (9, 6)         # (X, Y) внутренние углы
SQUARE_SIZE_M = 0.0266  

IMAGES_GLOB = "calib_imgs/*.*"  # jpg/png
OUT_NPZ = "camera_calib.npz"

# --------------------------------------------------

def main():
    imgs = sorted(glob.glob(IMAGES_GLOB))
    if not imgs:
        print(f"[ERR] No images found: {IMAGES_GLOB}")
        return

    objp = np.zeros((INNER_CORNERS[0]*INNER_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:INNER_CORNERS[0], 0:INNER_CORNERS[1]].T.reshape(-1, 2)
    objp *= float(SQUARE_SIZE_M)

    objpoints = []
    imgpoints = []
    img_size = None

    ok_cnt = 0
    for fn in imgs:
        img = cv2.imread(fn)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        ok, corners = cv2.findChessboardCorners(gray, INNER_CORNERS)
        if not ok:
            continue

        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        )

        objpoints.append(objp)
        imgpoints.append(corners)
        ok_cnt += 1

    if ok_cnt < 10:
        print(f"[ERR] Too few valid frames: {ok_cnt}. Need ~15-30.")
        return
    else:
        print(f"qty {ok_cnt}. Need ~15-30.")
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    print("[OK] calibrateCamera ret:", ret)
    print("K=\n", K)
    print("D=", D.ravel())

    np.savez(OUT_NPZ, K=K, D=D, img_size=np.array(img_size, dtype=np.int32))
    print(f"[OK] Saved: {OUT_NPZ}")

    # quick sanity: mean reprojection error
    total_err = 0.0
    total_pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err * err
        total_pts += len(objpoints[i])
    rmse = (total_err / max(1, total_pts)) ** 0.5
    print(f"[CHECK] reprojection RMSE px: {rmse:.3f} (обычно < 0.7 хорошо)")

if __name__ == "__main__":
    main()