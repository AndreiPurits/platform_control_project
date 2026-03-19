# -*- coding: utf-8 -*-
from __future__ import annotations

import faulthandler
faulthandler.enable()

import numpy as np
import cv2
from multiprocessing.connection import Connection

def _make_aruco_detector():
    """
    Returns (aruco, aruco_dict, detector_or_None, params_or_None)
    Tuned for STABILITY (no blinking at 1–3 m).
    """
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available")

    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()

    # ---------- CRITICAL TUNING ----------
    params.adaptiveThreshWinSizeMin = 7
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 6

    params.minMarkerPerimeterRate = 0.015
    params.maxMarkerPerimeterRate = 4.0

    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.005

    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01

    detector = None
    if hasattr(aruco, "ArucoDetector"):
        try:
            detector = aruco.ArucoDetector(aruco_dict, params)
        except Exception:
            detector = None

    return aruco, aruco_dict, detector, params

def _to_gray(img: np.ndarray) -> np.ndarray | None:
    """
    Accepts:
      - HxW uint8 (gray)
      - HxWx3 uint8 (BGR)
    Returns:
      - gray uint8 contiguous or None
    """
    if img is None:
        return None
    img = np.ascontiguousarray(img)
    if img.size == 0:
        return None
    if img.dtype != np.uint8:
        img = img.astype(np.uint8, copy=False)

    if img.ndim == 2:
        return np.ascontiguousarray(img)
    if img.ndim == 3 and img.shape[2] == 3:
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return np.ascontiguousarray(g)
    return None


def _downscale_gray(gray: np.ndarray, ds: int) -> tuple[np.ndarray, int]:
    """
    Downscale gray by ds using INTER_AREA.
    Guards against too small result.
    Returns (small_gray, effective_ds)
    """
    H, W = gray.shape[:2]
    ds = int(ds) if ds else 1
    if ds < 1:
        ds = 1
    if ds > 8:
        ds = 8

    if ds == 1:
        return gray, 1

    newW = W // ds
    newH = H // ds

    # avoid tiny images that kill detection
    if newW < 80 or newH < 80:
        return gray, 1

    small = cv2.resize(gray, (newW, newH), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(small), ds


def _pick_best_marker(corners, ids, wanted_ids: set[int]):
    """
    Select best candidate among wanted_ids by max contour area.
    corners: list of (1,4,2) arrays in SMALL coords
    ids: Nx1
    Return: (mid, c_small_4x2_float32, area_small) or None
    """
    if ids is None or len(ids) == 0:
        return None

    best = None
    for i in range(len(ids)):
        mid = int(ids[i][0])
        if mid not in wanted_ids:
            continue
        c = corners[i].reshape(-1, 2)
        c32 = np.ascontiguousarray(c, dtype=np.float32)
        area = float(cv2.contourArea(c32))
        if best is None or area > best[2]:
            best = (mid, c32, area)
    return best


def _detect_one(
    img: np.ndarray,
    wanted_ids: set[int],
    marker_len_m: float,
    K: np.ndarray | None,
    D: np.ndarray | None,
    downscale: int,
    aruco, aruco_dict, detector, params,
):
    """
    Decode input image (gray or bgr), detect on downscaled,
    return coords in FULL-res image.
    """
    gray_full = _to_gray(img)
    if gray_full is None:
        return None

    H, W = gray_full.shape[:2]

    # downscale for detection
    small, ds_eff = _downscale_gray(gray_full, int(downscale) if downscale is not None else 1)

    # detectMarkers (expects gray)
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(small)
    else:
        corners, ids, _ = aruco.detectMarkers(small, aruco_dict, parameters=params)

    best = _pick_best_marker(corners, ids, wanted_ids)
    if best is None:
        return None

    mid, c_small, area_small = best

    # scale corners back to full-res coords
    if ds_eff != 1:
        c_full = c_small * float(ds_eff)
    else:
        c_full = c_small

    c_full = np.ascontiguousarray(c_full, dtype=np.float32)

    cx = float(c_full[:, 0].mean())
    cy = float(c_full[:, 1].mean())

    tvec = None
    dist_m = None
    pose_ok = False

    # ❌ НИКАКИХ "фейковых метров"
    if not pose_ok:
        dist_m = None

    side = "R" if (tvec is not None and tvec[0] > 0.0) else ("L" if cx < (W * 0.5) else "R")

    return {
            "id": int(mid),
            "cx": float(cx),
            "cy": float(cy),
            "dist_m": dist_m,
            "pose_ok": bool(pose_ok),
            "tvec": tvec,
            "corners": c_full.tolist(),
            "img_wh": (int(W), int(H)),
        }
                                   
def run(conn: Connection):

    aruco, aruco_dict, detector, params = _make_aruco_detector()

    # можно крутить из state через message, но тут фиксируем как "то состояние"
    downscale = 1  # НЕ уменьшаем (раз у тебя и так crop)

    while True:
        try:
            msg = conn.recv()
        except EOFError:
            break
        except Exception:
            continue

        if msg is None:
            break

        # ожидаем: (jpg_bytes, wanted_ids, marker_len_m, K, D)
        try:
            jpg_bytes, wanted_ids, marker_len_m, K, D = msg
        except Exception:
            continue

        # decode jpg -> gray
        try:
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
            if img is None or img.size == 0:
                conn.send(None)
                continue
        except Exception:
            conn.send(None)
            continue

        gray = np.ascontiguousarray(img)
        H, W = gray.shape[:2]

        # downscale (обычно 1)
        small, ds_eff = _downscale_gray(gray, downscale)

        # detect
        try:
            if detector is not None:
                corners, ids, _ = detector.detectMarkers(small)
            else:
                corners, ids, _ = aruco.detectMarkers(small, aruco_dict, parameters=params)
        except Exception:
            conn.send(None)
            continue

        if ids is None or len(ids) == 0:
            conn.send(None)
            continue

        wanted_set = set(int(x) for x in (wanted_ids or []))
        best = _pick_best_marker(corners, ids, wanted_set)
        if best is None:
            conn.send(None)
            continue

        mid, c_small, area_small = best

        # corners -> full coords of CROPPED image
        if ds_eff != 1:
            c_full = c_small * float(ds_eff)
        else:
            c_full = c_small
        c_full = np.ascontiguousarray(c_full, dtype=np.float32)

        cx = float(c_full[:, 0].mean())
        cy = float(c_full[:, 1].mean())

        tvec = None
        dist_m = None
        pose_ok = False

        # ---- POSE (как "то состояние"): считаем на CROPPED, но K/D от FULL (без пересчёта) ----
        try:
            if K is not None:
                K = np.asarray(K, dtype=np.float64)
            if D is not None:
                D = np.asarray(D, dtype=np.float64)

            if (K is not None) and (K.size == 9) and (marker_len_m is not None):
                # estimatePoseSingleMarkers expects corners shape: (N, 1, 4, 2)
                c_in = c_full.reshape(1, 1, 4, 2).astype(np.float32)

                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    c_in,
                    float(marker_len_m),
                    K,
                    D if D is not None else None,
                )
                if tvecs is not None and len(tvecs) > 0:
                    tv = np.asarray(tvecs[0, 0], dtype=np.float64)
                    tvec = (float(tv[0]), float(tv[1]), float(tv[2]))
                    dist_m = float(np.linalg.norm(tv))
                    pose_ok = True
        except Exception:
            # если поза не посчиталась — оставим None
            tvec = None
            dist_m = None
            pose_ok = False

        side = "R" if (tvec is not None and float(tvec[0]) > 0.0) else ("L" if cx < (W * 0.5) else "R")

        out = {
            "id": int(mid),
            "cx": float(cx),
            "cy": float(cy),
            "dist_m": dist_m,
            "pose_ok": bool(pose_ok),
            "tvec": tvec,
            "corners": c_full.tolist(),
            "img_wh": (int(W), int(H)),  # это размеры CROPPED
            "side": side,
        }

        try:
            conn.send(out)
        except Exception:
            pass
if __name__ == "__main__":
    pass