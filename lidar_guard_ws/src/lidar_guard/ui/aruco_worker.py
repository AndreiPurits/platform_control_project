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
    Compatible with OpenCV 4.6+ and newer ArucoDetector API.
    """
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available in this OpenCV build")

    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Parameters object
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    elif hasattr(aruco, "DetectorParameters"):
        params = aruco.DetectorParameters()
    else:
        params = None

    # --- tune params for far/small markers ---
    if params is not None:
        # Subpixel corner refinement -> helps a lot at distance
        try:
            params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        except Exception:
            pass

        # Adaptive threshold windows (robust to uneven lighting)
        for name, val in [
            ("adaptiveThreshWinSizeMin", 3),
            ("adaptiveThreshWinSizeMax", 63),
            ("adaptiveThreshWinSizeStep", 10),
        ]:
            try:
                setattr(params, name, val)
            except Exception:
                pass

        # Allow smaller markers (careful: too low => false positives)
        try:
            params.minMarkerPerimeterRate = 0.015  # a bit lower than typical
        except Exception:
            pass

        # Corner distance & border margins
        try:
            params.minCornerDistanceRate = 0.01
        except Exception:
            pass

        # Often helps with weak edges
        try:
            params.perspectiveRemovePixelPerCell = 8
        except Exception:
            pass
        try:
            params.perspectiveRemoveIgnoredMarginPerCell = 0.20
        except Exception:
            pass

    # New API (4.7+): ArucoDetector
    detector = None
    if hasattr(aruco, "ArucoDetector") and params is not None:
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

    # pose estimate in FULL coords (K/D must correspond to full frame)
    if K is not None and D is not None:
        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [c_full], float(marker_len_m), K, D
            )
            tv = tvecs[0][0]
            tvec = (float(tv[0]), float(tv[1]), float(tv[2]))
            dist_m = float(tv[2])
        except Exception:
            tvec = None
            dist_m = None

    # fallback monotonic "distance" if pose failed
    if dist_m is None:
        area_full_est = max(1e-9, float(area_small) * float(ds_eff * ds_eff))
        dist_m = 1.0 / max(1e-6, np.sqrt(area_full_est))

    side = "R" if (tvec is not None and tvec[0] > 0.0) else ("L" if cx < (W * 0.5) else "R")

    return {
        "id": int(mid),
        "cx": float(cx),
        "cy": float(cy),
        "area_small": float(area_small),
        "downscale": int(ds_eff),
        "dist_m": float(dist_m),
        "side": side,
        "tvec": tvec,
        "corners": c_full.tolist(),  # 4x2 in FULL coords
        "img_wh": (int(W), int(H)),
    }


def run(conn: Connection):
    """
    Protocol:
      None -> exit
      (img_bytes, wanted_ids_list, marker_len_m, K, D, downscale_int)
        -> send(result_dict_or_None)
    """

    # Stabilize OpenCV threading
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass
    try:
        cv2.ocl.setUseOpenCL(False)
    except Exception:
        pass

    aruco, aruco_dict, detector, params = _make_aruco_detector()

    while True:
        msg = conn.recv()
        if msg is None:
            break

        # back-compat: 5 fields -> downscale=2
        try:
            if len(msg) == 5:
                img_bytes, wanted_ids, marker_len_m, K, D = msg
                downscale = 2
            else:
                img_bytes, wanted_ids, marker_len_m, K, D, downscale = msg
        except Exception:
            conn.send(None)
            continue

        # decode (PNG/JPEG) -> gray or bgr
        try:
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)  # can be HxW or HxWx3
        except Exception:
            img = None

        if img is None or getattr(img, "size", 0) == 0:
            conn.send(None)
            continue

        # K/D as contiguous float64
        if K is not None:
            K = np.ascontiguousarray(K, dtype=np.float64)
        if D is not None:
            D = np.ascontiguousarray(D, dtype=np.float64)

        wanted = set(int(x) for x in (wanted_ids or []))

        try:
            res = _detect_one(
                img=img,
                wanted_ids=wanted,
                marker_len_m=float(marker_len_m),
                K=K,
                D=D,
                downscale=int(downscale) if downscale is not None else 1,
                aruco=aruco,
                aruco_dict=aruco_dict,
                detector=detector,
                params=params,
            )
        except Exception:
            res = None

        conn.send(res)


if __name__ == "__main__":
    pass