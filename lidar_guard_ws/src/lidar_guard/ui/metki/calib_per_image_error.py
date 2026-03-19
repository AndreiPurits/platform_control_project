# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import json
import glob
import csv
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import cv2


# =========================================================
# НАСТРОЙКИ ДОСКИ (как ты сказал)
# =========================================================
PATTERN_SIZE = (9, 6)   # (cols, rows) ВНУТРЕННИЕ углы
SQUARE_SIZE_M = 0.028   # размер клетки в метрах
ERR_MAX_PX = 0.7        # удаляем кадры с ошибкой > 0.7 px
# =========================================================


@dataclass
class Det:
    path: str
    img_size: Tuple[int, int]  # (w,h)
    corners: np.ndarray        # (N,1,2) float32 refined
    err_px: float = -1.0       # per-image mean reprojection error (px)


def _ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


def _as_bgr(img: np.ndarray) -> np.ndarray:
    if img is None:
        return img
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _read_image(path: str) -> Optional[np.ndarray]:
    return cv2.imread(path, cv2.IMREAD_COLOR)


def _safe_remove(path: str) -> bool:
    try:
        os.remove(path)
        return True
    except Exception as e:
        print(f"[WARN] can't remove {path}: {e}")
        return False


def _find_chessboard(
    img_bgr: np.ndarray,
    pattern_size: Tuple[int, int],
    refine: bool = True,
) -> Optional[np.ndarray]:
    """
    pattern_size = (cols, rows) внутренних углов!
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )

    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok or corners is None:
        return None

    if refine:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-6)
        win = (11, 11)
        zero = (-1, -1)
        corners = cv2.cornerSubPix(gray, corners, win, zero, crit)

    return corners.astype(np.float32)


def _object_points(pattern_size: Tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_m)
    return objp


def _reproj_error_for_view(
    objp: np.ndarray,
    corners: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> float:
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    obs = corners.reshape(-1, 2)
    e = np.linalg.norm(proj - obs, axis=1)
    return float(np.mean(e))


def main():
    # =========================
    # ПАПКИ
    # =========================
    img_dir = "calib_imgs"   # папка с фото
    out_dir = "calib_out"    # сюда результаты
    vis_dir = os.path.join(out_dir, "vis")

    # какие расширения искать
    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tif", "*.tiff")

    # рисовать ещё и reprojection точки (projected) поверх?
    draw_projected = True

    # =========================
    _ensure_dir(out_dir)
    _ensure_dir(vis_dir)

    pattern_size = PATTERN_SIZE
    square_size_m = float(SQUARE_SIZE_M)

    paths: List[str] = []
    for e in exts:
        paths += glob.glob(os.path.join(img_dir, e))
    paths = sorted(paths)

    if not paths:
        raise SystemExit(f"[ERR] No images found in {img_dir}. Put images there.")

    objp = _object_points(pattern_size, square_size_m)

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    dets: List[Det] = []

    removed_miss = 0
    removed_err = 0

    # =========================================================
    # 1) Детект углов + УДАЛЕНИЕ MISS
    # =========================================================
    for p in paths:
        img = _read_image(p)
        if img is None:
            print(f"[WARN] can't read: {p}")
            continue

        h, w = img.shape[:2]
        corners = _find_chessboard(img, pattern_size, refine=True)

        if corners is None:
            print(f"[MISS][DEL] {os.path.basename(p)}")
            # виз "MISS"
            vis = _as_bgr(img).copy()
            cv2.putText(vis, "MISS (deleted)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(vis_dir, os.path.basename(p)), vis)
            if _safe_remove(p):
                removed_miss += 1
            continue

        objpoints.append(objp.copy())
        imgpoints.append(corners)
        dets.append(Det(path=p, img_size=(w, h), corners=corners))

        # визуализация найденных углов
        vis = _as_bgr(img).copy()
        cv2.drawChessboardCorners(vis, pattern_size, corners, True)
        cv2.putText(vis, "OK (corners found)", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(vis_dir, os.path.basename(p)), vis)

    if not dets:
        raise SystemExit("[ERR] No valid images after MISS удаления.")

    # =========================================================
    # 2) Калибровка на оставшихся
    # =========================================================
    img_size = dets[0].img_size  # (w,h)
    print(f"[INFO] Calibrating on {len(dets)} images, img_size={img_size}")

    flags = 0
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None, flags=flags
    )

    print(f"[INFO] RMS (OpenCV): {ret:.4f} px")
    print(f"[INFO] K=\n{K}")
    print(f"[INFO] dist={dist.ravel()}")

    # =========================================================
    # 3) Ошибка по каждому кадру + УДАЛЕНИЕ > 0.7
    # =========================================================
    errs = []
    kept_dets: List[Det] = []

    for i, d in enumerate(dets):
        err = _reproj_error_for_view(objp, d.corners, rvecs[i], tvecs[i], K, dist)
        d.err_px = float(err)
        errs.append(d.err_px)

        img = _read_image(d.path)
        if img is None:
            continue
        vis = _as_bgr(img).copy()

        # observed corners (зелёные)
        cv2.drawChessboardCorners(vis, pattern_size, d.corners, True)

        if draw_projected:
            proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, dist)
            proj = proj.reshape(-1, 2).astype(np.int32)
            for (x, y) in proj:
                cv2.circle(vis, (int(x), int(y)), 3, (0, 0, 255), -1, cv2.LINE_AA)

        base = os.path.basename(d.path)

        if d.err_px > float(ERR_MAX_PX):
            # помечаем и удаляем
            cv2.putText(vis, f"BAD err={d.err_px:.3f}px > {ERR_MAX_PX} (deleted)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(vis_dir, base), vis)

            print(f"[BAD][DEL] {base}  err={d.err_px:.3f}px")
            if _safe_remove(d.path):
                removed_err += 1
            continue

        # good
        cv2.putText(vis, f"GOOD err={d.err_px:.3f}px", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(vis_dir, base), vis)
        kept_dets.append(d)

    if errs:
        mean_err = float(np.mean(errs))
        med_err = float(np.median(errs))
        p90 = float(np.percentile(errs, 90))
    else:
        mean_err = med_err = p90 = 0.0

    print(f"[INFO] per-image mean error (BEFORE prune by err): mean={mean_err:.4f}px  median={med_err:.4f}px  p90={p90:.4f}px")
    print(f"[INFO] deleted: MISS={removed_miss}  BAD_ERR>{ERR_MAX_PX}={removed_err}")
    print(f"[INFO] kept after deletion: {len(kept_dets)}")

    # =========================================================
    # 4) CSV (по тем, что мы СМОТРЕЛИ; можно писать только kept — я пишу ВСЕ ошибки до удаления)
    # =========================================================
    csv_path = os.path.join(out_dir, "per_image_errors.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["filename", "err_mean_px", "status"])
        # сортируем по err у dets (включая те, что удалили по err)
        for d in sorted(dets, key=lambda x: x.err_px, reverse=True):
            status = "DELETED_BAD_ERR" if (d.err_px > float(ERR_MAX_PX)) else "KEPT"
            # MISS уже удалены и не попали в dets
            wr.writerow([os.path.basename(d.path), f"{d.err_px:.6f}", status])
    print(f"[OK] wrote {csv_path}")

    # =========================================================
    # 5) JSON
    # =========================================================
    js = {
        "pattern_size_inner_corners": list(pattern_size),
        "square_size_m": float(square_size_m),
        "opencv_rms_px": float(ret),
        "per_image_mean_px_before_prune": float(mean_err),
        "per_image_median_px_before_prune": float(med_err),
        "per_image_p90_px_before_prune": float(p90),
        "err_delete_threshold_px": float(ERR_MAX_PX),
        "deleted_miss": int(removed_miss),
        "deleted_bad_err": int(removed_err),
        "n_images_used_in_calibrate": int(len(dets)),
        "n_images_kept_after_delete": int(len(kept_dets)),
        "K": K.tolist(),
        "dist": dist.ravel().tolist(),
    }
    json_path = os.path.join(out_dir, "calib_result.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(js, f, ensure_ascii=False, indent=2)
    print(f"[OK] wrote {json_path}")

    # =========================================================
    # 6) Печать худших (из dets, т.е. до удаления по err)
    # =========================================================
    worst = sorted(dets, key=lambda x: x.err_px, reverse=True)[:30]
    print("[WORST 5]")
    for d in worst[:5]:
        print(f"  {os.path.basename(d.path)}  err={d.err_px:.3f}px  {'(DELETED)' if d.err_px > float(ERR_MAX_PX) else ''}")


if __name__ == "__main__":
    main()