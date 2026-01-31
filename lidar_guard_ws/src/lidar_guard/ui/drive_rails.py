# -*- coding: utf-8 -*-
"""
drive_rails.py

Рейлы/главная дорога поверх road_bin (маски сегментации).

Требования, которые соблюдены:
- Никакого управления моторами.
- Уважает существующие переменные:
    st.rails_enabled (bool)   -> включает/выключает рейлы
    st.nav_mode               -> режимы MODE_MARKERS/MODE_ROAD/MODE_TRAJ
- В MODE_MARKERS рейлы НЕ считаем и НЕ рисуем (даже если rails_enabled=True),
  потому что в этом режиме камера 4K и сегментации дороги нет.
- В MODE_ROAD / MODE_TRAJ (если road_bin есть) — считаем/рисуем.
- Все состояния пишем только в st._main_* и st._rail_* (как ты уже используешь).
"""

from __future__ import annotations

import time as _time
from collections import deque
from typing import Optional, Tuple, List

import numpy as np
import cv2


# ------------------------------------------------------------
# MODE names (совпадают с тем, что уже в UI/VideoController)
# ------------------------------------------------------------
MODE_MARKERS = "markers"
MODE_ROAD = "road"
MODE_TRAJ = "traj"


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _lp(prev, target, dt, tau):
    """Экспоненциальный low-pass по dt."""
    tau = max(1e-4, float(tau))
    a = 1.0 - float(np.exp(-float(dt) / tau))
    return float(prev + a * (float(target) - float(prev)))


def _median_int(vals) -> Optional[int]:
    vals = [int(v) for v in vals if v is not None]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def _rails_active_for_state(st) -> bool:
    """Единая точка решения: считаем/рисуем рейлы или нет."""
    if not bool(getattr(st, "rails_enabled", False)):
        return False
    mode = str(getattr(st, "nav_mode", MODE_MARKERS) or MODE_MARKERS)
    if mode == MODE_MARKERS:
        return False
    return True


# ============================================================
# 1) Main road detection from road_bin_crop
# ============================================================
def compute_main_road_from_roadbin(st, road_bin_crop: np.ndarray, crop_y0_full: int) -> None:
    """
    Ищет "главную дорогу" как самый дальний (верхний) "хороший" сегмент,
    предпочитая центральные кандидаты.

    Пишет:
      st._main_line = { y_scan_crop, xL_crop, xR_crop, slope_out }
      st._main_px_crop = (xL, xR)
      st._main_px_full = (xL, xR)              # (x те же, y хранится отдельно)
      st._main_y_scan_full = crop_y0_full + y  # абсолютная y в full кадре
      st.main_trace_frac = доля попаданий по коридору
    """
    # всегда сбрасываем (чтобы оверлей не "залипал")
    st._main_line = None
    st._main_px_crop = None
    st._main_px_full = None
    st._main_y_scan_full = None

    if not _rails_active_for_state(st):
        return

    if road_bin_crop is None or getattr(road_bin_crop, "size", 0) == 0:
        return

    rb = (road_bin_crop.astype(np.uint8) > 0).astype(np.uint8)
    Hc, W = rb.shape[:2]
    if Hc < 8 or W < 16:
        return

    # --- поиск "горизонта" (в crop координатах) ---
    y_search_top_frac = float(getattr(st, "main_y_search_top_frac", 0.03) or 0.03)
    y_search_bot_frac = float(getattr(st, "main_y_search_bot_frac", 0.45) or 0.45)

    y_search_top = int(_clamp(int(Hc * y_search_top_frac), 0, Hc - 2))
    y_search_bot = int(_clamp(int(Hc * y_search_bot_frac), y_search_top + 1, Hc - 1))

    # критерии кандидата
    min_row_fill = float(getattr(st, "main_min_row_fill", 0.10) or 0.10)
    min_row_fill = float(_clamp(min_row_fill, 0.01, 0.80))

    min_width = int(getattr(st, "main_min_width_px", 120) or 120)
    min_width = int(_clamp(min_width, 30, W))

    min_seg_px = int(getattr(st, "main_min_seg_px", 24) or 24)
    min_seg_px = int(_clamp(min_seg_px, 4, W))

    # --- helper: largest run of 1s in row ---
    def _largest_run(row_u8: np.ndarray) -> Optional[Tuple[int, int, int]]:
        idx = np.where(row_u8 > 0)[0]
        if idx.size == 0:
            return None
        splits = np.where(np.diff(idx) > 1)[0]
        runs = []
        start = 0
        for s in splits:
            runs.append(idx[start:s + 1])
            start = s + 1
        runs.append(idx[start:])

        best = None
        best_len = 0
        for r in runs:
            ln = int(r.size)
            if ln > best_len:
                best_len = ln
                best = r
        if best is None or best_len < min_seg_px:
            return None
        return int(best[0]), int(best[-1]), int(best_len)

    cx_frame = 0.5 * float(W)

    # предпочтение "центральных"
    center_gate_px = float(getattr(st, "main_center_gate_px", 0.14 * W) or (0.14 * W))
    center_gate_px = float(_clamp(center_gate_px, 0.03 * W, 0.40 * W))

    # веса скоринга (минимизируем)
    w_y = float(getattr(st, "main_score_w_y", 2.2) or 2.2)
    w_c = float(getattr(st, "main_score_w_center", 1.0) or 1.0)
    w_w = float(getattr(st, "main_score_w_width", 0.25) or 0.25)

    candidates: List[Tuple[float, int, int, int, int, float]] = []
    denom_y = max(1.0, float(y_search_bot - y_search_top))
    for y in range(y_search_top, y_search_bot):
        row = rb[y, :]
        fill = float(row.mean())
        if fill < min_row_fill:
            continue

        run = _largest_run(row)
        if run is None:
            continue

        xl, xr, _ = run
        width = int(xr - xl + 1)
        if width < min_width:
            continue

        cx = 0.5 * float(xl + xr)
        dist_c = abs(cx - cx_frame)

        y_n = float(y - y_search_top) / denom_y
        c_n = float(dist_c) / max(1.0, float(center_gate_px))
        w_n = float(width) / max(1.0, float(W))

        score = (w_y * y_n) + (w_c * c_n) - (w_w * w_n)
        candidates.append((float(score), int(y), int(xl), int(xr), int(width), float(dist_c)))

    if not candidates:
        return

    central = [c for c in candidates if c[5] <= center_gate_px]
    if central:
        # самый верхний, потом ближе к центру, потом шире
        central.sort(key=lambda t: (t[1], t[5], -t[4]))
        _, y_scan, xL, xR, _, _ = central[0]
    else:
        candidates.sort(key=lambda t: t[0])
        _, y_scan, xL, xR, _, _ = candidates[0]

    y_scan = int(_clamp(y_scan, 0, Hc - 2))
    xL = int(_clamp(xL, 0, W - 1))
    xR = int(_clamp(xR, 0, W - 1))
    if xR < xL:
        xL, xR = xR, xL

    # --- валидация вниз "коридором" ---
    base_deg = float(getattr(st, "main_base_deg", 12.0) or 12.0)
    slope_out = float(np.tan(np.deg2rad(base_deg)))

    step = int(getattr(st, "main_trace_step_px", 8) or 8)
    step = int(_clamp(step, 4, 16))

    need_frac = float(getattr(st, "main_need_frac", 0.85) or 0.85)
    need_frac = float(_clamp(need_frac, 0.30, 0.98))

    max_expand_frac = float(getattr(st, "main_max_expand_frac", 0.60) or 0.60)
    max_expand_px = float(_clamp(max_expand_frac, 0.05, 0.95)) * float(W)

    n_all = 0
    n_hit = 0
    for y in range(Hc - 1, y_scan, -step):
        dy = float((Hc - 1) - y)
        expand = float(_clamp(slope_out * dy, 0.0, max_expand_px))

        xl = int(_clamp(int(xL - expand), 0, W - 1))
        xr = int(_clamp(int(xR + expand), 0, W - 1))
        if xr <= xl:
            continue

        seg = rb[y, xl:xr + 1]
        n_all += 1
        if seg.size > 0 and float(seg.mean()) >= 0.5:
            n_hit += 1

    frac = float(n_hit) / float(max(1, n_all))
    st.main_trace_frac = float(frac)
    if frac < need_frac:
        return

    st._main_line = {
        "y_scan_crop": int(y_scan),
        "xL_crop": int(xL),
        "xR_crop": int(xR),
        "slope_out": float(slope_out),
    }
    st._main_px_crop = (int(xL), int(xR))
    st._main_px_full = (int(xL), int(xR))
    st._main_y_scan_full = int(crop_y0_full + int(y_scan))


# ============================================================
# 2) Rail anchors update from bottom of road_bin_crop
# ============================================================
def update_rail_anchors_from_roadbin(st, road_bin_crop: np.ndarray, crop_y0_full: int) -> None:
    """
    Вычисляет нижние якоря рейлов (xL_bot, xR_bot) устойчиво даже если низ "залит".
    Пишет:
      st._rail_mask_L_crop / R_crop
      st._rail_mask_L_full / R_full

    Дополнительно хранит сглаживание/историю:
      st._railL_hist, st._railR_hist
      st._railLx_f, _railLy_f, _railRx_f, _railRy_f
      st._rail_anchor_raw_prev, _rail_mask_upd_ts, _rail_anchor_prev_ts
    """
    if not _rails_active_for_state(st):
        return

    if road_bin_crop is None or getattr(road_bin_crop, "size", 0) == 0:
        return

    ml = getattr(st, "_main_line", None)
    if not ml:
        return

    rb = (road_bin_crop.astype(np.uint8) > 0).astype(np.uint8)
    Hc, W = rb.shape[:2]
    if Hc < 8 or W < 16:
        return

    # --- throttle обновления ---
    upd_s = float(getattr(st, "rail_mask_update_s", 0.8) or 0.8)
    upd_s = float(_clamp(upd_s, 0.05, 10.0))

    now = _time.monotonic()
    last = float(getattr(st, "_rail_mask_upd_ts", 0.0) or 0.0)
    if last > 0.0 and (now - last) < upd_s:
        return
    st._rail_mask_upd_ts = float(now)

    # --- dt по реальному времени ---
    prev_ts = float(getattr(st, "_rail_anchor_prev_ts", 0.0) or 0.0)
    dt = 0.12 if prev_ts <= 0.0 else float(_clamp(now - prev_ts, 0.02, 1.0))
    st._rail_anchor_prev_ts = float(now)

    # y_scan, чтобы окно снизу не лезло слишком высоко
    y_scan = int(ml.get("y_scan_crop", int(Hc * 0.25)))
    y_scan = int(_clamp(y_scan, 0, Hc - 2))

    # --- нижняя строка ---
    yb = Hc - 1
    rowb = rb[yb, :]
    xs = np.where(rowb > 0)[0]

    sat_thr = float(getattr(st, "rail_mask_bottom_saturated_thr", 0.98) or 0.98)
    sat_thr = float(_clamp(sat_thr, 0.60, 0.999))
    bottom_saturated = (float(rowb.mean()) >= sat_thr)

    # устойчивые параметры
    win_h = int(getattr(st, "rail_bottom_window_h", 35) or 35)
    win_h = int(_clamp(win_h, 8, 80))

    min_width = int(getattr(st, "rail_min_width_px", 60) or 60)
    min_width = int(_clamp(min_width, 20, W))

    xL_bot = xR_bot = None
    yL_bot = yR_bot = yb

    if (xs.size >= 2) and (not bottom_saturated):
        xL_bot = int(xs.min())
        xR_bot = int(xs.max())
    else:
        # окно строк снизу: ищем строки, которые НЕ залиты полностью
        y0w = max(y_scan + 1, yb - win_h)
        cand_Lx, cand_Ly, cand_Rx, cand_Ry = [], [], [], []

        for y in range(yb, y0w - 1, -1):
            row = rb[y, :]
            fill = float(row.mean())
            if fill >= sat_thr:
                continue
            xs2 = np.where(row > 0)[0]
            if xs2.size < 2:
                continue
            cand_Lx.append(int(xs2.min())); cand_Ly.append(int(y))
            cand_Rx.append(int(xs2.max())); cand_Ry.append(int(y))

        if len(cand_Lx) >= 3:
            xL_bot = _median_int(cand_Lx)
            yL_bot = _median_int(cand_Ly) if cand_Ly else yb
        else:
            xL_bot, yL_bot = 0, yb

        if len(cand_Rx) >= 3:
            xR_bot = _median_int(cand_Rx)
            yR_bot = _median_int(cand_Ry) if cand_Ry else yb
        else:
            xR_bot, yR_bot = (W - 1), yb

    if xL_bot is None or xR_bot is None:
        return

    xL_bot = int(_clamp(int(xL_bot), 0, W - 1))
    xR_bot = int(_clamp(int(xR_bot), 0, W - 1))
    yL_bot = int(_clamp(int(yL_bot), 0, Hc - 1))
    yR_bot = int(_clamp(int(yR_bot), 0, Hc - 1))

    if xR_bot < xL_bot:
        xL_bot, xR_bot = xR_bot, xL_bot
        yL_bot, yR_bot = yR_bot, yL_bot

    if (xR_bot - xL_bot) < min_width:
        return

    # --- history median ---
    hn = int(getattr(st, "rail_mask_hist_n", 7) or 7)
    hn = int(_clamp(hn, 1, 15))
    if getattr(st, "_railL_hist", None) is None:
        st._railL_hist = deque(maxlen=hn)
    if getattr(st, "_railR_hist", None) is None:
        st._railR_hist = deque(maxlen=hn)

    st._railL_hist.append((int(xL_bot), int(yL_bot)))
    st._railR_hist.append((int(xR_bot), int(yR_bot)))

    xL_med = _median_int([p[0] for p in st._railL_hist])
    yL_med = _median_int([p[1] for p in st._railL_hist])
    xR_med = _median_int([p[0] for p in st._railR_hist])
    yR_med = _median_int([p[1] for p in st._railR_hist])
    if xL_med is None or xR_med is None or yL_med is None or yR_med is None:
        return

    # --- deadband ---
    db_px = int(getattr(st, "rail_anchor_deadband_px", 6) or 6)
    db_px = int(_clamp(db_px, 0, 40))
    prev_raw = getattr(st, "_rail_anchor_raw_prev", None)
    cur_raw = (int(xL_med), int(yL_med), int(xR_med), int(yR_med))
    if prev_raw is not None and db_px > 0:
        d = abs(cur_raw[0] - prev_raw[0]) + abs(cur_raw[1] - prev_raw[1]) + abs(cur_raw[2] - prev_raw[2]) + abs(cur_raw[3] - prev_raw[3])
        if d < db_px:
            xL_med, yL_med, xR_med, yR_med = prev_raw
    st._rail_anchor_raw_prev = (int(xL_med), int(yL_med), int(xR_med), int(yR_med))

    # --- smoothing + clamp jump ---
    tau = float(getattr(st, "rail_mask_tau", 3.5) or 3.5)
    tau = float(_clamp(tau, 0.2, 8.0))

    prevLx = float(getattr(st, "_railLx_f", xL_med) or xL_med)
    prevLy = float(getattr(st, "_railLy_f", yL_med) or yL_med)
    prevRx = float(getattr(st, "_railRx_f", xR_med) or xR_med)
    prevRy = float(getattr(st, "_railRy_f", yR_med) or yR_med)

    max_jump = float(getattr(st, "rail_anchor_max_jump_px", 35.0) or 35.0)
    max_jump = float(_clamp(max_jump, 5.0, 200.0))

    def clamp_jump(prev, target):
        d = float(target) - float(prev)
        if d > max_jump:
            return float(prev + max_jump)
        if d < -max_jump:
            return float(prev - max_jump)
        return float(target)

    xL_t = clamp_jump(prevLx, xL_med)
    yL_t = clamp_jump(prevLy, yL_med)
    xR_t = clamp_jump(prevRx, xR_med)
    yR_t = clamp_jump(prevRy, yR_med)

    st._railLx_f = _lp(prevLx, xL_t, dt, tau)
    st._railLy_f = _lp(prevLy, yL_t, dt, tau)
    st._railRx_f = _lp(prevRx, xR_t, dt, tau)
    st._railRy_f = _lp(prevRy, yR_t, dt, tau)

    # сохранить якоря
    st._rail_mask_L_crop = (int(round(st._railLx_f)), int(round(st._railLy_f)))
    st._rail_mask_R_crop = (int(round(st._railRx_f)), int(round(st._railRy_f)))
    st._rail_mask_L_full = (int(round(st._railLx_f)), int(crop_y0_full + int(round(st._railLy_f))))
    st._rail_mask_R_full = (int(round(st._railRx_f)), int(crop_y0_full + int(round(st._railRy_f))))


# ============================================================
# 3) Overlays
# ============================================================
def draw_rails_overlay(vis_bgr: np.ndarray, st) -> None:
    """
    Синие рейлы:
      TOP: из st._main_line (xL_top/xR_top на y_top)
      BOT: из st._rail_mask_*_full (xL_bot/xR_bot)
    """
    if vis_bgr is None:
        return
    if not _rails_active_for_state(st):
        return

    ml = getattr(st, "_main_line", None)
    if not ml:
        return

    L = getattr(st, "_rail_mask_L_full", None)
    R = getattr(st, "_rail_mask_R_full", None)
    if not L or not R:
        return

    h, w = vis_bgr.shape[:2]
    crop_y0 = int(getattr(st, "_seg_crop_y0", 0) or 0)

    y_scan_crop = int(ml.get("y_scan_crop", 0))
    y_top = int(_clamp(crop_y0 + y_scan_crop, 0, h - 1))

    xL_top = int(_clamp(int(ml.get("xL_crop", 0)), 0, w - 1))
    xR_top = int(_clamp(int(ml.get("xR_crop", w - 1)), 0, w - 1))
    if xR_top < xL_top:
        xL_top, xR_top = xR_top, xL_top

    xL_bot, _yL_bot = L
    xR_bot, _yR_bot = R
    xL_bot = int(_clamp(int(xL_bot), 0, w - 1))
    xR_bot = int(_clamp(int(xR_bot), 0, w - 1))
    if xR_bot < xL_bot:
        xL_bot, xR_bot = xR_bot, xL_bot

    rail_y_start = float(getattr(st, "rail_y_start", 1.0) or 1.0)
    rail_y_start = float(_clamp(rail_y_start, 0.70, 1.0))
    y_bot = int(_clamp(int(h * rail_y_start), 0, h - 1))
    y_bot = int(max(y_top + 1, y_bot))
    if y_top >= y_bot:
        return

    inset = float(getattr(st, "rail_inset_px", 25.0) or 25.0)
    inset = float(_clamp(inset, 0.0, 200.0))

    xL_top = int(_clamp(xL_top, inset, w - 1 - inset))
    xR_top = int(_clamp(xR_top, inset, w - 1 - inset))
    xL_bot = int(_clamp(xL_bot, inset, w - 1 - inset))
    xR_bot = int(_clamp(xR_bot, inset, w - 1 - inset))

    step = int(getattr(st, "rail_step_px", 6) or 6)
    step = int(_clamp(step, 4, 16))

    def lerp(a, b, t):
        return (1.0 - t) * a + t * b

    denom = float(max(1, (y_bot - y_top)))

    ptsL, ptsR = [], []
    for y in range(y_bot, y_top, -step):
        t = float(y - y_top) / denom
        xl = lerp(float(xL_top), float(xL_bot), t)
        xr = lerp(float(xR_top), float(xR_bot), t)

        xl = float(_clamp(xl, inset, w - 1 - inset))
        xr = float(_clamp(xr, inset, w - 1 - inset))
        if xr <= xl + 5:
            continue

        ptsL.append((int(xl), int(y)))
        ptsR.append((int(xr), int(y)))

    # низ рейлов: в края кадра
    if ptsL:
        ptsL[0] = (0, int(y_bot))
    if ptsR:
        ptsR[0] = (w - 1, int(y_bot))

    if len(ptsL) >= 2:
        cv2.polylines(vis_bgr, [np.array(ptsL, np.int32)], False, (255, 0, 0), 3, cv2.LINE_AA)
    if len(ptsR) >= 2:
        cv2.polylines(vis_bgr, [np.array(ptsR, np.int32)], False, (255, 0, 0), 3, cv2.LINE_AA)


def draw_main_road_overlay(vis_bgr: np.ndarray, st) -> None:
    """
    Тёмно-зелёная "главная дорога" как трапеция.
    Эффект: затемняем область полигона (видно всегда), слегка добавляя зелёный.
    """
    if vis_bgr is None:
        return
    if not _rails_active_for_state(st):
        return

    ml = getattr(st, "_main_line", None)
    if not ml:
        return

    Lb = getattr(st, "_rail_mask_L_full", None)
    Rb = getattr(st, "_rail_mask_R_full", None)
    if not Lb or not Rb:
        return

    h, w = vis_bgr.shape[:2]
    crop_y0 = int(getattr(st, "_seg_crop_y0", 0) or 0)

    y_scan_crop = int(ml.get("y_scan_crop", 0))
    y_top = int(_clamp(crop_y0 + y_scan_crop, 0, h - 1))

    xL_top = int(_clamp(int(ml.get("xL_crop", 0)), 0, w - 1))
    xR_top = int(_clamp(int(ml.get("xR_crop", w - 1)), 0, w - 1))
    if xR_top < xL_top:
        xL_top, xR_top = xR_top, xL_top

    xL_bot, _yL_bot = Lb
    xR_bot, _yR_bot = Rb
    xL_bot = int(_clamp(int(xL_bot), 0, w - 1))
    xR_bot = int(_clamp(int(xR_bot), 0, w - 1))
    if xR_bot < xL_bot:
        xL_bot, xR_bot = xR_bot, xL_bot

    rail_y_start = float(getattr(st, "rail_y_start", 0.98) or 0.98)
    rail_y_start = float(_clamp(rail_y_start, 0.70, 1.0))
    y_bot = int(_clamp(int(h * rail_y_start), 0, h - 1))
    y_bot = int(max(y_top + 1, y_bot))
    if y_top >= y_bot:
        return

    inset = float(getattr(st, "rail_inset_px", 35.0) or 35.0)
    inset = float(_clamp(inset, 0.0, 200.0))

    xL_top = int(_clamp(xL_top, inset, w - 1 - inset))
    xR_top = int(_clamp(xR_top, inset, w - 1 - inset))
    xL_bot = int(_clamp(xL_bot, inset, w - 1 - inset))
    xR_bot = int(_clamp(xR_bot, inset, w - 1 - inset))

    if (xR_top - xL_top) < 4 or (xR_bot - xL_bot) < 4:
        return

    poly = np.array(
        [[xL_top, y_top], [xR_top, y_top], [xR_bot, y_bot], [xL_bot, y_bot]],
        dtype=np.int32,
    )

    # затемнение
    dark_k = float(getattr(st, "main_road_dark_k", 0.55) or 0.55)  # 0..1 (меньше -> темнее)
    dark_k = float(_clamp(dark_k, 0.15, 0.95))

    green_add = int(getattr(st, "main_road_green_add", 18) or 18)
    green_add = int(_clamp(green_add, 0, 80))

    mask_poly = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask_poly, [poly], 255)

    m = (mask_poly > 0)
    roi = vis_bgr[m].astype(np.float32)
    roi *= dark_k
    roi[:, 1] = np.clip(roi[:, 1] + green_add, 0, 255)
    vis_bgr[m] = roi.astype(np.uint8)