# -*- coding: utf-8 -*-
"""
eval_rails_offline.py

Офлайн тест: roadseg -> road_bin preprocess -> main road -> rails anchors -> overlays.
Сохраняет картинки (overlay) + лог метрик по каждому кадру.

Требования:
- roadseg.py рядом/доступен (как у тебя в проекте)
- drive_rails.py доступен (compute_main_road_from_roadbin, update_rail_anchors_from_roadbin, draw_*)

Запуск:
 python3 eval_rails_offline.py \
     --input /path/to/images \
     --onnx  /path/to/roadseg_best.onnx \
     --out   /path/to/out_eval

python3 eval_rails_offline.py \
  --input /images \
  --onnx  /ft/roadseg_snow_best.onnx \
  --out   /eval_new
"""

import os
import sys
import argparse
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


# -----------------------------
# Import your project modules
# -----------------------------
def add_to_syspath(p: Path):
   p = str(p.resolve())
   if p not in sys.path:
       sys.path.append(p)

HERE = Path(__file__).resolve().parent
add_to_syspath(HERE / "roadseg_work")  # если roadseg.py в roadseg_work

from roadseg import RoadSeg

from drive_rails import (
   compute_main_road_from_roadbin,
   update_rail_anchors_from_roadbin,
   draw_main_road_overlay,
   draw_rails_overlay,
)


# -----------------------------
# Helpers
# -----------------------------
def clamp(x, lo, hi):
   return lo if x < lo else hi if x > hi else x

def list_images(input_dir: Path):
   exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
   return sorted([p for p in input_dir.iterdir() if p.suffix.lower() in exts])

def ensure_dir(p: Path):
   p.mkdir(parents=True, exist_ok=True)

def put_text(img, lines, org=(10, 22)):
   y = org[1]
   for s in lines:
       cv2.putText(img, s, (org[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
       cv2.putText(img, s, (org[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
       y += 20

# -----------------------------
# Road bin builder (копия твоей логики, но без EMA для офлайн)
# -----------------------------
def build_road_bin_from_mask(
   st,
   mask: np.ndarray,           # HxW float [0..1]
) -> tuple[np.ndarray, int, float, dict]:
   """
   Возвращает:
     road_bin_crop (bool) по crop,
     crop_y0_full,
     thr,
     debug_dict (метрики до/после)
   """
   H, W = mask.shape[:2]
   crop_y0_full = int(H * float(getattr(st, "seg_crop_y0", 0.38) or 0.38))
   crop_y0_full = max(0, min(H - 2, crop_y0_full))
   st._seg_crop_y0 = int(crop_y0_full)

   mask_crop = mask[crop_y0_full:, :]

   thr = float(getattr(st, "seg_thr", 0.60) or 0.60)
   thr = float(clamp(thr, 0.05, 0.95))

   road_bin = (mask_crop > thr)

   dbg = {}
   dbg["crop_y0"] = int(crop_y0_full)
   dbg["thr"] = float(thr)
   dbg["mask_mean_crop"] = float(mask_crop.mean()) if mask_crop.size else 0.0
   dbg["rb_mean_raw"] = float(road_bin.mean()) if road_bin.size else 0.0

   # --- постпроцесс как у тебя ---
   try:
       rb = (road_bin.astype(np.uint8) * 255)

       close_k = int(getattr(st, "rb_close_k", 11) or 11)
       open_k = int(getattr(st, "rb_open_k", 3) or 3)
       close_iter = int(getattr(st, "rb_close_iter", 2) or 2)
       open_iter = int(getattr(st, "rb_open_iter", 1) or 1)

       close_k = int(clamp(close_k, 3, 21))
       open_k = int(clamp(open_k, 1, 11))
       close_iter = int(clamp(close_iter, 1, 5))
       open_iter = int(clamp(open_iter, 0, 3))

       k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
       k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))

       rb = cv2.morphologyEx(rb, cv2.MORPH_CLOSE, k_close, iterations=close_iter)
       if open_k >= 3 and open_iter > 0:
           rb = cv2.morphologyEx(rb, cv2.MORPH_OPEN, k_open, iterations=open_iter)

       # fill holes
       hh, ww = rb.shape[:2]
       ff = rb.copy()
       mask_ff = np.zeros((hh + 2, ww + 2), np.uint8)
       cv2.floodFill(ff, mask_ff, (0, 0), 255)
       ff_inv = cv2.bitwise_not(ff)
       rb = cv2.bitwise_or(rb, ff_inv)

       # keep main component anchored to bottom band
       num, labels, stats, _ = cv2.connectedComponentsWithStats((rb > 0).astype(np.uint8), connectivity=8)
       dbg["cc_num"] = int(num)

       if num > 1:
           Hc = rb.shape[0]
           band_h = max(5, int(Hc * float(getattr(st, "rb_keep_bottom_frac", 0.18) or 0.18)))
           bottom = labels[Hc - band_h:, :]

           best_lbl = 0
           best_score = 0
           for lbl in range(1, num):
               cnt = int(np.sum(bottom == lbl))
               if cnt > best_score:
                   best_score = cnt
                   best_lbl = lbl
           dbg["cc_best_lbl"] = int(best_lbl)
           dbg["cc_best_score"] = int(best_score)

           if best_lbl > 0:
               rb = ((labels == best_lbl).astype(np.uint8) * 255)

       road_bin = (rb > 0)
       dbg["rb_mean_post"] = float(road_bin.mean()) if road_bin.size else 0.0

   except Exception as e:
       dbg["post_err"] = str(e)

   return road_bin, int(crop_y0_full), float(thr), dbg


# -----------------------------
# Render overlay (mask + road_bin + rails + main)
# -----------------------------
def render_overlay(frame_bgr, mask, road_bin, crop_y0_full, st) -> np.ndarray:
   vis = frame_bgr.copy()
   H, W = vis.shape[:2]

   # 1) mask prob heat (зеленый по всему кадру, чтобы видеть "почему не находится")
   if mask is not None and mask.size:
       mm = (mask * 255.0).clip(0, 255).astype(np.uint8)
       color = np.zeros_like(vis)
       color[:, :, 1] = mm
       vis = cv2.addWeighted(vis, 1.0, color, 0.35, 0.0)

   # 2) road_bin (ярче, но только в crop)
   if road_bin is not None and road_bin.size:
       full = np.zeros((H, W), dtype=np.uint8)
       rb = (road_bin.astype(np.uint8) * 255)
       hh = min(H - crop_y0_full, rb.shape[0])
       if hh > 0:
           full[crop_y0_full:crop_y0_full + hh, :W] = rb[:hh, :W]

       color2 = np.zeros_like(vis)
       color2[:, :, 1] = full
       vis = cv2.addWeighted(vis, 1.0, color2, 0.55, 0.0)

   # 3) main + rails
   try:
       draw_main_road_overlay(vis, st)
   except Exception:
       pass

   try:
       draw_rails_overlay(vis, st)
   except Exception:
       pass

   return vis


# -----------------------------
# Main
# -----------------------------
def main():
   ap = argparse.ArgumentParser()
   ap.add_argument("--input", required=True, help="Folder with images")
   ap.add_argument("--onnx", required=True, help="Path to ONNX model")
   ap.add_argument("--out", required=True, help="Output folder for overlays")
   ap.add_argument("--limit", type=int, default=0, help="Process first N images (0=all)")
   args = ap.parse_args()

   input_dir = Path(args.input).resolve()
   out_dir = Path(args.out).resolve()
   ensure_dir(out_dir)

   if not input_dir.exists():
       raise FileNotFoundError(input_dir)
   if not Path(args.onnx).exists():
       raise FileNotFoundError(args.onnx)

   imgs = list_images(input_dir)
   if args.limit and args.limit > 0:
       imgs = imgs[: args.limit]

   print(f"[DATA] images={len(imgs)}  input={input_dir}")
   print(f"[OUT]  {out_dir}")

   # state stub с параметрами, которые ты крутишь в GUI
   st = SimpleNamespace()

   # --- ключевые параметры из твоего VideoController ---
   st.seg_crop_y0 = 0.38
   st.seg_thr = 0.60

   st.rb_close_k = 11
   st.rb_open_k = 3
   st.rb_close_iter = 2
   st.rb_open_iter = 1
   st.rb_keep_bottom_frac = 0.18

   # --- параметры рейлов/главной дороги (то, что ты крутишь) ---
   st.main_y_search_top_frac = 0.03
   st.main_y_search_bot_frac = 0.45
   st.main_min_row_fill = 0.10
   st.main_min_width_px = 120
   st.main_min_seg_px = 24
   st.main_center_gate_px = None  # если None -> берется дефолт из кода (0.14*W)
   st.main_score_w_y = 2.2
   st.main_score_w_center = 1.0
   st.main_score_w_width = 0.25
   st.main_base_deg = 12.0
   st.main_trace_step_px = 8
   st.main_need_frac = 0.80
   st.main_max_expand_frac = 0.60

   st.rail_y_start = 0.98
   st.rail_inset_px = 35.0
   st.rail_step_px = 8

   st.rail_mask_update_s = 0.8
   st.rail_mask_bottom_saturated_thr = 0.98
   st.rail_bottom_window_h = 28
   st.rail_min_width_px = 120
   st.rail_mask_hist_n = 7
   st.rail_anchor_deadband_px = 6
   st.rail_mask_tau = 2.5
   st.rail_anchor_max_jump_px = 35.0

   # main road overlay params
   st.main_road_alpha = 0.45
   st.main_road_dark_k = 0.55
   st.main_road_green_add = 18

   # seg
   seg = RoadSeg(onnx_path=str(Path(args.onnx).resolve()))
   if not getattr(seg, "ok", False):
       print("[ERR] RoadSeg not ok. Check onnxruntime / model path.")
       return

   # log
   log_path = out_dir / "eval_log.csv"
   with open(log_path, "w", encoding="utf-8") as f:
       f.write("name,mask_mean_crop,rb_mean_raw,rb_mean_post,cc_num,railLx,railRx,main_xL,main_xR,main_trace_frac\n")

       for i, p in enumerate(imgs, 1):
           img = cv2.imread(str(p), cv2.IMREAD_COLOR)
           if img is None:
               continue

           mask = seg.infer(img)  # HxW prob
           road_bin, crop_y0, thr, dbg = build_road_bin_from_mask(st, mask)

           # mainline + rails anchors (как в рантайме)
           compute_main_road_from_roadbin(st, road_bin, int(crop_y0))
           update_rail_anchors_from_roadbin(st, road_bin, int(crop_y0))

           # overlay
           vis = render_overlay(img, mask, road_bin, crop_y0, st)

           # debug text
           L = getattr(st, "_rail_mask_L_full", None)
           R = getattr(st, "_rail_mask_R_full", None)
           ml = getattr(st, "_main_line", None)

           railLx = railRx = None
           if L and R:
               railLx = int(L[0]); railRx = int(R[0])

           main_xL = main_xR = None
           if ml:
               main_xL = int(ml.get("xL_crop", 0))
               main_xR = int(ml.get("xR_crop", 0))

           trace_frac = float(getattr(st, "main_trace_frac", 0.0) or 0.0)

           put_text(
               vis,
               [
                   f"{p.name}",
                   f"thr={dbg.get('thr',0):.2f} crop_y0={dbg.get('crop_y0',0)}",
                   f"mask_mean_crop={dbg.get('mask_mean_crop',0):.3f}",
                   f"rb_raw={dbg.get('rb_mean_raw',0):.3f} rb_post={dbg.get('rb_mean_post',0):.3f} cc={dbg.get('cc_num','')}",
                   f"rails: {railLx} .. {railRx}",
                   f"main:  {main_xL} .. {main_xR} trace={trace_frac:.2f}",
               ],
               org=(10, 22),
           )

           out_img = out_dir / f"{i:05d}_{p.stem}.jpg"
           cv2.imwrite(str(out_img), vis)

           f.write(
               f"{p.name},"
               f"{dbg.get('mask_mean_crop',0):.6f},"
               f"{dbg.get('rb_mean_raw',0):.6f},"
               f"{dbg.get('rb_mean_post',0):.6f},"
               f"{dbg.get('cc_num','')},"
               f"{'' if railLx is None else railLx},"
               f"{'' if railRx is None else railRx},"
               f"{'' if main_xL is None else main_xL},"
               f"{'' if main_xR is None else main_xR},"
               f"{trace_frac:.6f}\n"
           )

           if (i % 50) == 0:
               print(f"[OK] {i}/{len(imgs)}")

   print(f"[DONE] overlays -> {out_dir}")
   print(f"[LOG]    {log_path}")


if __name__ == "__main__":
   main()