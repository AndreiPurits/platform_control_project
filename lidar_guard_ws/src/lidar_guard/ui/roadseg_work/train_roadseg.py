
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent

# добавляем roadseg_work в sys.path
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))

try:
    from roadseg import RoadSeg
except Exception as e:
    print("[SEG] cannot import RoadSeg:", e)
    sys.exit(1)


def main():
    if len(sys.argv) < 2:
        print("Usage: python eval_roadseg_masks.py <images_dir_rel_or_abs>")
        print("Пример: python eval_roadseg_masks.py datasets/photos/Poly_asf")
        sys.exit(1)

    # Папка с картинками (можно относительный путь от текущей директории)
    in_dir = Path(sys.argv[1]).expanduser().resolve()
    if not in_dir.is_dir():
        print(f"[ERR] input dir not found: {in_dir}")
        sys.exit(1)

    # Папка с масками: <имя_папки>_masks рядом
    out_dir = in_dir.parent / f"{in_dir.name}_masks"
    out_dir.mkdir(parents=True, exist_ok=True)

    onnx_path = HERE / "roadseg_asphalt.onnx"
    print(f"[SEG] loading ONNX: {onnx_path}")

    seg = RoadSeg(
        onnx_path=str(onnx_path),
        input_size=(512, 512),
    )

    if not getattr(seg, "ok", True):
        print("[SEG] WARNING: RoadSeg reports ok=False, возможен stub-режим")

    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for pat in exts:
        files.extend(sorted(glob.glob(str(in_dir / pat))))

    if not files:
        print(f"[WARN] no images found in {in_dir}")
        sys.exit(0)

    print(f"[INFO] found {len(files)} images in {in_dir}")
    print(f"[INFO] masks will be saved to {out_dir}")

    for path in files:
        path = Path(path)
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] cannot read {path.name}")
            continue

        try:
            mask = seg.infer(img)
        except Exception as e:
            print(f"[ERR] seg.infer error on {path.name}: {e}")
            continue

        if mask is None or mask.size == 0:
            print(f"[WARN] empty mask for {path.name}")
            continue

        # mask (H,W) с float 0..1 → uint8 0..255
        mask_u8 = (np.clip(mask, 0.0, 1.0) * 255.0).astype("uint8")

        # сохраняем как PNG-грэйскейл
        out_name = out_dir / f"{path.stem}_mask.png"
        cv2.imwrite(str(out_name), mask_u8)

        print(f"[MASK] {path.name} -> {out_name.name}")

    print("[DONE] all images processed")


if __name__ == "__main__":
    main()