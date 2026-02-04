#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF -> PNG + (width,height) по bbox геометрии.

- Рендерит геометрию modelspace в PNG (белый фон, чёрные линии).
- Считает bbox по всем сущностям и печатает ширину/высоту.
- По желанию переводит в метры через коэффициент --unit-scale (м на 1 единицу DXF).

Зависимости:
 pip install ezdxf matplotlib

Пример:
 python dxf_to_png_and_size.py map.dxf --out map.png --dpi 300 --unit-scale 0.001
   (если в DXF единицы мм -> метры)

 python dxf_to_png_and_size.py map.dxf --out map.png --unit-scale 1.0
   (если в DXF единицы уже метры)
"""

import argparse
import json
import os
from typing import Tuple

import ezdxf
from ezdxf import bbox as ezbbox

# matplotlib нужен именно для рендера
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend


def compute_extents(doc) -> Tuple[float, float, float, float]:
   """
   Возвращает bbox (minx, miny, maxx, maxy) по modelspace.
   """
   msp = doc.modelspace()

   # ezdxf.bbox.extents учитывает большинство типов геометрии.
   # Для очень экзотических сущностей может быть не идеально, но для линий/полилиний/сплайнов обычно ок.
   ext = ezbbox.extents(msp, fast=False)
   if ext is None:
       raise RuntimeError("Не удалось вычислить bbox: modelspace пустой или нет геометрии.")

   # ext = BoundingBox
   (minx, miny, _), (maxx, maxy, _) = ext.extmin, ext.extmax
   return float(minx), float(miny), float(maxx), float(maxy)


def render_to_png(doc, out_png: str, dpi: int = 300, margin: float = 0.02):
   """
   Рисует modelspace в PNG через ezdxf drawing + matplotlib.
   """
   ctx = RenderContext(doc)

   # Настройка цветов: белый фон, чёрные линии.
   # В ezdxf можно управлять политикой отображения, но самый надёжный путь —
   # после отрисовки принудительно поставить цвета линий в чёрный через matplotlib.
   fig = plt.figure()
   ax = fig.add_axes([0, 0, 1, 1])  # без рамок
   ax.set_aspect("equal")
   ax.set_axis_off()
   fig.patch.set_facecolor("white")
   ax.set_facecolor("white")

   backend = MatplotlibBackend(ax)
   Frontend(ctx, backend).draw_layout(doc.modelspace(), finalize=True)

   # Принудительно чёрный цвет для всех объектов (если DXF цвета разные)
   for artist in ax.get_children():
       try:
           if hasattr(artist, "set_color"):
               artist.set_color("black")
       except Exception:
           pass

   # Поджимаем по bbox
   minx, miny, maxx, maxy = compute_extents(doc)
   w = maxx - minx
   h = maxy - miny
   pad_x = w * margin
   pad_y = h * margin
   ax.set_xlim(minx - pad_x, maxx + pad_x)
   ax.set_ylim(miny - pad_y, maxy + pad_y)

   # Сохраняем
   os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
   fig.savefig(out_png, dpi=dpi, facecolor="white", bbox_inches="tight", pad_inches=0.0)
   plt.close(fig)


def main():
   ap = argparse.ArgumentParser()
   ap.add_argument("dxf", help="Входной .dxf")
   ap.add_argument("--out", default=None, help="PNG файл (по умолчанию <base>.png)")
   ap.add_argument("--dpi", type=int, default=300, help="DPI для PNG")
   ap.add_argument("--unit-scale", type=float, default=1.0,
                   help="Сколько МЕТРОВ в 1 единице DXF. Примеры: мм->м: 0.001, см->м:0.01, м->м:1.0")
   ap.add_argument("--json", default=None, help="Куда сохранить meta JSON (по умолчанию <base>_meta.json)")
   args = ap.parse_args()

   dxf_path = args.dxf
   if not os.path.isfile(dxf_path):
       raise SystemExit(f"DXF не найден: {dxf_path}")

   base = os.path.splitext(os.path.basename(dxf_path))[0]
   out_png = args.out or f"{base}.png"
   out_json = args.json or f"{base}_meta.json"

   doc = ezdxf.readfile(dxf_path)

   # bbox
   minx, miny, maxx, maxy = compute_extents(doc)
   width_units = maxx - minx
   height_units = maxy - miny

   width_m = width_units * args.unit_scale
   height_m = height_units * args.unit_scale

   # render png
   render_to_png(doc, out_png, dpi=args.dpi)

   meta = {
       "dxf": os.path.abspath(dxf_path),
       "bbox_units": {"minx": minx, "miny": miny, "maxx": maxx, "maxy": maxy},
       "size_units": {"width": width_units, "height": height_units},
       "unit_scale_m_per_unit": args.unit_scale,
       "size_meters": {"width": width_m, "height": height_m},
       "png": os.path.abspath(out_png),
       "dpi": int(args.dpi),
   }

   with open(out_json, "w", encoding="utf-8") as f:
       json.dump(meta, f, ensure_ascii=False, indent=2)

   print("=== DXF EXTENTS ===")
   print(f"bbox units: min=({minx:.6f},{miny:.6f}) max=({maxx:.6f},{maxy:.6f})")
   print(f"size units: width={width_units:.6f}  height={height_units:.6f}")
   print(f"unit_scale: {args.unit_scale} m/unit")
   print(f"size meters: width={width_m:.6f} m  height={height_m:.6f} m")
   print(f"PNG saved: {out_png}")
   print(f"META saved: {out_json}")


if __name__ == "__main__":
   main()