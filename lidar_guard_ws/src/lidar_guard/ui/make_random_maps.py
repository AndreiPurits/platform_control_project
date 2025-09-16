# make_random_maps.py
# Генерация N случайных карт (PNG) + JSON с polyline (замкнутый Catmull–Rom сплайн)
import os, math, json, random, argparse
from typing import List, Tuple
from PIL import Image, ImageDraw, ImageFont

# ---------------------- геометрия ----------------------
def catmull_rom_spline(points: List[Tuple[float, float]], samples_per_seg=24, closed=True):
    """Вернёт сглаженную полилинию по опорным точкам (Catmull–Rom).
    points: список (x,y). Если closed=True — кривая замкнута.
    """
    if len(points) < 3:
        return points[:]
    pts = points[:]
    if closed:
        pts = [pts[-1]] + pts + [pts[0], pts[1]]
    else:
        pts = [pts[0]] + pts + [pts[-1]]

    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i-1], pts[i], pts[i+1], pts[i+2]
        for j in range(samples_per_seg):
            t = j / samples_per_seg
            t2 = t * t
            t3 = t2 * t
            # Catmull-Rom basis
            x = 0.5 * ((2*p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            y = 0.5 * ((2*p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            out.append((x, y))
    if closed:
        out.append(out[0])
    return out

def ring_random_points(cx, cy, r, jitter=0.2, knots=10):
    """Сгенерировать опорные точки по кругу с радиальным разбросом."""
    angles = sorted([random.random()*2*math.pi for _ in range(knots)])
    pts = []
    for a in angles:
        rr = r * (1.0 + (random.random()*2 - 1)*jitter)  # r * (1 ± jitter)
        x = cx + rr * math.cos(a)
        y = cy + rr * math.sin(a)
        pts.append((x, y))
    return pts

# ---------------------- рисование ----------------------
def draw_grid(draw: ImageDraw.ImageDraw, W, H, step=100):
    g = (230, 230, 230)
    for x in range(0, W, step):
        draw.line((x, 0, x, H), fill=g, width=1)
    for y in range(0, H, step):
        draw.line((0, y, W, y), fill=g, width=1)
    draw.rectangle((0, 0, W-1, H-1), outline=(180, 180, 180), width=2)

def draw_arrow_along_poly(draw: ImageDraw.ImageDraw, poly, every=120, length=18, color=(120,120,120)):
    """Маленькие стрелки направления вдоль полилинии через каждые ~every пикселей."""
    acc = 0.0
    for i in range(1, len(poly)):
        x0, y0 = poly[i-1]
        x1, y1 = poly[i]
        seg = math.hypot(x1-x0, y1-y0)
        acc += seg
        if acc >= every:
            acc = 0.0
            # стрелка по направлению сегмента
            ang = math.atan2(y1-y0, x1-x0)
            dx = math.cos(ang)
            dy = math.sin(ang)
            # перо стрелки
            p = (x1, y1)
            l = length
            w = length * 0.5
            left = (x1 - l*dx + w*dy, y1 - l*dy - w*dx)
            right= (x1 - l*dx - w*dy, y1 - l*dy + w*dx)
            draw.line([left, p, right], fill=color, width=3)

def stroke_path(draw: ImageDraw.ImageDraw, pts, width, color):
    """Обводка полилинии толстой линией (дорога)."""
    if not pts: return
    draw.line(pts, fill=color, width=width, joint="curve")

# ---------------------- генерация карты ----------------------
def make_one_map(out_dir, idx, W=1200, H=800, knots=10, jitter=0.20, road_w=44, bg=(250,250,250)):
    os.makedirs(out_dir, exist_ok=True)
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)

    # сетка и рамка
    draw_grid(d, W, H, step=100)

    # опорные точки в кольце
    cx, cy = W//2, H//2
    r = min(W, H)*0.35
    ctrl = ring_random_points(cx, cy, r, jitter=jitter, knots=knots)

    # сглаженная кривая
    poly = catmull_rom_spline(ctrl, samples_per_seg=24, closed=True)

    # дорога: широкая серая линия + лёгкая тень по краям (имитация)
    stroke_path(d, poly, width=road_w+6, color=(205,205,205))
    stroke_path(d, poly, width=road_w,   color=(220,220,220))

    # стрелки направления
    draw_arrow_along_poly(d, poly, every=140, length=16, color=(130,130,130))

    # надписи
    try:
        f = ImageFont.load_default()
        d.text((10,10), f"RAND MAP #{idx:03d}", fill=(0,0,0), font=f)
        d.text((10,28), f"{W}x{H}px; knots={knots}; jitter={jitter:.2f}", fill=(0,0,0), font=f)
    except:
        pass

    # сохранение
    base = f"rand_map_{idx:03d}"
    png_path = os.path.join(out_dir, base + ".png")
    img.save(png_path, "PNG")

    # JSON (polyline)
    json_path = os.path.join(out_dir, base + ".json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({"polyline": poly}, f, ensure_ascii=False, indent=2)

    return png_path, json_path

# ---------------------- CLI ----------------------
def parse_args():
    ap = argparse.ArgumentParser(description="Генерация случайных карт (PNG + JSON polyline)")
    ap.add_argument("--out", type=str, default=os.path.expanduser("~/Downloads/maps_repo"),
                    help="Папка для сохранения (по умолчанию: ~/Downloads/maps_repo)")
    ap.add_argument("--count", type=int, default=3, help="Сколько карт создать")
    ap.add_argument("--w", type=int, default=1200, help="Ширина карты, px")
    ap.add_argument("--h", type=int, default=800,  help="Высота карты, px")
    ap.add_argument("--knots", type=int, default=10, help="Опорные точки сплайна (больше — извилистее)")
    ap.add_argument("--jitter", type=float, default=0.20, help="Радиальный разброс (0..0.5)")
    ap.add_argument("--roadw", type=int, default=44, help="Ширина дороги (px)")
    ap.add_argument("--seed", type=int, default=None, help="Seed для повторяемости")
    return ap.parse_args()

def main():
    args = parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    os.makedirs(args.out, exist_ok=True)
    for i in range(1, args.count+1):
        png_path, json_path = make_one_map(
            out_dir=args.out,
            idx=i,
            W=args.w,
            H=args.h,
            knots=args.knots,
            jitter=max(0.0, min(0.5, args.jitter)),
            road_w=args.roadw,
        )
        print("Created:", png_path)
        print("Created:", json_path)

if __name__ == "__main__":
    main()