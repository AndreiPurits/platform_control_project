# make_test_map.py
# Создаёт ~/maps_repo/test_map.png и ~/maps_repo/test_map.json (сплайн-кольцо)
import os, json, math
from PIL import Image, ImageDraw, ImageFont

out_dir = os.path.expanduser("~/maps_repo")
os.makedirs(out_dir, exist_ok=True)

W, H = 1200, 800
bg = (250, 250, 250)
img = Image.new("RGB", (W, H), bg)
draw = ImageDraw.Draw(img)

# сетка
step = 100
for x in range(0, W, step):
    draw.line((x, 0, x, H), fill=(230, 230, 230), width=1)
for y in range(0, H, step):
    draw.line((0, y, W, y), fill=(230, 230, 230), width=1)

# рамка
draw.rectangle((0, 0, W-1, H-1), outline=(180, 180, 180), width=2)

# круговая "дорога" (два концентрических кольца + стрелки направления)
cx, cy = W//2, H//2
r_outer = min(W, H)//3
r_inner = r_outer - 50

# “асфальт”
draw.ellipse((cx-r_outer, cy-r_outer, cx+r_outer, cy+r_outer), outline=None, fill=(220,220,220))
draw.ellipse((cx-r_inner, cy-r_inner, cx+r_inner, cy+r_inner), outline=None, fill=bg)

# стрелки направления по окружности
for ang in range(0, 360, 30):
    a = math.radians(ang)
    rx = cx + (r_inner + 20) * math.cos(a)
    ry = cy + (r_inner + 20) * math.sin(a)
    rdx = 14 * math.cos(a)
    rdy = 14 * math.sin(a)
    draw.line((rx-rdx, ry-rdy, rx+rdx, ry+rdy), fill=(120,120,120), width=3)

# надписи
try:
    font = ImageFont.load_default()
    draw.text((10, 10), "TEST MAP (pixels)", fill=(0,0,0), font=font)
    draw.text((10, 28), "Grid=100 px; Ring road", fill=(0,0,0), font=font)
except:
    pass

png_path = os.path.join(out_dir, "test_map.png")
img.save(png_path, "PNG")

# Сплайн: полилиния по середине дороги (между inner/outer) — окружность из точек
r_mid = (r_outer + r_inner) / 2.0
polyline = []
for ang in range(0, 360, 2):
    a = math.radians(ang)
    x = cx + r_mid * math.cos(a)
    y = cy + r_mid * math.sin(a)
    polyline.append([x, y])

json_path = os.path.join(out_dir, "test_map.json")
with open(json_path, "w", encoding="utf-8") as f:
    json.dump({"polyline": polyline}, f, ensure_ascii=False, indent=2)

print("Created:", png_path)
print("Created:", json_path)