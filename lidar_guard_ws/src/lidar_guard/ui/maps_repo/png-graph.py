#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PNG -> граф (узлы deg>=3, рёбра как цепочки пикселей), выпуск в 600x450.
- Скелетизация -> поиск развилок (deg>=3) -> трассировка рёбер.
- Все координаты графа масштабируются в 600x450.
- Масштаб в метры пересчитывается под новый размер (анизотропно).
- Сохраняются:
    <base>_600x450.png
    <base>_graph.json
    <base>_points_pixels.json
    <base>_points_meters.json
Зависимости: numpy, pillow, scikit-image
"""

import os, json, math
import numpy as np
from PIL import Image
from skimage import color, filters, morphology

# ====== входные картинки и реальные габариты (метры) ======
IMAGES = {
    "Poly_asf.png": { "width": 499.71006620512344, "height": 565.9339837834705 },
    "Poly_grd.png": { "width": 1000.7753089902862, "height": 637.6776363680535 },
}

# целевой размер выходной карты
TARGET_W = 600
TARGET_H = 450

# 8-соседство
OFFS = [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]


def neighbors(y: int, x: int, H: int, W: int):
    for dy, dx in OFFS:
        ny, nx = y + dy, x + dx
        if 0 <= ny < H and 0 <= nx < W:
            yield ny, nx


def skeletonize_png(path: str) -> np.ndarray:
    """Otsu-порог -> скелетизация (без OpenCV). Возвращает bool-маску скелета."""
    arr = np.array(Image.open(path).convert("RGB"))
    gray = color.rgb2gray(arr)
    t = filters.threshold_otsu(gray)
    bw = gray < t
    skel = morphology.skeletonize(bw)
    return skel


def build_graph(skel: np.ndarray):
    """
    Граф по скелету:
      - узлы = пиксели со степенью >=3 (развилки),
      - рёбра = трассировка от узла к узлу/тупику,
      - отдельные циклы (компоненты, где все deg==2) → edges c u=v=None, closed=True.
    Возвращает: nodes_px: List[[x,y]], edges_raw: List[dict], (W,H)
    edges_raw[i] = { "u": int|None, "v": int|None, "pixels": [(x,y),...], "closed": bool }
    """
    H, W = skel.shape
    pix_ids = -np.ones_like(skel, dtype=int)
    ys, xs = np.where(skel)
    coords = []
    for i, (y, x) in enumerate(zip(ys, xs)):
        pix_ids[y, x] = i
        coords.append((int(x), int(y)))

    if not coords:
        return [], [], (W, H)

    # степень по скелету
    deg = np.zeros(len(coords), dtype=int)
    for i, (x, y) in enumerate(coords):
        c = 0
        for ny, nx in neighbors(y, x, H, W):
            if skel[ny, nx]:
                c += 1
        deg[i] = c

    # базовые узлы: deg >= 3
    node_mask = (deg >= 3)

    # индексация узлов
    node_id = -np.ones(len(coords), dtype=int)
    nodes_px = []
    for i, isnode in enumerate(node_mask):
        if isnode:
            nid = len(nodes_px)
            node_id[i] = nid
            x, y = coords[i]
            nodes_px.append([float(x), float(y)])

    # соседство в индексовом пространстве пикселей
    nbrs = [[] for _ in coords]
    for i, (x, y) in enumerate(coords):
        for ny, nx in neighbors(y, x, H, W):
            j = pix_ids[ny, nx]
            if j >= 0:
                nbrs[i].append(j)

    visited = set()
    edges = []

    def trace(u_pix, v_pix):
        """Идём по пикселям, пока не встретим узел (node_mask) или тупик."""
        path = [u_pix]
        prev, cur = u_pix, v_pix
        while True:
            path.append(cur)
            if node_mask[cur]:
                return path, cur
            nxts = [nb for nb in nbrs[cur] if nb != prev]
            if not nxts:
                return path, None
            prev, cur = cur, nxts[0]

    # рёбра от каждого узла
    for pi in range(len(coords)):
        if node_mask[pi]:
            for nb in nbrs[pi]:
                if (pi, nb) in visited or (nb, pi) in visited:
                    continue
                path, last = trace(pi, nb)
                for a, b in zip(path, path[1:]):
                    visited.add((a, b))
                u = node_id[pi]
                v = node_id[last] if (last is not None and node_mask[last]) else None
                poly = [(float(coords[k][0]), float(coords[k][1])) for k in path]
                edges.append({
                    "u": (int(u) if u is not None else None),
                    "v": (int(v) if v is not None else None),
                    "pixels": poly,
                    "closed": False,
                })

    # отдельные циклы (компоненты без узлов)
    used_pix = np.zeros(len(coords), dtype=bool)
    for e in edges:
        for (x, y) in e["pixels"]:
            pid = pix_ids[int(y), int(x)]
            if pid >= 0:
                used_pix[pid] = True

    for i in range(len(coords)):
        if not used_pix[i] and deg[i] == 2:
            cyc = [i]
            cur = i
            prev = -1
            while True:
                nxts = [nb for nb in nbrs[cur] if nb != prev]
                if not nxts:
                    break
                nxt = nxts[0]
                if nxt == i:
                    break
                cyc.append(nxt)
                prev, cur = cur, nxt
            if len(cyc) >= 2:
                poly = [(float(coords[k][0]), float(coords[k][1])) for k in cyc]
                edges.append({"u": None, "v": None, "pixels": poly, "closed": True})

    return nodes_px, edges, (W, H)


def process(image_name: str, meta: dict):
    """
    Строит граф по исходной PNG, масштабирует координаты в 600x450,
    пересчитывает м/px и сохраняет JSON/PNG.
    """
    # 1) скелет и граф в исходном размере
    skel = skeletonize_png(image_name)
    nodes_px, edges_raw, (W, H) = build_graph(skel)

    # 2) коэффициенты перехода к целевому пиксельному размеру
    sx_px = TARGET_W / max(W, 1)
    sy_px = TARGET_H / max(H, 1)

    # 3) масштаб в метры ДЛЯ НОВОГО размера (м/px для 600x450):
    # шаг пикселя по X/Y — от реальных метров и количества шагов (W-1/H-1) в новом растре
    m_per_px_x = float(meta["width"])  / max(TARGET_W - 1, 1)
    m_per_px_y = float(meta["height"]) / max(TARGET_H - 1, 1)

    # 4) масштабируем узлы и рёбра в ПИКСЕЛЯХ под 600×450
    nodes_px_scaled = [[float(x) * sx_px, float(y) * sy_px] for (x, y) in nodes_px]

    edges_scaled = []
    for e in edges_raw:
        poly_px_scaled = [(float(x) * sx_px, float(y) * sy_px) for (x, y) in e["pixels"]]
        edges_scaled.append({
            "u": e["u"],
            "v": e["v"],
            "closed": bool(e.get("closed", False)),
            "poly_px": poly_px_scaled,  # финальное имя поля
        })

    # 5) длины и финальные поля из уже масштабированных пикселей
    total_len_px = 0.0
    total_len_m  = 0.0
    edges_out = []
    all_points_px = []
    all_points_m  = []

    for e in edges_scaled:
        poly_px = e["poly_px"]
        # посегментно (анизотропно): метры считаем по dx*m_per_px_x, dy*m_per_px_y
        poly_m = [(x * m_per_px_x, y * m_per_px_y) for (x, y) in poly_px]

        L_px = sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(poly_px, poly_px[1:]))
        L_m  = sum(math.hypot((x2 - x1) * m_per_px_x, (y2 - y1) * m_per_px_y)
                   for (x1, y1), (x2, y2) in zip(poly_px, poly_px[1:]))

        total_len_px += L_px
        total_len_m  += L_m

        all_points_px.extend([[float(x), float(y)] for (x, y) in poly_px])
        all_points_m.extend([[float(x), float(y)] for (x, y) in poly_m])

        edges_out.append({
            "u": e["u"],
            "v": e["v"],
            "closed": e["closed"],
            "length_px": float(L_px),
            "length_m":  float(L_m),
            "poly_px": [[float(x), float(y)] for (x, y) in poly_px],
            "poly_m":  [[float(x), float(y)] for (x, y) in poly_m],
        })

    # 6) узлы в метрах — из уже масштабированных пикселей
    nodes_m = [[x * m_per_px_x, y * m_per_px_y] for (x, y) in nodes_px_scaled]

    base = os.path.splitext(image_name)[0]

    # 7) (опционально) сохраним уменьшенную PNG для наглядного соответствия
    try:
        im = Image.open(image_name).convert("RGB")
        im_small = im.resize((TARGET_W, TARGET_H), Image.BILINEAR)
        im_small.save(f"{base}_600x450.png")
    except Exception as e:
        print(f"[warn] cannot save resized PNG: {e}")

    # 8) плоские списки точек
    with open(f"{base}_points_pixels.json", "w", encoding="utf-8") as f:
        json.dump(all_points_px, f, ensure_ascii=False, indent=2)
    with open(f"{base}_points_meters.json", "w", encoding="utf-8") as f:
        json.dump(all_points_m, f, ensure_ascii=False, indent=2)

    # 9) финальный граф JSON — в новой пиксельной системе 600×450
    graph_json = {
        "image_size_px": [int(TARGET_W), int(TARGET_H)],
        "scale": {"m_per_px_x": m_per_px_x, "m_per_px_y": m_per_px_y},
        "nodes": {"px": nodes_px_scaled, "m": nodes_m},
        "edges": edges_out,
        "stats": {
            "junctions": len(nodes_px_scaled),
            "edges": len(edges_out),
            "cycles": sum(1 for e in edges_out if e["closed"]),
            "total_length_px": float(total_len_px),
            "total_length_m":  float(total_len_m),
        }
    }
    with open(f"{base}_graph.json", "w", encoding="utf-8") as f:
        json.dump(graph_json, f, ensure_ascii=False, indent=2)

    # 10) консоль
    print(f"[{image_name}] {W}×{H}px  ->  {TARGET_W}×{TARGET_H}px")
    print(f"  Развилок: {len(nodes_px_scaled)}")
    print(f"  Рёбер: {len(edges_out)} (циклов: {sum(1 for e in edges_out if e['closed'])})")
    print(f"  Длина: {total_len_m:.2f} м  ({total_len_px:.1f} px)")
    print(f"  → {base}_600x450.png, {base}_points_pixels.json, {base}_points_meters.json, {base}_graph.json")


if __name__ == "__main__":
    for img, meta in IMAGES.items():
        if not os.path.exists(img):
            print(f"skip: {img} not found"); continue
        process(img, meta)