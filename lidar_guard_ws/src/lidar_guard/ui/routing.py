# routing.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtCore

def load_map_spline_for(map_path: str, state) -> None:
    """Читает JSON с тем же basename, кладёт polyline в state.spline_polyline.
       Пытается «принормировать» координаты (если были не в пикселях) — базово.
    """
    import os, json
    base, _ = os.path.splitext(map_path)
    cand = base + ".json"
    state.spline_polyline = None
    if not os.path.isfile(cand):
        print("[ROUTE] рядом .json не найден")
        return
    try:
        with open(cand, "r", encoding="utf-8") as f:
            data = json.load(f)
        poly = data.get("polyline")
        if isinstance(poly, list) and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in poly):
            pts = [(float(px), float(py)) for (px, py) in poly]
            # проверка, не в «метрах» ли (грубая эвристика)
            if max(abs(x) for x, _ in pts[:50]) < 20 and max(abs(y) for _, y in pts[:50]) < 20:
                print("[ROUTE] polyline в другой системе координат → нормализуем в пиксели изображения")
                # без знания масштаба сложно — оставим как есть, но сообщим
            state.spline_polyline = pts
            # Сбросим индексы, если меняется карта
            state.robot_idx = None
            state.goal_idx = None
            print(f"[ROUTE] загружен сплайн: {len(pts)} точек; карта {map_info(map_path)}")
        else:
            print("[ROUTE] неверный формат polyline в JSON")
    except Exception as e:
        print(f"[ROUTE] ошибка чтения JSON: {e}")

def map_info(path: str) -> str:
    from PyQt5 import QtGui
    pm = QtGui.QPixmap(path)
    if pm.isNull(): return "—"
    return f"{pm.width()}×{pm.height()}px"

def nearest_index(poly: List[Tuple[float,float]], pt: QtCore.QPointF) -> int:
    best_i, best_d2 = 0, float("inf")
    for i, (x,y) in enumerate(poly):
        dx = x - pt.x(); dy = y - pt.y()
        d2 = dx*dx + dy*dy
        if d2 < best_d2:
            best_d2, best_i = d2, i
    return best_i

def shortest_subpath(poly: List[Tuple[float,float]], ia: int, ib: int) -> List[Tuple[float,float]]:
    """Замкнутый сплайн: вернёт кратчайшую дугу между индексами ia и ib."""
    if ia == ib: return [poly[ia]]
    n = len(poly)
    if ia < ib:
        forward  = poly[ia:ib+1]
        backward = poly[ib:] + poly[:ia+1]
    else:
        forward  = poly[ia:] + poly[:ib+1]
        backward = poly[ib:ia+1]
    return forward if len(forward) <= len(backward) else backward