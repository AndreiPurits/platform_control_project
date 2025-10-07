# routing.py
# -*- coding: utf-8 -*-
"""
== ROUTING (новый, только graph) ============================================
Ожидаемый формат артефактов рядом с PNG:
  <base>_graph.json           : {"nodes":{"px":[[x,y],...], "m":[[X,Y],...]}, "edges":[{"u":int,"v":int,"poly_px":[...],"poly_m":[...]}, ...]}
  <base>_points_pixels.json   : [[x,y], ...] — плотные точки для снэпа кликов
  <base>_points_meters.json   : {"meters_per_pixel": float} (опционально)

Основные функции:
- load_graph_and_points_for(png_path, state) -> bool
- set_robot_pose_px(click_px, state) -> (x,y)     # снэп к ближайшей точке из *_points_pixels.json
- set_goal_px(click_px, state)  -> (x,y)
- build_route_from_robot_to_goal(state) -> bool   # Дейкстра с «виртуальными» узлами как в interactive_route.py
- redraw_all(state, idle_view, drive_view) -> None  # перерисовать маркеры + маршрут (для Main.to_idle()/to_drive())
- route_caption_text(state) -> str                # Текст «Маршрут/Пройдено/Осталось»
=========================================================================== 
"""

from typing import List, Tuple, Optional, Dict, Any
import os, json, math, heapq

Point = Tuple[float, float]

EDGE_POLY_KEY_PX = "poly_px"
EDGE_POLY_KEY_M  = "poly_m"

# ---------------- utils ----------------
def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def _subpoly_idx(poly: List[Point], i0: int, i1: int) -> List[Point]:
    if i0 <= i1:
        return poly[i0:i1+1]
    seg = poly[i1:i0+1]
    seg.reverse()
    return seg

def _edge_polyline_px(e: Dict[str, Any]) -> List[Point]:
    poly = e.get(EDGE_POLY_KEY_PX)
    if isinstance(poly, list) and (not poly or isinstance(poly[0], (list, tuple))):
        try:
            return [(float(p[0]), float(p[1])) for p in poly]
        except Exception:
            return []
    return []

def _edge_polyline_m(e: Dict[str, Any]) -> List[Point]:
    poly = e.get(EDGE_POLY_KEY_M)
    if isinstance(poly, list) and (not poly or isinstance(poly[0], (list, tuple))):
        try:
            return [(float(p[0]), float(p[1])) for p in poly]
        except Exception:
            return []
    return []

# ---------------- загрузка артефактов ----------------
def load_graph_and_points_for(png_path: str, state) -> bool:
    base, _ = os.path.splitext(png_path)
    graph_json  = f"{base}_graph.json"
    points_json = f"{base}_points_pixels.json"
    meters_json = f"{base}_points_meters.json"

    print("[ROUTING] ---- load_graph_and_points_for ----")
    print("[ROUTING] png:  ", png_path)
    print("[ROUTING] try graph: ", graph_json,  "  exists=", os.path.isfile(graph_json))
    print("[ROUTING] try points:", points_json, "  exists=", os.path.isfile(points_json))
    print("[ROUTING] try meters:", meters_json, "  exists=", os.path.isfile(meters_json))

    if not (os.path.isfile(graph_json) and os.path.isfile(points_json)):
        print("[ROUTING] graph/points missing")
        return False

    try:
        with open(graph_json, "r", encoding="utf-8") as f:
            graph = json.load(f)

        with open(points_json, "r", encoding="utf-8") as f:
            pts_raw = json.load(f)
        if isinstance(pts_raw, dict) and "points" in pts_raw:
            pts_raw = pts_raw["points"]
        if not (isinstance(pts_raw, list) and (not pts_raw or isinstance(pts_raw[0], (list, tuple)))):
            print("[ROUTING] bad points format:", type(pts_raw).__name__)
            pts = []
        else:
            pts = [(float(x), float(y)) for x, y in pts_raw]

        mpp = None
        if os.path.isfile(meters_json):
            try:
                with open(meters_json, "r", encoding="utf-8") as f:
                    pm = json.load(f)
                mpp = pm.get("meters_per_pixel")
            except Exception:
                pass

        state.graph = graph
        state.points_px = pts
        state.meters_per_pixel = float(mpp) if mpp else None

        print(f"[ROUTING] OK: graph={os.path.basename(graph_json)}, points={os.path.basename(points_json)}, mpp={state.meters_per_pixel}")
        return True
    except Exception as e:
        print("[ROUTING] load error:", e)
        return False

# ---------------- снэп позиций ----------------
def _nearest_from_list(target: Point, cloud: List[Point]) -> Point:
    tx, ty = target
    best, best_d2 = None, float("inf")
    for (x,y) in cloud:
        d2=(x-tx)*(x-tx)+(y-ty)*(y-ty)
        if d2<best_d2:
            best_d2=d2; best=(x,y)
    return best if best else target

def set_robot_pose_px(click_px: Point, state) -> Point:
    pts = getattr(state, "points_px", None) or []
    snapped = _nearest_from_list(click_px, pts) if pts else (float(click_px[0]), float(click_px[1]))
    state.robot_px = (float(snapped[0]), float(snapped[1]))
    print("[ROUTING] robot_px =", state.robot_px)
    return state.robot_px

def set_goal_px(click_px: Point, state) -> Point:
    pts = getattr(state, "points_px", None) or []
    snapped = _nearest_from_list(click_px, pts) if pts else (float(click_px[0]), float(click_px[1]))
    state.goal_px = (float(snapped[0]), float(snapped[1]))
    print("[ROUTING] goal_px  =", state.goal_px)
    return state.goal_px

# ---------------- строитель маршрута (как в interactive_route.py) ----------------
def build_route_from_robot_to_goal(state) -> bool:
    start = getattr(state, "robot_px", None)
    goal  = getattr(state, "goal_px",  None)
    graph = getattr(state, "graph", None)
    if not start or not goal or not graph:
        print("[ROUTING] missing start/goal/graph")
        return False

    nodes_px = graph.get("nodes", {}).get("px") or []
    edges    = graph.get("edges", []) or []
    if not nodes_px or not edges:
        print("[ROUTING] empty graph")
        return False

    # смежность реального графа (между узлами u,v; вес = длина poly_px)
    n = len(nodes_px)
    adj = [[] for _ in range(n)]  # (v, edge_index, weight_px)
    for ei, e in enumerate(edges):
        u = e.get("u"); v = e.get("v")
        poly_px = _edge_polyline_px(e)
        if u is None or v is None or len(poly_px) < 2:
            continue
        w = sum(_dist(a,b) for a,b in zip(poly_px, poly_px[1:]))
        adj[u].append((v, ei, w))
        adj[v].append((u, ei, w))

    # индекс всех вершин всех рёбер → (edge_id, local_idx, xy)
    all_vertices: List[Tuple[int,int,Point]] = []
    for ei, e in enumerate(edges):
        poly_px = _edge_polyline_px(e)
        for li, (x,y) in enumerate(poly_px):
            all_vertices.append((ei, li, (x,y)))

    def nearest_edge_vertex(pt: Point):
        tx, ty = pt
        best = (float("inf"), -1, -1, (tx,ty))
        for ei, li, (x,y) in all_vertices:
            d2 = (x-tx)*(x-tx)+(y-ty)*(y-ty)
            if d2 < best[0]:
                best = (d2, ei, li, (x,y))
        return best  # (d2, ei, li, q)

    # 1) ближайшие вершины рёбер к start/goal
    _, ei_s, li_s, q_s = nearest_edge_vertex(start)
    _, ei_g, li_g, q_g = nearest_edge_vertex(goal)
    if ei_s < 0 or ei_g < 0:
        print("[ROUTING] no edge near start/goal")
        return False

    es = edges[ei_s]; eg = edges[ei_g]
    polyS = _edge_polyline_px(es)
    polyG = _edge_polyline_px(eg)
    if not polyS or not polyG:
        print("[ROUTING] bad edge polylines")
        return False

    # быстрый кейс: оба клика в одном ребре
    if ei_s == ei_g:
        seg_px = _subpoly_idx(polyS, li_s, li_g)
        seg_m  = _subpoly_idx(_edge_polyline_m(es), li_s, li_g)
        state.route_pts_px = seg_px
        if seg_m:
            state.route_len_m = sum(_dist(a,b) for a,b in zip(seg_m, seg_m[1:]))
        else:
            mpp = float(getattr(state, "meters_per_pixel", 0.0) or 0.0)
            state.route_len_m = sum(_dist(a,b) for a,b in zip(seg_px, seg_px[1:])) * mpp
        return True

    # 2) строим расширенный граф с 2 «виртуальными» узлами
    nodes_ext = list(nodes_px)
    idx_start = len(nodes_ext); nodes_ext.append([q_s[0], q_s[1]])
    idx_goal  = len(nodes_ext); nodes_ext.append([q_g[0], q_g[1]])
    adj_ext = [list(row) for row in adj]  # копия
    adj_ext.extend([[], []])

    def connect_virtual(idx_vrt: int, e_idx: int, l_idx: int):
        e = edges[e_idx]
        poly_px = _edge_polyline_px(e)
        # аккумулированные длины
        acc = [0.0]
        for i in range(len(poly_px)-1):
            acc.append(acc[-1] + _dist(poly_px[i], poly_px[i+1]))
        segL = acc[-1] if acc else 0.0
        s_here = acc[l_idx] if l_idx < len(acc) else 0.0
        u = e.get("u"); v = e.get("v")
        if u is not None:
            w = s_here
            adj_ext[idx_vrt].append((u, -1, w))
            adj_ext[u].append((idx_vrt, -1, w))
        if v is not None:
            w = max(segL - s_here, 0.0)
            adj_ext[idx_vrt].append((v, -1, w))
            adj_ext[v].append((idx_vrt, -1, w))

    connect_virtual(idx_start, ei_s, li_s)
    connect_virtual(idx_goal,  ei_g, li_g)

    # если старт/финиш на изолированных циклах (u=v=None) -> пути нет
    if (es.get("u") is None and es.get("v") is None) or (eg.get("u") is None and eg.get("v") is None):
        print("[ROUTING] start/goal on isolated loop")
        return False

    # 3) Дейкстра
    N = len(adj_ext)
    D = [float("inf")]*N
    P: List[Optional[Tuple[int,int]]] = [None]*N   # (prev_node, edge_index_meta) ; edge_index_meta==-1 для виртуальных связей
    D[idx_start] = 0.0
    pq = [(0.0, idx_start)]
    while pq:
        d,u = heapq.heappop(pq)
        if d != D[u]: continue
        if u == idx_goal: break
        for v, ei_meta, w in adj_ext[u]:
            nd = d + w
            if nd < D[v]:
                D[v] = nd
                P[v] = (u, ei_meta)
                heapq.heappush(pq, (nd, v))
    if D[idx_goal] == float("inf"):
        print("[ROUTING] path not found")
        return False

    # 4) восстановим маршрут (точь-в-точь как в interactive_route.py)
    # шаги (prev, cur, ei_meta)
    steps = []
    cur = idx_goal
    while cur != idx_start:
        prev, ei_meta = P[cur]
        steps.append((prev, cur, ei_meta))
        cur = prev
    steps.reverse()

    # быстрое сопоставление ребра по паре узлов
    edge_by_pair: Dict[Tuple[int,int], int] = {}
    for ei, e in enumerate(edges):
        u, v = e.get("u"), e.get("v")
        if u is not None and v is not None:
            edge_by_pair[(u, v)] = ei
            edge_by_pair[(v, u)] = ei

    def edge_between(u, v) -> int:
        return edge_by_pair.get((u, v), -1)

    pts_px: List[Point] = []
    length_px = 0.0
    length_m  = 0.0

    def add_edge_segment(edge_idx: int, i0: int, i1: int):
        nonlocal pts_px, length_px, length_m
        e = edges[edge_idx]
        seg_px = _subpoly_idx(_edge_polyline_px(e), i0, i1)
        seg_m  = _subpoly_idx(_edge_polyline_m(e),  i0, i1) if _edge_polyline_m(e) else []
        if pts_px and seg_px:
            if pts_px[-1] == seg_px[0]:
                pts_px.extend(seg_px[1:])
            else:
                pts_px.extend(seg_px)
        else:
            pts_px.extend(seg_px)
        length_px += sum(_dist(a,b) for a,b in zip(seg_px, seg_px[1:]))
        if seg_m:
            length_m  += sum(_dist(a,b) for a,b in zip(seg_m, seg_m[1:]))

    # нужен также индекс вершин внутри стартового/финишного ребра
    for (a, b, ei_meta) in steps:
        if ei_meta == -1:
            # виртуальная связь: добавляем половину своего ребра
            if a == idx_start or b == idx_start:
                e = edges[ei_s]
                if a == idx_start:
                    if e.get("u") == (b if b < len(nodes_px) else -1):
                        add_edge_segment(ei_s, li_s, 0)
                    elif e.get("v") == (b if b < len(nodes_px) else -1):
                        add_edge_segment(ei_s, li_s, len(_edge_polyline_px(e)) - 1)
                else:
                    if e.get("u") == (a if a < len(nodes_px) else -1):
                        add_edge_segment(ei_s, 0, li_s)
                    elif e.get("v") == (a if a < len(nodes_px) else -1):
                        add_edge_segment(ei_s, len(_edge_polyline_px(e)) - 1, li_s)

            elif a == idx_goal or b == idx_goal:
                e = edges[ei_g]
                if a == idx_goal:
                    if e.get("u") == (b if b < len(nodes_px) else -1):
                        add_edge_segment(ei_g, li_g, 0)
                    elif e.get("v") == (b if b < len(nodes_px) else -1):
                        add_edge_segment(ei_g, li_g, len(_edge_polyline_px(e)) - 1)
                else:
                    if e.get("u") == (a if a < len(nodes_px) else -1):
                        add_edge_segment(ei_g, 0, li_g)
                    elif e.get("v") == (a if a < len(nodes_px) else -1):
                        add_edge_segment(ei_g, len(_edge_polyline_px(e)) - 1, li_g)
            else:
                # виртуал между реальными узлами — геометрию добавит следующий шаг
                pass
        else:
            # нормальное ребро между двумя реальными узлами
            ei = ei_meta if ei_meta >= 0 else edge_between(a, b)
            if ei >= 0:
                e = edges[ei]
                if e.get("u") == a and e.get("v") == b:
                    add_edge_segment(ei, 0, len(_edge_polyline_px(e)) - 1)
                elif e.get("u") == b and e.get("v") == a:
                    add_edge_segment(ei, len(_edge_polyline_px(e)) - 1, 0)

    state.route_pts_px = pts_px
    if length_m > 0.0:
        state.route_len_m = float(length_m)
    else:
        mpp = float(getattr(state, "meters_per_pixel", 0.0) or 0.0)
        state.route_len_m = sum(_dist(a,b) for a,b in zip(pts_px, pts_px[1:])) * mpp
    return True

# ---------------- совместимость и помощники ----------------
def build_route_snap_pixels(start_px: Point, goal_px: Point, state) -> bool:
    set_robot_pose_px(start_px, state)
    set_goal_px(goal_px, state)
    return build_route_from_robot_to_goal(state)

def route_caption_text(state) -> str:
    L = float(getattr(state, "route_len_m", 0.0) or 0.0)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)
    left = max(0.0, L - done)
    return f"Маршрут: {L:.1f} м | Пройдено: {done:.1f} м | Осталось: {left:.1f} м"

# для старых вызовов
def update_progress_text_for_robot(*args, **kwargs) -> str:
    state = args[-1] if args else kwargs.get("state")
    return route_caption_text(state)

# Перерисовать маркеры и маршрут (используйте в Main.to_idle()/to_drive())
def refresh_all_overlays(state, idle_view, drive_view):
    from graphics import redraw_markers, redraw_route
    for view in (idle_view, drive_view):
        if view and view.scene():
            redraw_markers(state, view.scene())
    # маршрут в обеих сценах
    if hasattr(state, "route_pts_px") and state.route_pts_px:
        redraw_route(state, None)