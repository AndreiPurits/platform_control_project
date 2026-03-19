# -*- coding: utf-8 -*-
"""
== ROUTING (graph) ==========================================================
Артефакты рядом с PNG:
  <base>_graph.json           : {"nodes":{"px":[[x,y],...], "m":[[X,Y],...]}, "edges":[{"u":int,"v":int,"poly_px":[...],"poly_m":[...]}, ...]}
  <base>_points_pixels.json   : [[x,y], ...] — плотные точки для снэпа кликов

Основные функции:
- load_graph_and_points_for(png_path, state) -> bool
- set_robot_pose_px(click_px, state) -> (x,y)
- set_goal_px(click_px, state) -> (x,y)
- set_control_item_px(click_px, state) -> list[(x,y)]
- build_route_from_robot_to_goal(state) -> bool   # путь собираем по PX, метрику считаем по масштабу
- refresh_all_overlays(state, idle_view, drive_view) -> None
- route_caption_text(state) -> str
===========================================================================
"""

from typing import List, Tuple, Optional, Dict, Any
import os, json, math, heapq

Point = Tuple[float, float]

EDGE_POLY_KEY_PX = "poly_px"
EDGE_POLY_KEY_M  = "poly_m"     # НЕ используем для метрики — путь по PX, метры через масштаб

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

# Евклидова длина отрезка в МЕТРАХ при анизотропном масштабе
def _seg_len_m_from_px(a_px: Point, b_px: Point, mx: float, my: float) -> float:
    dx = (b_px[0] - a_px[0]) * mx
    dy = (b_px[1] - a_px[1]) * my
    return math.hypot(dx, dy)

# ---------------- загрузка артефактов ----------------
def load_graph_and_points_for(png_path: str, state) -> bool:
    base, _ = os.path.splitext(png_path)
    graph_json  = f"{base}_graph.json"
    points_json = f"{base}_points_pixels.json"

    print("[ROUTING] ---- load_graph_and_points_for ----", flush=True)
    print("[ROUTING] png:   ", png_path, flush=True)
    print("[ROUTING] graph: ", graph_json,  "  exists=", os.path.isfile(graph_json), flush=True)
    print("[ROUTING] points:", points_json, "  exists=", os.path.isfile(points_json), flush=True)

    if not (os.path.isfile(graph_json) and os.path.isfile(points_json)):
        print("[ROUTING] graph/points missing", flush=True)
        return False

    try:
        with open(graph_json, "r", encoding="utf-8") as f:
            graph = json.load(f)

        with open(points_json, "r", encoding="utf-8") as f:
            pts_raw = json.load(f)
        if isinstance(pts_raw, dict) and "points" in pts_raw:
            pts_raw = pts_raw["points"]
        if not (isinstance(pts_raw, list) and (not pts_raw or isinstance(pts_raw[0], (list, tuple)))):
            print("[ROUTING] bad points format:", type(pts_raw).__name__, flush=True)
            pts = []
        else:
            pts = [(float(x), float(y)) for x, y in pts_raw]

        # сохранить в состояние
        state.graph = graph
        state.points_px = pts

        # Установить анизотропный масштаб по имени карты
        name = os.path.basename(png_path).lower()
        if "poly_asf" in name:
            state.m_per_px_x = 0.8342405111938622
            state.m_per_px_y = 1.2604320351524956
        elif "poly_grd" in name:
            state.m_per_px_x = 1.670743420684952
            state.m_per_px_y = 1.4202174529355311
        else:
            # Если вдруг другая карта — хотя бы не нули (можно подменить своими)
            state.m_per_px_x = 1.0
            state.m_per_px_y = 1.0

        print(f"[ROUTING] scale: m_per_px_x={state.m_per_px_x}  m_per_px_y={state.m_per_px_y}", flush=True)
        return True
    except Exception as e:
        print("[ROUTING] load error:", e, flush=True)
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
    print("[ROUTING] robot_px =", state.robot_px, flush=True)
    return state.robot_px

def set_goal_px(click_px: Point, state) -> Point:
    pts = getattr(state, "points_px", None) or []
    snapped = _nearest_from_list(click_px, pts) if pts else (float(click_px[0]), float(click_px[1]))
    state.goal_px = (float(snapped[0]), float(snapped[1]))
    print("[ROUTING] goal_px  =", state.goal_px, flush=True)
    return state.goal_px

def set_control_item_px(click_px: Point, state) -> list[Point]:
    pts = getattr(state, "points_px", None) or []
    snapped = _nearest_from_list(click_px, pts) if pts else (float(click_px[0]), float(click_px[1]))
    ctrl_pt = (float(snapped[0]), float(snapped[1]))
    if not hasattr(state, "control_pts_px") or state.control_pts_px is None:
        state.control_pts_px = []
    state.control_pts_px.append(ctrl_pt)
    print(f"[ROUTING] Added control point: {ctrl_pt}", flush=True)
    print(f"[ROUTING] Total control points: {len(state.control_pts_px)}", flush=True)
    return state.control_pts_px

# ---------------- строитель маршрута (по графу; путь по PX) ----------------
def build_route_from_robot_to_goal(state) -> bool:
    start = getattr(state, "robot_px", None)
    goal  = getattr(state, "goal_px",  None)
    graph = getattr(state, "graph", None)
    if not (start and goal and graph):
        print("[ROUTING] missing start/goal/graph", flush=True)
        return False

    nodes_px = graph.get("nodes", {}).get("px") or []
    edges    = graph.get("edges", []) or []
    if not nodes_px or not edges:
        print("[ROUTING] empty graph", flush=True)
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
        print("[ROUTING] no edge near start/goal", flush=True)
        return False

    es = edges[ei_s]; eg = edges[ei_g]
    polyS = _edge_polyline_px(es)
    polyG = _edge_polyline_px(eg)
    if not polyS or not polyG:
        print("[ROUTING] bad edge polylines", flush=True)
        return False

    # быстрый кейс: оба клика в одном ребре
    if ei_s == ei_g:
        seg_px = _subpoly_idx(polyS, li_s, li_g)
        state.route_pts_px = seg_px
        # метрика ВСЕГДА через масштаб
        mx = float(getattr(state, "m_per_px_x", 1.0)); my = float(getattr(state, "m_per_px_y", 1.0))
        state.route_pts_m  = [(x*mx, y*my) for (x,y) in seg_px]
        state.route_len_m  = sum(_dist(a,b) for a,b in zip(state.route_pts_m, state.route_pts_m[1:]))
        # сброс прогресса
        state.route_progress_idx = 0
        state.route_seg_off_m = 0.0
        state.route_done_m = 0.0
        state.route_finished = False
        print(f"[ROUTING] route pts: px={len(state.route_pts_px)}  m={len(state.route_pts_m)}  L={state.route_len_m:.2f}", flush=True)
        compute_controls_on_route(state, eps_px=2.0)
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
        # аккумулированные длины (в пикселях)
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
        print("[ROUTING] start/goal on isolated loop", flush=True)
        return False
    
    # 3) Дейкстра по расширенному графу (вес — пиксельная длина)
    N = len(adj_ext)
    D = [float("inf")]*N
    P: List[Optional[Tuple[int,int]]] = [None]*N   # (prev_node, edge_index_meta)
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
        print("[ROUTING] path not found", flush=True)
        return False

    # 4) восстановим маршрут по шагам (prev, cur, ei_meta)
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

    def add_edge_segment(edge_idx: int, i0: int, i1: int):
        nonlocal pts_px
        e = edges[edge_idx]
        seg_px = _subpoly_idx(_edge_polyline_px(e), i0, i1)
        if pts_px and seg_px:
            if pts_px[-1] == seg_px[0]:
                pts_px.extend(seg_px[1:])
            else:
                pts_px.extend(seg_px)
        else:
            pts_px.extend(seg_px)

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

    # --- МЕТРИКА ВСЕГДА ЧЕРЕЗ МАСШТАБ (анизотропно) ---
    mx = float(getattr(state, "m_per_px_x", 1.0))
    my = float(getattr(state, "m_per_px_y", 1.0))
    state.route_pts_m = [(x*mx, y*my) for (x,y) in state.route_pts_px]
    state.route_len_m = sum(_dist(a,b) for a,b in zip(state.route_pts_m, state.route_pts_m[1:]))

    # --- сброс прогресса для нового маршрута ---
    state.route_progress_idx = 0
    state.route_seg_off_m = 0.0
    state.route_done_m = 0.0
    state.route_finished = False

    print(f"[ROUTING] route pts: px={len(state.route_pts_px)}  m={len(state.route_pts_m)}  L={state.route_len_m:.2f}", flush=True)

    # --- КР: координаты вдоль маршрута в метрах ---
    compute_controls_on_route(state, eps_px=2.0)
    return True

# --- controls on route (метры получаем из PX через масштаб) -----------------
def _seg_len(a, b):
    import math
    return math.hypot(b[0]-a[0], b[1]-a[1])

def _point_to_segment_proj_px(a_px, b_px, p_px):
    """
    Проекция точки p_px на отрезок a_px–b_px в ПИКСЕЛЯХ.
    Возврат: (dist_px, t_clamped, proj_point_px). t in [0..1].
    """
    ax, ay = a_px; bx, by = b_px; px, py = p_px
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx*vx + vy*vy
    if vv <= 1e-12:
        t = 0.0
        qx, qy = ax, ay
    else:
        t = (wx*vx + wy*vy) / vv
        if t < 0.0: t = 0.0
        elif t > 1.0: t = 1.0
        qx, qy = ax + t*vx, ay + t*vy
    dist = _seg_len((px,py), (qx,qy))
    return dist, t, (qx, qy)

def _cumlen(poly):
    acc = [0.0]
    for a, b in zip(poly, poly[1:]):
        acc.append(acc[-1] + _seg_len(a, b))
    return acc

def compute_controls_on_route(state, eps_px: float = 3.0):
    """
    Считает state.route_controls_m = [(pt_px, s_m), ...].
    Геометрия маршрута: PX + параллельная М-ломаная, где М = PX ⊗ (mx,my).
    """
    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    ctrls   = getattr(state, "control_pts_px", None) or []

    out = []
    if len(poly_px) < 2 or len(poly_px) != len(poly_m) or not ctrls:
        state.route_controls_m = out
        return

    cum_m = _cumlen(poly_m)

    vert_s_exact = { (float(p[0]), float(p[1])): cum_m[i] for i, p in enumerate(poly_px) }
    def _key_round(p): return (round(float(p[0]), 3), round(float(p[1]), 3))
    vert_s_round = { _key_round(p): cum_m[i] for i, p in enumerate(poly_px) }

    for pt in ctrls:
        pt_px = (float(pt[0]), float(pt[1]))

        if pt_px in vert_s_exact:
            out.append((pt_px, vert_s_exact[pt_px]))
            continue

        k = _key_round(pt_px)
        if k in vert_s_round:
            out.append((pt_px, vert_s_round[k]))
            continue

        # проекция в PX -> длина в М через параллельную метрическую ломаную
        best = (float("inf"), 0, 0.0)
        for i, (a_px, b_px) in enumerate(zip(poly_px, poly_px[1:])):
            dist_px, t, _ = _point_to_segment_proj_px(a_px, b_px, pt_px)
            if dist_px < best[0]:
                best = (dist_px, i, t)

        dist_px, i, t = best
        if dist_px <= eps_px:
            seg_len_m = _seg_len(poly_m[i], poly_m[i+1])
            s_m = cum_m[i] + t * seg_len_m
            out.append((pt_px, s_m))

    out.sort(key=lambda x: x[1])
    state.route_controls_m = out

# ---------------- статусные тексты ----------------
def route_caption_text(state) -> str:
    L = float(getattr(state, "route_len_m", 0.0) or 0.0)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)
    left = max(0.0, L - done)
    line1 = f"Маршрут: {L:.1f} м | Пройдено: {done:.1f} м | Осталось: {left:.1f} м"

    ctrls = getattr(state, "route_controls_m", None) or []
    if not ctrls:
        line2 = "Нет контрольных точек"
    else:
        nxt = None
        for _, s_m in ctrls:
            if s_m > done + 1e-6:
                nxt = s_m
                break
        if nxt is None:
            line2 = "Все контрольные точки пройдены"
        else:
            to_next = max(0.0, nxt - done)
            line2 = f"До ближайшей КТ: {to_next:.1f} м"

    return line1 + "\n" + line2

# совместимость с robot_cmd.update_progress_text_for_robot(...)
def update_progress_text_for_robot(*args, **kwargs) -> str:
    state = args[-1] if args else kwargs.get("state")
    return route_caption_text(state)

# UI-хелпер для перерисовки поверх карты
def refresh_all_overlays(state, idle_view, drive_view):
    from graphics import redraw_markers, redraw_route
    for view in (idle_view, drive_view):
        if view and view.scene():
            redraw_markers(state, view.scene())
    if hasattr(state, "route_pts_px") and state.route_pts_px:
        redraw_route(state, None)

# Диагностика причин «нет валидного маршрута»
def debug_route_health(state, where: str = ""):
    try:
        print(f"[ROUTE DEBUG] {where}", flush=True)
        print("  robot_px:", getattr(state, "robot_px", None), flush=True)
        print("  goal_px :", getattr(state, "goal_px", None), flush=True)
        rpx = getattr(state, "route_pts_px", None) or []
        rmp = getattr(state, "route_pts_m",  None) or []
        print("  route_px pts:", len(rpx), flush=True)
        print("  route_m  pts:", len(rmp), flush=True)
        print("  L(m):", getattr(state, "route_len_m", 0.0), flush=True)
        print("  scale: mx=", getattr(state, "m_per_px_x", None),
              " my=", getattr(state, "m_per_px_y", None), flush=True)
    except Exception:
        pass