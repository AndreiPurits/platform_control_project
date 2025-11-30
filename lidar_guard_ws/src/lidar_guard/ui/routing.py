# -*- coding: utf-8 -*-
"""
== ROUTING (graph) ==========================================================
Артефакты рядом с PNG:
  <base>_graph.json           : {
      "nodes": {
          "px": [[x,y], ...],          # узлы в пикселях
          "m":  [[X,Y], ...]           # (может быть, но для метрик НЕ обязателен)
      },
      "edges": [
          {
            "u": int,                  # индекс узла
            "v": int,                  # индекс узла
            "poly_px": [[x,y], ...],   # ломаная вдоль дороги в PX
            "poly_m":  [[X,Y], ...]    # (может быть; для метрик НЕ обязателен)
          },
          ...
      ]
  }

  <base>_points_pixels.json   : [[x,y], ...] — плотные точки для снэпа кликов

Основные функции:
- load_graph_and_points_for(png_path, state) -> bool
- set_robot_pose_px(click_px, state) -> (x,y)
- set_goal_px(click_px, state) -> (x,y)
- set_control_item_px(click_px, state) -> list[(x,y)]
- build_route_from_robot_to_goal(state) -> bool   # путь строим по PX, метры через масштаб
- compute_controls_on_route(state, eps_px=...)    # привязка КТ к маршруту
- refresh_all_overlays(state, idle_view, drive_view) -> None
- route_caption_text(state) -> str
- debug_route_health(state, where="") -> None

ВНУТРЕННИЕ ИНВАРИАНТЫ:
- state.m_per_px_x, state.m_per_px_y задают АНИЗО масштабы карты.
- ВСЕ длины в метрах считаются ИЗ PX через (m_per_px_x, m_per_px_y), даже если poly_m есть.
===========================================================================
"""

from typing import List, Tuple, Optional, Dict, Any
import os
import json
import math
import heapq

Point = Tuple[float, float]

EDGE_POLY_KEY_PX = "poly_px"
EDGE_POLY_KEY_M  = "poly_m"     # не используем для метрик напрямую

# ---------------- utils ----------------

def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _subpoly_idx(poly: List[Point], i0: int, i1: int) -> List[Point]:
    """Подотрезок ломаной poly[i0..i1] с возможным разворотом."""
    if not poly:
        return []
    i0 = max(0, min(len(poly) - 1, int(i0)))
    i1 = max(0, min(len(poly) - 1, int(i1)))
    if i0 <= i1:
        return poly[i0:i1 + 1]
    seg = poly[i1:i0 + 1]
    seg.reverse()
    return seg


def _edge_polyline_px(e: Dict[str, Any]) -> List[Point]:
    """Читаем и нормализуем poly_px для ребра."""
    poly = e.get(EDGE_POLY_KEY_PX)
    if not isinstance(poly, list):
        return []
    if poly and not isinstance(poly[0], (list, tuple)):
        return []
    try:
        return [(float(p[0]), float(p[1])) for p in poly]
    except Exception:
        return []


def _nearest_from_list(target: Point, cloud: List[Point]) -> Point:
    """Простой линейный снэп к ближайшей точке из cloud."""
    tx, ty = target
    best, best_d2 = None, float("inf")
    for (x, y) in cloud:
        d2 = (x - tx) * (x - tx) + (y - ty) * (y - ty)
        if d2 < best_d2:
            best_d2 = d2
            best = (x, y)
    return best if best is not None else target


# ---------------- загрузка артефактов ----------------

def load_graph_and_points_for(png_path: str, state) -> bool:
    """
    Загружает:
      - graph:   <base>_graph.json
      - points:  <base>_points_pixels.json

    И выставляет:
      state.graph
      state.points_px
      state.m_per_px_x / state.m_per_px_y (для известных карт)
      state.junctions_px               (склеенные развилки)
    """
    base, _ = os.path.splitext(png_path)
    graph_json  = f"{base}_graph.json"
    points_json = f"{base}_points_pixels.json"

    print("[ROUTING] ---- load_graph_and_points_for ----")
    print("[ROUTING] png:", png_path)
    print("[ROUTING] graph exists:", os.path.isfile(graph_json))
    print("[ROUTING] points exists:", os.path.isfile(points_json))

    if not (os.path.isfile(graph_json) and os.path.isfile(points_json)):
        print("[ROUTING] missing graph/points files")
        return False

    try:
        # --- загрузка графа и точек ---
        with open(graph_json, "r", encoding="utf-8") as f:
            graph = json.load(f)
        with open(points_json, "r", encoding="utf-8") as f:
            pts_raw = json.load(f)

        if isinstance(pts_raw, dict) and "points" in pts_raw:
            pts_raw = pts_raw["points"]
        pts = [(float(x), float(y)) for x, y in pts_raw] if isinstance(pts_raw, list) else []

        state.graph = graph
        state.points_px = pts

        # --- масштаб только для известных карт (если надо, можно расширить) ---
        name = os.path.basename(png_path).lower()
        if "poly_asf" in name:
            state.m_per_px_x = 0.8342405111938622
            state.m_per_px_y = 1.2604320351524956
        elif "poly_grd" in name:
            state.m_per_px_x = 1.670743420684952
            state.m_per_px_y = 1.4202174529355311
        else:
            # по умолчанию хоть что-то, чтобы не было нулей
            state.m_per_px_x = float(getattr(state, "m_per_px_x", 1.0) or 1.0)
            state.m_per_px_y = float(getattr(state, "m_per_px_y", 1.0) or 1.0)

        # --- развилки (junctions_px) ---
        try:
            nodes_px = graph.get("nodes", {}).get("px") or []
            edges    = graph.get("edges", []) or []
            n        = len(nodes_px)

            if n == 0:
                state.junctions_px = []
                print("[ROUTING] junctions_px: no nodes", flush=True)
            else:
                # 1) считаем степень каждого узла
                deg = [0] * n
                for e in edges:
                    u = e.get("u")
                    v = e.get("v")
                    if (
                        isinstance(u, int) and isinstance(v, int)
                        and 0 <= u < n and 0 <= v < n
                    ):
                        deg[u] += 1
                        deg[v] += 1

                # 2) сырые кандидаты: степень >= 3
                junction_candidates = [i for i, d in enumerate(deg) if d >= 3]
                print(f"[ROUTING] raw junction candidates (deg>=3): {len(junction_candidates)}", flush=True)

                # 3) склеиваем кандидатов в кластеры по радиусу THRESHOLD_PX
                THRESHOLD_PX = 10.0
                junction_nodes: List[int] = []

                def _dist_px(i: int, j: int) -> float:
                    xi, yi = nodes_px[i]
                    xj, yj = nodes_px[j]
                    return _dist((xi, yi), (xj, yj))

                for i in junction_candidates:
                    if not junction_nodes:
                        junction_nodes.append(i)
                        continue
                    too_close = False
                    for j in junction_nodes:
                        if _dist_px(i, j) <= THRESHOLD_PX:
                            too_close = True
                            break
                    if not too_close:
                        junction_nodes.append(i)

                # 4) сохраняем центры кластеров в state.junctions_px
                state.junctions_px = [
                    (float(nodes_px[i][0]), float(nodes_px[i][1]))
                    for i in junction_nodes
                ]

                print(
                    f"[ROUTING] junctions_px fused (R={THRESHOLD_PX}px): "
                    f"{len(state.junctions_px)}",
                    flush=True
                )
        except Exception as ex:
            state.junctions_px = []
            print("[ROUTING] junctions_px calc error:", ex, flush=True)

        return True

    except Exception as e:
        print("[ROUTING] load error:", e, flush=True)
        return False


# ---------------- снэп позиций ----------------

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


def set_control_item_px(click_px: Point, state) -> List[Point]:
    """
    Добавляет новую контрольную точку (по клику) снэпнув к ближайшему
    point из points_px. Результат лежит в state.control_pts_px.
    """
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
    """
    Основной план:
      1) берем ближайшие к robot_px и goal_px вершины вдоль рёбер (по poly_px)
      2) строим расширенный граф с 2 виртуальными вершинами (start, goal)
      3) Дейкстра по пиксельной длине
      4) восстанавливаем полилинию route_pts_px
      5) считаем route_pts_m и route_len_m через (m_per_px_x, m_per_px_y)
    """
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
    adj: List[List[Tuple[int, int, float]]] = [[] for _ in range(n)]  # (v, edge_index, weight_px)

    for ei, e in enumerate(edges):
        u = e.get("u")
        v = e.get("v")
        poly_px = _edge_polyline_px(e)
        if (
            not isinstance(u, int) or not isinstance(v, int)
            or u < 0 or v < 0 or u >= n or v >= n
            or len(poly_px) < 2
        ):
            continue
        w = sum(_dist(a, b) for a, b in zip(poly_px, poly_px[1:]))
        adj[u].append((v, ei, w))
        adj[v].append((u, ei, w))

    # индекс всех вершин всех рёбер → (edge_id, local_idx, xy)
    all_vertices: List[Tuple[int, int, Point]] = []
    for ei, e in enumerate(edges):
        poly_px = _edge_polyline_px(e)
        for li, (x, y) in enumerate(poly_px):
            all_vertices.append((ei, li, (x, y)))

    def nearest_edge_vertex(pt: Point):
        """Возвращает (d2, ei, li, q) — ближайшая вершина ломаной среди всех poly_px."""
        tx, ty = pt
        best = (float("inf"), -1, -1, (tx, ty))
        for ei, li, (x, y) in all_vertices:
            d2 = (x - tx) * (x - tx) + (y - ty) * (y - ty)
            if d2 < best[0]:
                best = (d2, ei, li, (x, y))
        return best

    # 1) ближайшие вершины рёбер к start/goal
    _, ei_s, li_s, q_s = nearest_edge_vertex(start)
    _, ei_g, li_g, q_g = nearest_edge_vertex(goal)
    if ei_s < 0 or ei_g < 0:
        print("[ROUTING] no edge near start/goal", flush=True)
        return False

    es = edges[ei_s]
    eg = edges[ei_g]
    polyS = _edge_polyline_px(es)
    polyG = _edge_polyline_px(eg)
    if not polyS or not polyG:
        print("[ROUTING] bad edge polylines", flush=True)
        return False

    # быстрый кейс: оба клика в одном ребре — просто подотрезок poly_px
    if ei_s == ei_g:
        seg_px = _subpoly_idx(polyS, li_s, li_g)
        state.route_pts_px = seg_px

        # метрика ВСЕГДА через масштаб
        mx = float(getattr(state, "m_per_px_x", 1.0) or 1.0)
        my = float(getattr(state, "m_per_px_y", 1.0) or 1.0)
        state.route_pts_m = [(x * mx, y * my) for (x, y) in seg_px]
        state.route_len_m = sum(_dist(a, b) for a, b in zip(state.route_pts_m, state.route_pts_m[1:]))

        # сброс прогресса
        state.route_progress_idx = 0
        state.route_seg_off_m = 0.0
        state.route_done_m = 0.0
        state.route_finished = False

        print(f"[ROUTING] route (single edge): px={len(state.route_pts_px)}  m={len(state.route_pts_m)}  L={state.route_len_m:.2f}", flush=True)
        compute_controls_on_route(state, eps_px=2.0)
        return True

    # 2) строим расширенный граф с 2 «виртуальными» узлами
    nodes_ext = list(nodes_px)
    idx_start = len(nodes_ext); nodes_ext.append([q_s[0], q_s[1]])
    idx_goal  = len(nodes_ext); nodes_ext.append([q_g[0], q_g[1]])

    adj_ext: List[List[Tuple[int, int, float]]] = [list(row) for row in adj]
    adj_ext.extend([[], []])  # для двух виртуальных

    def _edge_len_px(poly_px: List[Point]) -> float:
        return sum(_dist(a, b) for a, b in zip(poly_px, poly_px[1:]))

    def connect_virtual(idx_vrt: int, e_idx: int, l_idx: int):
        """
        Виртуальная вершина "сидит" на poly_px[e_idx][l_idx].
        Соединяем её с u/v этого ребра, вес = длина по poly_px.
        """
        e = edges[e_idx]
        poly_px = _edge_polyline_px(e)
        if len(poly_px) < 2:
            return
        u = e.get("u")
        v = e.get("v")
        if not isinstance(u, int) or not isinstance(v, int):
            return

        # аккумулированные длины вдоль poly_px
        acc = [0.0]
        for a, b in zip(poly_px, poly_px[1:]):
            acc.append(acc[-1] + _dist(a, b))
        L = acc[-1]
        s_here = acc[max(0, min(len(acc) - 1, l_idx))]

        # виртуальная вершина делит ребро в этой точке:
        # от неё до u — s_here; до v — L - s_here
        # NB: direction не критичен, главное — симметрия
        if 0 <= u < len(adj_ext):
            w = s_here
            adj_ext[idx_vrt].append((u, -1, w))
            adj_ext[u].append((idx_vrt, -1, w))
        if 0 <= v < len(adj_ext):
            w = max(L - s_here, 0.0)
            adj_ext[idx_vrt].append((v, -1, w))
            adj_ext[v].append((idx_vrt, -1, w))

    connect_virtual(idx_start, ei_s, li_s)
    connect_virtual(idx_goal,  ei_g, li_g)

    # 3) Дейкстра по расширенному графу (вес — пиксельная длина)
    N = len(adj_ext)
    D = [float("inf")] * N
    P: List[Optional[Tuple[int, int]]] = [None] * N   # (prev_node, edge_index_meta)
    D[idx_start] = 0.0
    pq: List[Tuple[float, int]] = [(0.0, idx_start)]

    while pq:
        d, u = heapq.heappop(pq)
        if d != D[u]:
            continue
        if u == idx_goal:
            break
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
    steps: List[Tuple[int, int, int]] = []
    cur = idx_goal
    while cur != idx_start:
        prev, ei_meta = P[cur]
        steps.append((prev, cur, ei_meta))
        cur = prev
    steps.reverse()

    # сопоставление ребра по (u,v)
    edge_by_pair: Dict[Tuple[int, int], int] = {}
    for ei, e in enumerate(edges):
        u, v = e.get("u"), e.get("v")
        if isinstance(u, int) and isinstance(v, int):
            edge_by_pair[(u, v)] = ei
            edge_by_pair[(v, u)] = ei

    def edge_between(u: int, v: int) -> int:
        return edge_by_pair.get((u, v), -1)

    pts_px: List[Point] = []

    def add_edge_segment(edge_idx: int, i0: int, i1: int):
        nonlocal pts_px
        e = edges[edge_idx]
        seg_px = _subpoly_idx(_edge_polyline_px(e), i0, i1)
        if not seg_px:
            return
        if pts_px:
            if pts_px[-1] == seg_px[0]:
                pts_px.extend(seg_px[1:])
            else:
                pts_px.extend(seg_px)
        else:
            pts_px.extend(seg_px)

    # где лежат виртуальные: их привязка к своим poly_px
    # (для восстановления кусочков к/от реальных узлов)
    # idx_start -> (ei_s, li_s), idx_goal -> (ei_g, li_g)

    for (a, b, ei_meta) in steps:
        # переход между двумя реальными узлами
        if a < n and b < n:
            ei = ei_meta if ei_meta >= 0 else edge_between(a, b)
            if ei < 0:
                continue
            e = edges[ei]
            poly = _edge_polyline_px(e)
            if e.get("u") == a and e.get("v") == b:
                add_edge_segment(ei, 0, len(poly) - 1)
            elif e.get("u") == b and e.get("v") == a:
                add_edge_segment(ei, len(poly) - 1, 0)
            continue

        # виртуальный -> реальный
        # простая логика: если шаг содержит idx_start или idx_goal,
        # добавляем половинку ребра до соответствующего узла
        if a == idx_start or b == idx_start:
            e = es
            poly = polyS
            # от виртуала (li_s) до узла a/b (u или v)
            node_idx = a if a < n else b
            if node_idx == e.get("u"):
                # путь "от" виртуала к u
                # если мы ДВИГАЕМСЯ к u, то в общую ломаную идём от li_s к 0
                add_edge_segment(ei_s, li_s, 0)
            elif node_idx == e.get("v"):
                add_edge_segment(ei_s, li_s, len(poly) - 1)

        elif a == idx_goal or b == idx_goal:
            e = eg
            poly = polyG
            node_idx = a if a < n else b
            if node_idx == e.get("u"):
                add_edge_segment(ei_g, 0, li_g)
            elif node_idx == e.get("v"):
                add_edge_segment(ei_g, len(poly) - 1, li_g)
        else:
            # переход между двумя виртуальными не ожидается
            pass

    # Если по какой-то причине всё ещё пусто — fallback:
    if not pts_px:
        print("[ROUTING] WARNING: pts_px empty after reconstruction, fallback to simple poly_s->poly_g", flush=True)
        # грубый Fallback: start ребро (от li_s до конца) + goal ребро (от начала до li_g)
        add_edge_segment(ei_s, li_s, len(polyS) - 1)
        add_edge_segment(ei_g, 0, li_g)

    state.route_pts_px = pts_px

    # --- МЕТРИКА: АНИЗО МАСШТАБ ---
    mx = float(getattr(state, "m_per_px_x", 1.0) or 1.0)
    my = float(getattr(state, "m_per_px_y", 1.0) or 1.0)
    state.route_pts_m = [(x * mx, y * my) for (x, y) in state.route_pts_px]
    state.route_len_m = sum(_dist(a, b) for a, b in zip(state.route_pts_m, state.route_pts_m[1:]))

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

def _seg_len(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _point_to_segment_proj_px(a_px: Point, b_px: Point, p_px: Point):
    """
    Проекция точки p_px на отрезок a_px–b_px в ПИКСЕЛЯХ.
    Возврат: (dist_px, t_clamped, proj_point_px). t in [0..1].
    """
    ax, ay = a_px
    bx, by = b_px
    px, py = p_px
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv <= 1e-12:
        t = 0.0
        qx, qy = ax, ay
    else:
        t = (wx * vx + wy * vy) / vv
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        qx, qy = ax + t * vx, ay + t * vy
    dist_px = _seg_len((px, py), (qx, qy))
    return dist_px, t, (qx, qy)


def _cumlen(poly: List[Point]) -> List[float]:
    acc = [0.0]
    for a, b in zip(poly, poly[1:]):
        acc.append(acc[-1] + _seg_len(a, b))
    return acc


def compute_controls_on_route(state, eps_px: float = 3.0):
    """
    Считает state.route_controls_m = [(pt_px, s_m), ...].

    Логика:
      - есть route_pts_px (ломаная маршрута в PX)
      - к каждому control_pts_px привязываемся через проекцию на эту ломаную
      - переводим положение вдоль маршрута в МЕТРЫ через параллельную poly_m
        (которая = route_pts_px ⊗ scale)
    """
    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    ctrls   = getattr(state, "control_pts_px", None) or []

    out = []
    if len(poly_px) < 2 or len(poly_px) != len(poly_m) or not ctrls:
        state.route_controls_m = out
        return

    cum_m = _cumlen(poly_m)

    vert_s_exact: Dict[Point, float] = {
        (float(p[0]), float(p[1])): cum_m[i] for i, p in enumerate(poly_px)
    }

    def _key_round(p: Point) -> Point:
        return (round(float(p[0]), 3), round(float(p[1]), 3))

    vert_s_round: Dict[Point, float] = {
        _key_round(p): cum_m[i] for i, p in enumerate(poly_px)
    }

    for pt in ctrls:
        pt_px = (float(pt[0]), float(pt[1]))

        # 1) точное совпадение вершины
        if pt_px in vert_s_exact:
            out.append((pt_px, vert_s_exact[pt_px]))
            continue

        # 2) округлённое совпадение
        k = _key_round(pt_px)
        if k in vert_s_round:
            out.append((pt_px, vert_s_round[k]))
            continue

        # 3) проекция в PX -> длина в М через параллельную метрическую ломаную
        best = (float("inf"), 0, 0.0)
        for i, (a_px, b_px) in enumerate(zip(poly_px, poly_px[1:])):
            dist_px, t, _ = _point_to_segment_proj_px(a_px, b_px, pt_px)
            if dist_px < best[0]:
                best = (dist_px, i, t)

        dist_px, i, t = best
        if dist_px <= eps_px:
            seg_len_m = _seg_len(poly_m[i], poly_m[i + 1])
            s_m = cum_m[i] + t * seg_len_m
            out.append((pt_px, s_m))

    out.sort(key=lambda x: x[1])
    state.route_controls_m = out


# ---------------- статусные тексты ----------------
from typing import Optional

def nearest_junction_distance(state, eps_px: float = 5.0) -> Optional[float]:
    """
    Расстояние по маршруту (в метрах) до ближайшего вперёд лежащего перекрёстка.
    Плюс побочные эффекты:
      - state.jdist       = это расстояние (или None)
      - state._junc_idx   = индекс точки маршрута, ближайшей к этому перекрёстку (или None)
    """
    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    junctions = getattr(state, "junctions_px", None) or []

    # значения по умолчанию
    state.jdist = None
    state._junc_idx = None

    if len(poly_px) < 2 or len(poly_px) != len(poly_m) or not junctions:
        return None

    # cum длины по poly_m
    cum_m = _cumlen(poly_m)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)

    best_delta = None
    best_idx   = None

    for j in junctions:
        pt_px = (float(j[0]), float(j[1]))

        best = (float("inf"), 0, 0.0)
        for i, (a_px, b_px) in enumerate(zip(poly_px, poly_px[1:])):
            dist_px, t, _ = _point_to_segment_proj_px(a_px, b_px, pt_px)
            if dist_px < best[0]:
                best = (dist_px, i, t)

        dist_px, i, t = best
        if dist_px > eps_px:
            continue

        seg_len_m = math.hypot(
            poly_m[i+1][0] - poly_m[i][0],
            poly_m[i+1][1] - poly_m[i][1],
        )
        s_m = cum_m[i] + t * seg_len_m

        if s_m <= done + 0.1:
            continue

        delta = s_m - done
        if best_delta is None or delta < best_delta:
            best_delta = delta
            best_idx   = i

    if best_delta is None:
        state.jdist = None
        state._junc_idx = None
        return None

    state.jdist = float(best_delta)
    state._junc_idx = int(best_idx)
    return best_delta





# UI-хелпер для перерисовки поверх карты
def refresh_all_overlays(state, idle_view, drive_view):
    from graphics import redraw_markers, redraw_route, redraw_junctions

    for view in (idle_view, drive_view):
        if view and view.scene():
            sc = view.scene()
            redraw_markers(state, sc)
            redraw_junctions(state, sc)

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
        print("  controls:", getattr(state, "control_pts_px", None), flush=True)
    except Exception:
        pass

def nearest_junction_ahead(state, max_dist_m: float = 20.0):
    """
    Ищет ближайшую развилку ВДОЛЬ МАРШРУТА вперёд от текущего прогресса.
    Возвращает (dist_to_junction_m, junction_px) или None.
    Печатает в лог: 'через X.X м перекрёсток', если нашло.
    """
    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    juncs   = getattr(state, "junctions_px", None) or []
    done    = float(getattr(state, "route_done_m", 0.0) or 0.0)

    if len(poly_px) < 2 or len(poly_px) != len(poly_m) or not juncs:
        return None

    # накопленные длины по маршруту в метрах
    cum_m = _cumlen(poly_m)

    best = (float("inf"), None)

    for jpx in juncs:
        pt_px = (float(jpx[0]), float(jpx[1]))
        # проецируем развилку на маршрут (в PX), но длину берём из метрической ломаной
        best_seg = (float("inf"), 0, 0.0)
        for i, (a_px, b_px) in enumerate(zip(poly_px, poly_px[1:])):
            dist_px, t, _ = _point_to_segment_proj_px(a_px, b_px, pt_px)
            if dist_px < best_seg[0]:
                best_seg = (dist_px, i, t)

        dist_px, i, t = best_seg
        # небольшой порог в пикселях, чтобы не ловить мусор
        if dist_px > 10.0:
            continue

        seg_len_m = _seg_len(poly_m[i], poly_m[i+1])
        s_m = cum_m[i] + t * seg_len_m   # абсолютная длина вдоль маршрута до развилки
        to_j = s_m - done                # сколько осталось от текущего прогресса

        if to_j > 0.0 and to_j < best[0]:
            best = (to_j, pt_px)

    if best[1] is None or best[0] > max_dist_m:
        return None

    dist_m, jpx = best
    print(f"[JUNC] через {dist_m:.1f} м перекрёсток @ {jpx}", flush=True)
    return dist_m, jpx

def build_route_via_goals(state) -> bool:
    """
    Строит кусочный маршрут:
        robot_px -> goal_pts_px[0] -> goal_pts_px[1] -> ... -> goal_pts_px[-1]
    Использует существующий build_route_from_robot_to_goal.
    Результат кладёт в state.route_pts_px / route_pts_m / route_len_m.
    """
    start = getattr(state, "robot_px", None)
    goals = list(getattr(state, "goal_pts_px", []) or [])

    if not start or not goals:
        print("[ROUTING] via_goals: missing start or goals", flush=True)
        return False

    waypoints = [start] + goals

    all_px: List[Point] = []
    all_m:  List[Point] = []

    # сохраним исходные robot/goal
    orig_robot = getattr(state, "robot_px", None)
    orig_goal  = getattr(state, "goal_px",  None)

    for i in range(len(waypoints) - 1):
        seg_start = waypoints[i]
        seg_goal  = waypoints[i+1]

        state.robot_px = seg_start
        state.goal_px  = seg_goal

        ok = build_route_from_robot_to_goal(state)
        if not ok:
            print(f"[ROUTING] via_goals: segment {i} failed: {seg_start} -> {seg_goal}", flush=True)
            # откат
            state.robot_px = orig_robot
            state.goal_px  = orig_goal
            return False

        seg_px = list(getattr(state, "route_pts_px", []) or [])
        seg_m  = list(getattr(state, "route_pts_m",  []) or [])

        if not seg_px or not seg_m:
            print(f"[ROUTING] via_goals: empty segment {i}", flush=True)
            state.robot_px = orig_robot
            state.goal_px  = orig_goal
            return False

        if not all_px:
            all_px.extend(seg_px)
            all_m.extend(seg_m)
        else:
            # чтобы не дублировать стык — добавляем с 1-го элемента
            all_px.extend(seg_px[1:])
            all_m.extend(seg_m[1:])

    # собираем общий маршрут
    state.route_pts_px = all_px
    state.route_pts_m  = all_m
    state.route_len_m  = sum(_seg_len(a, b) for a, b in zip(all_m, all_m[1:]))

    state.route_progress_idx = 0
    state.route_seg_off_m = 0.0
    state.route_done_m = 0.0
    state.route_finished = False

    # можно использовать контрольные точки (синие флаги) как КТ вдоль маршрута
    compute_controls_on_route(state, eps_px=3.0)

    print(
        f"[ROUTING] via_goals: total pts px={len(state.route_pts_px)} "
        f"m={len(state.route_pts_m)} L={state.route_len_m:.2f}",
        flush=True,
    )
    return True
def add_goal_point_px(click_px: Point, state) -> list[Point]:
    """
    Многоточечный режим: добавляет ещё одну цель в state.goal_pts_px
    (снэп по points_px) и обновляет state.goal_px (= последняя цель).
    """
    pts = getattr(state, "points_px", None) or []
    snapped = _nearest_from_list(click_px, pts) if pts else (float(click_px[0]), float(click_px[1]))
    gp = (float(snapped[0]), float(snapped[1]))

    if not hasattr(state, "goal_pts_px") or state.goal_pts_px is None:
        state.goal_pts_px = []
    state.goal_pts_px.append(gp)

    # для совместимости пусть goal_px = последняя цель
    state.goal_px = gp

    print(f"[ROUTING] Added goal point: {gp}", flush=True)
    print(f"[ROUTING] Total goal points: {len(state.goal_pts_px)}", flush=True)
    return state.goal_pts_px


def clear_goals_and_route(state, also_clear_controls: bool = False):
    """
    Полный сброс целей и маршрута — вызывать при повторном нажатии 'Цель'.
    """
    state.goal_px = None
    state.goal_pts_px = []

    # сам маршрут
    state.route_pts_px = []
    state.route_pts_m  = []
    state.route_len_m  = 0.0
    state.route_done_m = 0.0
    state.route_finished = True

    if also_clear_controls:
        state.control_pts_px = []
        state.route_controls_m = []

    from graphics import clear_route_visual, refresh_all_overlays
    clear_route_visual(state, also_clear_data=False)  # графика маршрута
    # сами данные мы уже стерли выше
    try:
        idle_view = getattr(state, "_idle_view", None)
        drive_view = getattr(state, "_drive_view", None)
        refresh_all_overlays(state, idle_view, drive_view)
    except Exception:
        pass

    print("[ROUTING] goals & route cleared", flush=True)

def _polyline_len_px(pts: List[Point], i0: int, i1: int) -> float:
    """
    Длина ломаной pts[i0..i1] в пикселях.
    Использует тот же helper _seg_len, что и остальной routing.
    """
    n = len(pts)
    if n < 2:
        return 0.0

    i0 = max(0, min(n - 1, int(i0)))
    i1 = max(0, min(n - 1, int(i1)))
    if i0 >= i1:
        return 0.0

    s = 0.0
    for a, b in zip(pts[i0:i1], pts[i0+1:i1+1]):
        s += _seg_len(a, b)
    return s


def _signed_turn_angle(p_prev: Point, p_junc: Point, p_next: Point) -> float:
    """
    Возвращает подписанный угол (радианы):
       + (плюс) = налево
       - (минус) = направо
       по трём точкам маршрута.
    """
    ax, ay = p_prev[0] - p_junc[0], p_prev[1] - p_junc[1]
    bx, by = p_next[0] - p_junc[0], p_next[1] - p_junc[1]

    la = math.hypot(ax, ay) or 1.0
    lb = math.hypot(bx, by) or 1.0
    ax /= la; ay /= la
    bx /= lb; by /= lb

    dot = max(-1.0, min(1.0, ax*bx + ay*by))
    ang = math.acos(dot)
    cross = ax*by - ay*bx  # знак
    return ang * (1.0 if cross > 0 else -1.0)

def detect_next_turn(state,
                     lookahead_m: float = 50.0,
                     min_angle_deg: float = 15.0):
    """
    Возвращает только turn ∈ {'left', 'right', 'straight'} или None.

    Параллельно вешает на state:
      state._detected_turn_j_idx  — индекс точки маршрута, где измеряли угол.
    """
    pts = getattr(state, "route_pts_px", None)
    robot_px = getattr(state, "robot_px", None)
    juncs = getattr(state, "junctions_px", None)

    # по умолчанию — ничего не нашли
    state._detected_turn_j_idx = None

    if not pts or robot_px is None or not juncs:
        return None

    # pixels-per-meter
    ppm = float(getattr(state, "ppm", getattr(state, "PPM", 80.0)) or 80.0)
    lookahead_px = lookahead_m * ppm

    # --- 1. Находим индекс ближайшей точки маршрута к роботу ---
    rx, ry = robot_px
    best_i = 0
    best_d2 = float("inf")

    for i, (x, y) in enumerate(pts):
        d2 = (x - rx) ** 2 + (y - ry) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    cur_idx = best_i
    n = len(pts)

    best_turn = None
    best_dist_m = None
    best_ji = None

    # --- 2. Для каждого перекрёстка ищем его положение на маршруте ---
    for jx, jy in juncs:
        # ближайшая точка маршрута к перекрёстку
        ji = 0
        jd2 = float("inf")
        for i, (x, y) in enumerate(pts):
            d2 = (x - jx) ** 2 + (y - jy) **2
            if d2 < jd2:
                jd2 = d2
                ji = i

        if ji <= cur_idx:
            # перекрёсток уже позади
            continue

        # расстояние вдоль ломаной от робота до перекрёстка (в px)
        dist_px = _polyline_len_px(pts, cur_idx, ji)
        if dist_px > lookahead_px:
            continue

        # угол поворота в точке ji
        if 0 < ji < n - 1:
            p_prev = pts[ji - 1]
            p_junc = pts[ji]
            p_next = pts[ji + 1]

            ang = _signed_turn_angle(p_prev, p_junc, p_next)
            ang_deg = ang * 180.0 / math.pi

            if abs(ang_deg) < min_angle_deg:
                turn = "straight"
            elif ang_deg > 0:
                turn = "left"
            else:
                turn = "right"

            dist_m = dist_px / ppm

            # выберем ближайший "осмысленный" поворот по расстоянию
            if best_dist_m is None or dist_m < best_dist_m:
                best_dist_m = dist_m
                best_turn = turn
                best_ji = ji

    # запоминаем, где именно мы измеряли поворот
    state._detected_turn_j_idx = best_ji
    return best_turn


def update_turn_hint(state,
                     lookahead_m: float = 15.0,
                     min_angle_deg: float = 35.0) -> None:
    """
    Обновляет:
      state.next_turn_dir ∈ {'left','right','straight'} или None

    Доп. логика: возле одного и того же перекрёстка направление
    не прыгает left→right→left:
      - если новый поворот противоположен старому,
        и индекс перекрёстка почти тот же, то смену игнорируем.
    """
    prev_dir = getattr(state, "next_turn_dir", None)
    prev_idx = getattr(state, "next_turn_j_idx", None)

    turn = detect_next_turn(
        state,
        lookahead_m=lookahead_m,
        min_angle_deg=min_angle_deg,
    )
    cur_idx = getattr(state, "_detected_turn_j_idx", None)

    # --- Гистерезис по "одному перекрёстку" ---
    if (
        prev_dir in ("left", "right") and
        turn in ("left", "right") and
        turn != prev_dir and
        prev_idx is not None and
        cur_idx is not None and
        abs(int(prev_idx) - int(cur_idx)) <= 5  # тот же самый узел/окрестность
    ):
        # Считаем, что это тот же перекрёсток, и не даём направлению прыгать.
        # Просто выходим, не меняя next_turn_dir.
        return

    # принимаем новое значение
    state.next_turn_dir = turn
    state.next_turn_j_idx = cur_idx

    # лог при изменении
    if prev_dir != turn:
        print(f"[TURN HINT] поворот {turn}", flush=True)

def route_caption_text(state) -> str:
    L = float(getattr(state, "route_len_m", 0.0) or 0.0)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)
    left = max(0.0, L - done)
    line1 = f"Маршрут: {L:.1f} м | Пройдено: {done:.1f} м | Осталось: {left:.1f} м"

    # КТ
    ctrls = getattr(state, "route_controls_m", None) or []
    if not ctrls:
        line2 = "До ближайшей КТ: —"
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

    # Перекрёсток
    try:
        jdist = nearest_junction_distance(state, eps_px=5.0)
    except Exception:
        jdist = None

    if jdist is None:
        line3 = "До перекрёстка: —"
    else:
        line3 = f"До перекрёстка: {jdist:.1f} м"

    return line1 + "\n" + line2 + "\n" + line3

def update_progress_text_for_robot(*args, **kwargs) -> str:
    state = args[-1] if args else kwargs.get("state")
    return route_caption_text(state)

import numpy as np

def maybe_snap_robot_to_junction_by_camera(
    state,
    mask: np.ndarray,
    max_graph_dist_m: float = 5.0,
    min_jdist_m: float = 1.0,
    max_jdist_m: float = 12.0,
) -> None:
    """
    Камера говорит: "похоже, это перекрёсток" → аккуратно притягиваем robot_px
    к ближайшему junction на графе и поправляем route_done_m.

    Условия:
      - есть текущий маршрут (route_pts_px / route_pts_m),
      - есть junctions_px,
      - jdist (расстояние до перекрёстка по маршруту) попадает в окно [min_jdist_m, max_jdist_m],
      - по маске видно "расширение" дороги (простейший детектор).
    """
    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    juncs   = getattr(state, "junctions_px", None) or []
    robot   = getattr(state, "robot_px", None)
    jdist   = getattr(state, "jdist", None)  # ТВОЙ СТАРЫЙ jdist для расстояния

    if (
        mask is None
        or mask.size == 0
        or not poly_px
        or not poly_m
        or not juncs
        or robot is None
        or jdist is None
    ):
        return

    # 1) проверяем, что мы реально "где-то у перекрёстка по маршруту"
    if not (min_jdist_m <= jdist <= max_jdist_m):
        return

    # 2) грубый детектор: в верхней части кадра дорога «расширяется»
    H, W = mask.shape[:2]
    thr = 0.6
    road_bin = (mask > thr)

    band_h = max(3, int(H * 0.25))   # верхняя четверть кадра
    band = road_bin[:band_h, :]

    # считаем заполненность слева/центра/справа
    third = max(1, W // 3)
    left_band   = band[:, :third]
    center_band = band[:, third:2*third]
    right_band  = band[:, 2*third:]

    lf = float(left_band.mean())
    cf = float(center_band.mean())
    rf = float(right_band.mean())

    # простое условие: перекрёсток, когда по сравнению с "прямой дорогой"
    # заполняются боковые зоны (типа "есть ещё ветви")
    # пороги можно будет потом крутить
    EXTRA = 0.04
    if (lf < cf + EXTRA) and (rf < cf + EXTRA):
        # похоже, просто прямая дорога, без явных боковых ответвлений
        return
    print(f"[CAM JUNC] камера видит развилку: lf={lf:.3f} cf={cf:.3f} rf={rf:.3f}", flush=True)
    # 3) если маска говорит "есть развилка", притягиваем robot_px
    #    к ближайшему по маршруту junction
    rx, ry = robot
    best = None  # (dist_px, jx, jy, ji)

    # вспомогательная: индекс точки маршрута, ближайшей к (jx,jy)
    def _nearest_idx_on_route(jx, jy):
        best_i = 0
        best_d2 = float("inf")
        for i, (x, y) in enumerate(poly_px):
            d2 = (x - jx)**2 + (y - jy)**2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i, best_d2

    for (jx, jy) in juncs:
        ji, d2 = _nearest_idx_on_route(jx, jy)
        # немного ограничим по пиксельной дистанции, чтобы не прыгнуть слишком далеко
        if d2 > (80.0 * 80.0):  # ~1 м при 80 px/м
            continue
        if best is None or d2 < best[0]:
            best = (d2, jx, jy, ji)

    if best is None:
        return

    _, jx, jy, ji = best

    # 4) пересчёт пройденного пути до этого индекса
    def _cumlen(points):
        acc = [0.0]
        total = 0.0
        for a, b in zip(points, points[1:]):
            d = math.hypot(b[0]-a[0], b[1]-a[1])
            total += d
            acc.append(total)
        return acc

    cum_m = _cumlen(poly_m)
    if 0 <= ji < len(cum_m):
        new_done = float(cum_m[ji])
    else:
        return

    # 5) аккуратно обновляем state
    state.robot_px = (float(jx), float(jy))
    state.route_done_m = new_done

    print(
        f"[CAM SNAP] перекрёсток детектирован по камере → "
        f"snap robot_px -> ({jx:.1f},{jy:.1f}), route_done_m={new_done:.2f}",
        flush=True,
    )