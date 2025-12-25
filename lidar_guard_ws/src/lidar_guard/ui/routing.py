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
import numpy as np
Point = Tuple[float, float]

EDGE_POLY_KEY_PX = "poly_px"
EDGE_POLY_KEY_M  = "poly_m"  

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
import os
import json
def apply_scale_from_graph(map_png_path: str, state) -> bool:
    """
    Читает масштаб из <basename>_graph.json и пишет в state.m_per_px_x / state.m_per_px_y.

    Поддерживаем варианты:
      - graph["m_per_px_x"], graph["m_per_px_y"]
      - graph["meta"]["m_per_px_x/y"] или meta["m_per_px"]
      - graph["scale"]["m_per_px_x/y"]  <-- твой текущий формат
    """
    base, _ = os.path.splitext(map_png_path)
    graph_path = base + "_graph.json"

    if not os.path.exists(graph_path):
        print(f"[MAP] graph json not found: {graph_path}", flush=True)
        return False

    try:
        with open(graph_path, "r", encoding="utf-8") as f:
            graph = json.load(f)
    except Exception as e:
        print(f"[MAP] failed to load graph json: {graph_path}  error={e}", flush=True)
        return False

    if not isinstance(graph, dict):
        print("[MAP] graph json is not a dict", flush=True)
        return False

    mppx = graph.get("m_per_px_x")
    mppy = graph.get("m_per_px_y")

    # --- NEW: scale.{m_per_px_x, m_per_px_y} ---
    if (mppx is None) or (mppy is None):
        sc = graph.get("scale")
        if isinstance(sc, dict):
            if mppx is None:
                mppx = sc.get("m_per_px_x", sc.get("m_per_px"))
            if mppy is None:
                mppy = sc.get("m_per_px_y", sc.get("m_per_px"))

    # --- fallback: meta.{m_per_px_x, m_per_px_y, m_per_px} ---
    if (mppx is None) or (mppy is None):
        meta = graph.get("meta")
        if isinstance(meta, dict):
            if mppx is None:
                mppx = meta.get("m_per_px_x", meta.get("m_per_px"))
            if mppy is None:
                mppy = meta.get("m_per_px_y", meta.get("m_per_px"))

    if not isinstance(mppx, (int, float)) or not isinstance(mppy, (int, float)):
        print("[MAP] no valid m_per_px_x / m_per_px_y in graph.json", flush=True)
        return False

    state.m_per_px_x = float(mppx)
    state.m_per_px_y = float(mppy)

    # --- LOG: финальный масштаб ---
    m_per_px_avg = 0.5 * (state.m_per_px_x + state.m_per_px_y)
    px_per_m = 1.0 / max(m_per_px_avg, 1e-9)
    print(
        "[SCALE] loaded from graph.json:"
        f" m_per_px_x={state.m_per_px_x:.12f}"
        f" m_per_px_y={state.m_per_px_y:.12f}"
        f" avg_m_per_px={m_per_px_avg:.12f}"
        f" avg_px_per_m={px_per_m:.3f}",
        flush=True
    )
    return True

def load_graph_and_points_for(png_path: str, state) -> bool:
    """
    Загружает:
      - graph:   <base>_graph.json
      - points:  <base>_points_pixels.json

    И выставляет:
      state.graph
      state.points_px
      state.m_per_px_x / state.m_per_px_y  (приоритет: graph.json -> fallback)
      state.junctions_px                   (склеенные развилки)
    """
    base, _ = os.path.splitext(png_path)
    graph_json  = f"{base}_graph.json"
    points_json = f"{base}_points_pixels.json"

    print("[ROUTING] ---- load_graph_and_points_for ----", flush=True)
    print("[ROUTING] png:", png_path, flush=True)
    print("[ROUTING] graph exists:", os.path.isfile(graph_json), flush=True)
    print("[ROUTING] points exists:", os.path.isfile(points_json), flush=True)

    if not (os.path.isfile(graph_json) and os.path.isfile(points_json)):
        print("[ROUTING] missing graph/points files", flush=True)
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

        # ------------------------------------------------------------
        # SCALE: приоритет = graph.json (apply_scale_from_graph),
        # fallback = хардкод по имени (только если в json нет)
        # ------------------------------------------------------------
        got_scale = apply_scale_from_graph(png_path, state)

        if not got_scale:
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

            m_avg = 0.5 * (state.m_per_px_x + state.m_per_px_y)
            print(
                "[SCALE] fallback scale:"
                f" m_per_px_x={state.m_per_px_x:.12f}"
                f" m_per_px_y={state.m_per_px_y:.12f}"
                f" avg_m_per_px={m_avg:.12f}",
                flush=True
            )

        # --- LOG: финальный масштаб, который реально будет использоваться везде ---
        m_avg = 0.5 * (float(state.m_per_px_x) + float(state.m_per_px_y))
        px_per_m = 1.0 / max(m_avg, 1e-9)
        print(
            "[SCALE] FINAL:"
            f" m_per_px_x={float(state.m_per_px_x):.12f}"
            f" m_per_px_y={float(state.m_per_px_y):.12f}"
            f" avg_m_per_px={m_avg:.12f}"
            f" avg_px_per_m={px_per_m:.3f}",
            flush=True
        )

        # ------------------------------------------------------------
        # JUNCTIONS (junctions_px): узлы степени >= 3, склейка близких
        # ------------------------------------------------------------
        try:
            nodes_px = graph.get("nodes", {}).get("px") or []
            edges    = graph.get("edges", []) or []
            n        = len(nodes_px)

            if n == 0:
                state.junctions_px = []
            else:
                # 1) степень каждого узла
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
                # 2) кандидаты: степень >= 3
                junction_candidates = [i for i, d in enumerate(deg) if d >= 3]

                # 3) склейка кандидатов в кластеры по радиусу
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

                # 4) сохраняем junctions_px
                state.junctions_px = [
                    (float(nodes_px[i][0]), float(nodes_px[i][1]))
                    for i in junction_nodes
                ]

                print(f"[ROUTING] junctions_px: {len(state.junctions_px)}", flush=True)

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
    #print(f"[ROUTING] Total control points: {len(state.control_pts_px)}", flush=True)
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
    if state.route_pts_px:
        start = state.route_pts_px[-1]
    else:
        start = state.robot_px
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
        pts_px = _subpoly_idx(polyS, li_s, li_g)
        if not pts_px or len(pts_px) < 2:
            print("[ROUTING] single-edge: empty segment", flush=True)
            return False

        # --- ДОПИСЫВАЕМ В ОБЩИЙ МАРШРУТ ---
        old_px = getattr(state, "route_pts_px", None) or []
        if old_px:
            if old_px[-1] == pts_px[0]:
                state.route_pts_px = old_px + pts_px[1:]
            else:
                state.route_pts_px = old_px + pts_px
        else:
            state.route_pts_px = pts_px

        # --- МЕТРЫ ДОПИСЫВАЕМ АНАЛОГИЧНО ---
        mx = float(getattr(state, "m_per_px_x", 1.0) or 1.0)
        my = float(getattr(state, "m_per_px_y", 1.0) or 1.0)

        new_m = [(x * mx, y * my) for (x, y) in pts_px]
        old_m = getattr(state, "route_pts_m", None) or []
        if old_m:
            if old_m[-1] == new_m[0]:
                state.route_pts_m = old_m + new_m[1:]
            else:
                state.route_pts_m = old_m + new_m
        else:
            state.route_pts_m = new_m

        # --- ДЛИНА: пересчёт по всему маршруту (надёжно) ---
        state.route_len_m = sum(_dist(a, b) for a, b in zip(state.route_pts_m, state.route_pts_m[1:]))

        #print(f"[ROUTING] route (single edge, appended): px={len(state.route_pts_px)}  m={len(state.route_pts_m)}  L={state.route_len_m:.2f}", flush=True)

        # прогресс НЕ сбрасываем
        compute_controls_on_route(state, eps_px=2.0)
        return True

    # 2) строим расширенный граф с 2 «виртуальными» узлами
    nodes_ext = list(nodes_px)
    idx_start = len(nodes_ext); nodes_ext.append([q_s[0], q_s[1]])
    idx_goal  = len(nodes_ext); nodes_ext.append([q_g[0], q_g[1]])

    adj_ext: List[List[Tuple[int, int, float]]] = [list(row) for row in adj]
    adj_ext.extend([[], []])  # для двух виртуальных

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

    old_px = getattr(state, "route_pts_px", None) or []

    if old_px:
        # если сегмент начинается с той же точки, что уже есть в конце — пропускаем дубль
        if pts_px and old_px[-1] == pts_px[0]:
            state.route_pts_px = old_px + pts_px[1:]
        else:
            state.route_pts_px = old_px + pts_px
    else:
        state.route_pts_px = pts_px
    # --- МЕТРИКА: АНИЗО МАСШТАБ ---
    mx = float(getattr(state, "m_per_px_x", 1.0) or 1.0)
    my = float(getattr(state, "m_per_px_y", 1.0) or 1.0)
    new_pts_m = [(x * mx, y * my) for (x, y) in pts_px]
    old_m = getattr(state, "route_pts_m", None) or []

    if old_m:
        if new_pts_m and old_m[-1] == new_pts_m[0]:
            state.route_pts_m = old_m + new_pts_m[1:]
        else:
            state.route_pts_m = old_m + new_pts_m
    else:
        state.route_pts_m = new_pts_m
    state.route_len_m = sum(
    _dist(a, b) for a, b in zip(state.route_pts_m, state.route_pts_m[1:])
)
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


def compute_controls_on_route(state, eps_px: float = 2.0):
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

    from graphics import clear_route_visual
    clear_route_visual(state, also_clear_data=False)  # графика маршрута
    # сами данные мы уже стерли выше
    try:
        idle_view = getattr(state, "_idle_view", None)
        drive_view = getattr(state, "_drive_view", None)
        refresh_all_overlays(state, idle_view, drive_view)
    except Exception:
        pass

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
        jdist = nearest_junction_distance(state, eps_px=15.0)
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



def maybe_snap_robot_to_junction_by_camera(
    state,
    mask: np.ndarray,
    # --- окна поиска перекрёстка ПО МАРШРУТУ (по метрам вдоль polyline) ---
    search_ahead_m: float = 12.0,     # ищем junction в пределах done..done+ahead
    search_behind_m: float = 12.0,    # и done-behind..done (чтобы уметь "снэпнуть назад")
    # --- когда разрешаем снап относительно найденного перекрёстка ---
    min_abs_delta_m: float = 2.0,     # если |s_junc - done| меньше — не снапаем (слишком поздно/шум)
    max_abs_delta_m: float = 25.0,    # если |s_junc - done| больше — не снапаем (слишком рано/далеко)
    # --- куда именно ставим done после детекта перекрёстка ---
    snap_before_m: float = 12.0,      # ставим в точку на polyline: (s_junc - snap_before_m)
    min_move_m: float = 0.3,          # чтобы не делать микро-снэпы
    # --- фильтр "камерой увидели перекрёсток" ---
    thr: float = 0.6,
    side_thr: float = 0.12,
    center_min: float = 0.10,
    cooldown_s: float = 30.0,

    # --- диагностика “не снапнулось” ---
    diag_window_m: float = 12.0,      # если jdist <= diag_window_m и прошли/почти прошли junction — можно логнуть
) -> None:
    """
    Камера говорит "похоже перекрёсток" -> корректируем robot_px/route_done_m вдоль маршрута.

    ВАЖНО:
    - junction берём из state._junc_proj (строится update_junction_turn_hint), т.е. та же база из 10 перекрёстков.
    - ищем ближайший подходящий junction и впереди, и сзади (в пределах окон search_*_m).
    - снап ставит done = s_junc - snap_before_m (до перекрёстка), не в центр.
    """

    import time, bisect

    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    robot   = getattr(state, "robot_px", None)

    if robot is None or mask is None or len(poly_px) < 2 or len(poly_m) < 2:
        return

    now = time.monotonic()
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)

    # ------------------------------------------------------------
    # A) ВСПОМОГАТЕЛЬНО: диагностируем "проехали junction по графу, но snap не случился"
    # ------------------------------------------------------------
    # Условия: есть активный junction из хинта (или ближайший по jdist),
    # done уже "прошёл" s_junc + pass_clear_m (или почти прошёл), но _last_cam_snap_j_id != j_id.
    # Лог делаем 1 раз на junction, чтобы не спамить.
    def _diag_missed(reason: str, details: str = ""):
        j_id = getattr(state, "_active_turn_j_id", None)
        s_j = getattr(state, "_active_turn_s_m", None)
        if j_id is None or s_j is None:
            return
        # один раз на junction
        key = (int(j_id),)
        seen = getattr(state, "_diag_missed_junc", None)
        if not isinstance(seen, set):
            seen = set()
            state._diag_missed_junc = seen
        if key in seen:
            return
        seen.add(key)

        turn = getattr(state, "next_turn_dir", None)
        adeg = float(getattr(state, "turn_deg", 0.0) or 0.0)
        lr = getattr(state, "_cam_junc_seen_lr", None)
        lr_txt = f" cam_lr={lr}" if lr is not None else ""

        print(
            f"[CAM SNAP][MISS] j_id={int(j_id)} turn={turn} deg={adeg:.1f} "
            f"done={done:.1f} s_j={float(s_j):.1f} reason={reason}{lr_txt}"
            + (f" | {details}" if details else ""),
            flush=True
        )

    # Если хинт активен и мы уже реально "перешагнули" junction, но снапа не было — логнем.
    active_j = getattr(state, "_active_turn_j_id", None)
    active_s = getattr(state, "_active_turn_s_m", None)
    if active_j is not None and active_s is not None:
        # Если мы уже прошли по маршруту junction + 1м, а снапа на этот j_id не было
        if done >= float(active_s) + 1.0:
            last_snap_j = getattr(state, "_last_cam_snap_j_id", None)
            if last_snap_j is None or int(last_snap_j) != int(active_j):
                # Причины ниже уточним в зависимости от состояния камеры/кулдауна.
                cam_ts = float(getattr(state, "cam_junc_seen_ts", 0.0) or 0.0)
                cam_ok = (cam_ts > 0.0) and ((now - cam_ts) <= 0.6)
                if not cam_ok:
                    _diag_missed("camera_not_seen_recent", "cam_junc_seen_ts too old or never set")
                else:
                    # камера видела, значит вероятнее всего отфильтровали окна/дельты/cooldown
                    last_ts = float(getattr(state, "_last_cam_snap_ts", 0.0) or 0.0)
                    if (now - last_ts) < float(cooldown_s):
                        _diag_missed("cooldown", f"cooldown_s={cooldown_s:.1f} remaining={cooldown_s-(now-last_ts):.1f}s")
                    else:
                        # могли не найти proj или отсеять по abs_delta
                        proj = getattr(state, "_junc_proj", None) or []
                        if not proj:
                            _diag_missed("no_junc_proj_cache", "update_junction_turn_hint hasn't built _junc_proj yet")
                        else:
                            _diag_missed("filtered_by_windows_or_abs_delta",
                                         f"search_ahead_m={search_ahead_m} search_behind_m={search_behind_m} "
                                         f"min_abs_delta_m={min_abs_delta_m} max_abs_delta_m={max_abs_delta_m}")

    # ------------------------------------------------------------
    # B) cooldown (не чаще 1 раза в cooldown_s)
    # ------------------------------------------------------------
    # твой флажок cam_junc_seen "живёт" 0.6s
    if (now - float(getattr(state, "cam_junc_seen_ts", 0.0) or 0.0)) > 0.6:
        state.cam_junc_seen = False

    last_ts = float(getattr(state, "_last_cam_snap_ts", 0.0) or 0.0)
    if (now - last_ts) < float(cooldown_s):
        return

    # ------------------------------------------------------------
    # C) детект "похоже перекрёсток" по маске: 30% / 40% / 30%
    # ------------------------------------------------------------
    H, W = mask.shape[:2]
    road_bin = (mask > float(thr))

    band_h = max(3, int(H * 0.60))
    band = road_bin[:band_h, :]

    wL = int(W * 0.30)
    wC = int(W * 0.40)
    wR = W - (wL + wC)
    if wR < 1:
        wR = 1
        wC = max(1, W - wL - wR)

    left_band   = band[:, :wL]
    center_band = band[:, wL:wL + wC]
    right_band  = band[:, wL + wC:]

    lf = float(left_band.mean()) if left_band.size else 0.0
    cf = float(center_band.mean()) if center_band.size else 0.0
    rf = float(right_band.mean()) if right_band.size else 0.0

    has_left   = (lf > float(side_thr))
    has_right  = (rf > float(side_thr))
    has_center = (cf > float(center_min))

    if not (has_center and (has_left or has_right)):
        # если по графу уже близко к junction — можно лаконично сообщить почему камера не подтверждает
        jdist = getattr(state, "jdist", None)
        if jdist is not None and float(jdist) <= float(diag_window_m):
            # один раз на активный junction (чтобы не спамить)
            active_j = getattr(state, "_active_turn_j_id", None)
            if active_j is not None:
                key = ("cam_reject", int(active_j))
                seen = getattr(state, "_diag_cam_reject", None)
                if not isinstance(seen, set):
                    seen = set()
                    state._diag_cam_reject = seen
                if key not in seen:
                    seen.add(key)
                    turn = getattr(state, "next_turn_dir", None)
                    adeg = float(getattr(state, "turn_deg", 0.0) or 0.0)
                    # причина “не хватает дороги на стороне поворота”
                    need_side = None
                    if turn == "left":
                        need_side = "left"
                    elif turn == "right":
                        need_side = "right"

                    # сформируем текст причины
                    details = f"lf={lf:.2f} cf={cf:.2f} rf={rf:.2f} thr={thr:.2f} side_thr={side_thr:.2f} center_min={center_min:.2f}"
                    if need_side == "left" and not has_left:
                        _diag_missed("cam_reject_not_enough_left_road", details)
                    elif need_side == "right" and not has_right:
                        _diag_missed("cam_reject_not_enough_right_road", details)
                    elif not has_center:
                        _diag_missed("cam_reject_no_center_road", details)
                    else:
                        _diag_missed("cam_reject", details)
        return

    # камера ВИДИТ перекрёсток
    state.cam_junc_seen = True
    state.cam_junc_seen_ts = now

    # важно для road-follow gating
    state._cam_junc_last_seen_ts = now
    state._cam_junc_seen_lr = (bool(has_left), bool(has_right), bool(has_center))

    # debug (как у тебя)
    state.fturn_l = lf
    state.fturn_c = cf
    state.fturn_r = rf

    # ------------------------------------------------------------
    # D) берём junction-projection из turn-hint (та же база 10 перекрёстков)
    # ------------------------------------------------------------
    proj = getattr(state, "_junc_proj", None) or []
    # ожидаемый формат: (s_m, seg_i, t, j_id, dist_px)
    if not proj:
        # если кэш ещё не построен — снап НЕ делаем
        return

    # ------------------------------------------------------------
    # E) выбираем целевой перекрёсток в окне по маршруту (и впереди, и сзади)
    # ------------------------------------------------------------
    lo = done - float(search_behind_m)
    hi = done + float(search_ahead_m)

    best = None  # (abs_delta, delta, s_junc, seg_i, t, j_id, dist_px)
    for (s_junc, seg_i, t, j_id, dist_px) in proj:
        s_junc = float(s_junc)
        if s_junc < lo or s_junc > hi:
            continue

        delta = s_junc - done
        abs_delta = abs(delta)

        if abs_delta < float(min_abs_delta_m) or abs_delta > float(max_abs_delta_m):
            continue

        if best is None or abs_delta < best[0]:
            best = (abs_delta, delta, s_junc, int(seg_i), float(t), int(j_id), float(dist_px))

    if best is None:
        return

    _, delta, s_junc, seg_i, t, j_id, dist_px = best

    # ------------------------------------------------------------
    # F) не повторяем снап на тот же перекрёсток
    # ------------------------------------------------------------
    last_j = getattr(state, "_last_cam_snap_j_id", None)
    if last_j is not None and int(last_j) == int(j_id):
        return

    # ------------------------------------------------------------
    # G) target_done = s_junc - snap_before_m (ДО перекрёстка)
    # ------------------------------------------------------------
    cum_m = [0.0]
    total = 0.0
    for a, b in zip(poly_m, poly_m[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
        cum_m.append(total)

    target_done = float(s_junc) - float(snap_before_m)
    target_done = max(0.0, min(float(cum_m[-1]), target_done))

    if abs(target_done - done) < float(min_move_m):
        return

    # ------------------------------------------------------------
    # H) интерполируем robot_px по target_done
    # ------------------------------------------------------------
    k = bisect.bisect_right(cum_m, target_done) - 1
    k = max(0, min(len(cum_m) - 2, k))

    seg_len = max(1e-9, float(cum_m[k + 1]) - float(cum_m[k]))
    tt = (target_done - float(cum_m[k])) / seg_len

    ax, ay = poly_px[k]
    bx, by = poly_px[k + 1]
    x = float(ax + tt * (bx - ax))
    y = float(ay + tt * (by - ay))

    # ------------------------------------------------------------
    # I) применяем snap
    # ------------------------------------------------------------
    state.robot_px = (x, y)
    state.route_done_m = float(target_done)
    state.route_progress_idx = int(k)

    state._last_cam_snap_j_id = int(j_id)
    state._last_cam_snap_ts = now

    print(
        f"[CAM SNAP] j_id={j_id}  delta={delta:+.1f}m  target_done={target_done:.1f}m  "
        f"s_junc={s_junc:.1f}m  dist_px={dist_px:.1f}  "
        f"lf={lf:.2f} cf={cf:.2f} rf={rf:.2f}",
        flush=True
    )


def update_junction_turn_hint(
    state,
    announce_dist_m: float = 9.0,
    eps_px: float = 11.0,
    straight_deg: float = 9.2,
    sample_before_m: float = 5.0,
    sample_after_m: float = 5.0,
    pass_margin_m: float = 0.50,
    backtrack_reset_m: float = 1.0,   # если done уменьшился > 1м — считаем, что поехали "назад", можно объявлять заново
) -> None:
    """
    Обновляет:
      state.jdist, state._junc_idx, state.next_turn_dir
      state.fturn — 1 раз на junction (на текущей "сессии движения")
    """

    import time, math, bisect
    now = time.monotonic()

    poly_px = getattr(state, "route_pts_px", None) or []
    poly_m  = getattr(state, "route_pts_m",  None) or []
    junctions = getattr(state, "junctions_px", None) or []

    # базовые сбросы
    state.jdist = None
    state._junc_idx = None
    state.next_turn_dir = None

    if len(poly_px) < 3 or len(poly_px) != len(poly_m) or not junctions:
        return

    # ---------------------------
    # СЕССИЯ ДВИЖЕНИЯ / АНТИДРЕБЕЗГ
    # ---------------------------
    is_running = bool(getattr(state, "is_running", False))
    prev_running = bool(getattr(state, "_turn_prev_running", False))
    state._turn_prev_running = is_running

    # старт новой "сессии" когда is_running: False -> True
    if is_running and not prev_running:
        state._turn_session_id = int(getattr(state, "_turn_session_id", 0) or 0) + 1
        state._announced_junc_ids = set()
        state._turn_last_done_m = float(getattr(state, "route_done_m", 0.0) or 0.0)

    # если не едем — можно не спамить вычислениями (по желанию)
    # но оставим расчёт next_turn_dir/jdist даже на паузе — иногда это удобно
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)

    # backtrack: если done заметно уменьшился — разрешаем объявлять заново
    last_done = float(getattr(state, "_turn_last_done_m", done) or done)
    if done < last_done - float(backtrack_reset_m):
        state._announced_junc_ids = set()
    state._turn_last_done_m = done

    # ---- timeout fturn (как у тебя) ----
    ft = getattr(state, "fturn", None)
    ft_ts = getattr(state, "_fturn_ts", None)
    if ft is not None and ft_ts is not None:
        if (now - float(ft_ts)) > 10.0:
            state.fturn = None
            state._fturn_ts = None

    # ---------------------------
    # cumlen (в метрах вдоль маршрута)
    # ---------------------------
    def _cumlen(points):
        acc = [0.0]
        for a, b in zip(points, points[1:]):
            acc.append(acc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        return acc

    cum_m = _cumlen(poly_m)
    total_m = float(cum_m[-1] if cum_m else 0.0)
    if total_m <= 1e-6:
        return

    # ---------------------------
    # helper: точка на полилинии по s (интерполяция в PX)
    # ---------------------------
    def point_px_at_s(target_s: float):
        target_s = max(0.0, min(float(target_s), total_m))
        i = bisect.bisect_right(cum_m, target_s) - 1
        i = max(0, min(i, len(cum_m) - 2))
        s0 = cum_m[i]
        s1 = cum_m[i + 1]
        seg = max(1e-9, s1 - s0)
        t = (target_s - s0) / seg
        ax, ay = poly_px[i]
        bx, by = poly_px[i + 1]
        return (ax + t * (bx - ax), ay + t * (by - ay)), i, t

    # ---------------------------
    # 1) КЕШ проекций junction -> s_m вдоль маршрута
    # ---------------------------
    def _seg_len(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _point_to_segment_proj_px(a_px, b_px, p_px):
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
            if t < 0.0: t = 0.0
            elif t > 1.0: t = 1.0
            qx, qy = ax + t * vx, ay + t * vy
        dist_px = _seg_len((px, py), (qx, qy))
        return dist_px, t, (qx, qy)

    cache_key = (len(poly_px), round(total_m, 3), len(junctions), round(float(eps_px), 3))
    if getattr(state, "_junc_cache_key", None) != cache_key or not hasattr(state, "_junc_proj"):
        proj = []
        for j_idx, j in enumerate(junctions):
            pt_px = (float(j[0]), float(j[1]))

            best = (float("inf"), None, 0.0)  # dist_px, seg_i, t
            for i, (a_px, b_px) in enumerate(zip(poly_px, poly_px[1:])):
                dist_px, t, _ = _point_to_segment_proj_px(a_px, b_px, pt_px)
                if dist_px < best[0]:
                    best = (dist_px, i, t)

            dist_px, i, t = best
            if i is None:
                continue
            if dist_px > float(eps_px):
                continue

            seg_len_m = math.hypot(
                poly_m[i+1][0] - poly_m[i][0],
                poly_m[i+1][1] - poly_m[i][1],
            )
            s_m = cum_m[i] + t * seg_len_m

            proj.append((float(s_m), int(i), float(t), int(j_idx), float(dist_px)))

        proj.sort(key=lambda x: x[0])
        state._junc_proj = proj
        state._junc_cache_key = cache_key

    proj = getattr(state, "_junc_proj", []) or []
    if not proj:
        return

    # ---------------------------
    # 2) ближайший junction ВПЕРЁД
    # ---------------------------
    s_list = [p[0] for p in proj]
    k = bisect.bisect_right(s_list, done + float(pass_margin_m))
    if k >= len(proj):
        return

    s_j, best_i, best_t, j_idx, dist_px = proj[k]
    jdist = s_j - done
    if jdist <= 0.0:
        return

    state.jdist = float(jdist)
    state._junc_idx = int(best_i)

    # ---------------------------
    # 3) направление по КУРСАМ (atan2) + интерполяция точек по s
    # ---------------------------
    s0 = s_j - float(sample_before_m)
    s2 = s_j + float(sample_after_m)

    p0, _, _ = point_px_at_s(s0)
    p1, _, _ = point_px_at_s(s_j)
    p2, _, _ = point_px_at_s(s2)

    # heading в "мат. координатах": y вверх => dy_screen инвертируем
    def heading(p_from, p_to):
        dx = float(p_to[0] - p_from[0])
        dy = float(p_to[1] - p_from[1])
        return math.atan2(-dy, dx)

    h1 = heading(p0, p1)
    h2 = heading(p1, p2)

    # wrap to [-pi, pi]
    d = h2 - h1
    while d > math.pi:  d -= 2.0 * math.pi
    while d < -math.pi: d += 2.0 * math.pi

    d_deg = math.degrees(d)
    state.turn_deg = d_deg
    if abs(d_deg) <= float(straight_deg):
        turn = "straight"
    else:
        turn = "left" if d_deg > 0 else "right"

    state.next_turn_dir = turn
    if int(j_idx) == 9 and turn == "straight":
        turn = "left"
        state.next_turn_dir = "left"
    # ---------------------------
    # 4) триггер 1 раз на junction за сессию движения (но можно повторно после backtrack/reset)
    # ---------------------------
    if state.jdist <= float(announce_dist_m):
        announced = getattr(state, "_announced_junc_ids", None)
        if not isinstance(announced, set):
            announced = set()
            state._announced_junc_ids = announced

        j_id = int(j_idx)  # твои "10 точек" => стабильный id = индекс в junctions_px

        if j_id not in announced:
            announced.add(j_id)
            state.fturn = turn
            state._fturn_ts = now
            state._active_turn_j_id = j_id
            state._active_turn_s_m  = s_j   # ВАЖНО: это абсолют по маршруту
            #print(f"[TURN HINT] {s_j} = s_j",flush=True)
            # лаконичный лог факта
            print(
                f"[TURN HINT] {turn}  j_id={j_id}  jdist={state.jdist:.1f}m  d={d_deg:.1f}deg  dist_px={dist_px:.1f}",
                flush=True
            )