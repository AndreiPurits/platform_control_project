# graphics.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtGui, QtCore
from robot_cmd import set_safety_stop, get_speed
from status import set_indicator
import math
import os, json, time
import cv2  

Point = Tuple[float, float]

# ---------- маркеры развилок ----------
def _make_junction_dot() -> QtWidgets.QGraphicsEllipseItem:
    r = 4
    item = QtWidgets.QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ffeb3b")))   # жёлтый кружок
    pen = QtGui.QPen(QtGui.QColor("#f59e0b"))
    pen.setWidth(1)
    pen.setCosmetic(True)
    item.setPen(pen)
    item.setZValue(90)
    return item


def redraw_junctions(state, scene: QtWidgets.QGraphicsScene):
    """
    Рисует жёлтые точки развилок (state.junctions_px) поверх карты.
    """
    if scene is getattr(state, "_idle_scene", None):
        # удалить старые
        for it in getattr(state, "idle_junction_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.idle_junction_items = []

        jlist = getattr(state, "junctions_px", []) or []
        for (x, y) in jlist:
            it = _make_junction_dot()
            it.setPos(float(x), float(y))
            scene.addItem(it)
            state.idle_junction_items.append(it)

    elif scene is getattr(state, "_drive_scene", None):
        for it in getattr(state, "drive_junction_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.drive_junction_items = []

        jlist = getattr(state, "junctions_px", []) or []
        for (x, y) in jlist:
            it = _make_junction_dot()
            it.setPos(float(x), float(y))
            scene.addItem(it)
            state.drive_junction_items.append(it)
            
# ---------- сцена и карта ----------
def ensure_scene(view: QtWidgets.QGraphicsView,
                 pixmap: Optional[QtGui.QPixmap] = None) -> QtWidgets.QGraphicsScene:
    sc = view.scene()
    if sc is None:
        sc = QtWidgets.QGraphicsScene()
        view.setScene(sc)
    if pixmap is not None:
        sc.clear()
        sc.addPixmap(pixmap)
    view.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
    return sc


def prepare_view(view: QtWidgets.QGraphicsView):
    if not view:
        return
    view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setDragMode(QtWidgets.QGraphicsView.NoDrag)


def show_map_on_views(png_path: str,
                      idle_view: Optional[QtWidgets.QGraphicsView],
                      drive_view: Optional[QtWidgets.QGraphicsView],
                      state) -> None:
    pm = QtGui.QPixmap(png_path)
    sc_idle = ensure_scene(idle_view, pm) if idle_view else None
    sc_drive = ensure_scene(drive_view, pm) if drive_view else None
    state._idle_scene = sc_idle
    state._drive_scene = sc_drive
    for name in ("idle_route_item", "drive_route_item",
                 "idle_robot_item", "drive_robot_item",
                 "idle_goal_item", "drive_goal_item"):
        if hasattr(state, name):
            setattr(state, name, None)


# ---------- утилиты ----------
def remove_item(scene: QtWidgets.QGraphicsScene, item: Optional[QtWidgets.QGraphicsItem]):
    if scene and item and item.scene() is scene:
        scene.removeItem(item)


# ---------- отрисовка пути ----------
def draw_polyline_path(scene: QtWidgets.QGraphicsScene,
                       old_item: Optional[QtWidgets.QGraphicsItem],
                       pts: List[Point],
                       color: str = "#e53935",
                       width: int = 3) -> Optional[QtWidgets.QGraphicsPathItem]:
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pts:
        return None
    p = QtGui.QPainterPath(QtCore.QPointF(pts[0][0], pts[0][1]))
    for (x, y) in pts[1:]:
        p.lineTo(x, y)
    pen = QtGui.QPen(QtGui.QColor(color))
    pen.setWidth(width)
    pen.setCosmetic(True)
    item = scene.addPath(p, pen)
    item.setZValue(100)
    return item


def redraw_route(state, ui=None) -> None:
    # 1 раз на обновление маршрута
    recompute_route_metrics(state)

    for sc, holder in ((getattr(state, "_idle_scene", None), "idle_route_item"),
                       (getattr(state, "_drive_scene", None), "drive_route_item")):
        if sc is None:
            continue
        item = getattr(state, holder, None)
        new_item = draw_polyline_path(
            sc, item,
            getattr(state, "route_pts_px", []) or [],
            "#e53935", 3
        )
        setattr(state, holder, new_item)


# ---------- маркеры ----------
def make_robot_dot() -> QtWidgets.QGraphicsEllipseItem:
    r = 6
    item = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#00ff00")))
    item.setPen(QtGui.QPen(QtGui.QColor("#006400"), 1))
    item.setZValue(120)
    return item


def draw_robot_dot(scene: QtWidgets.QGraphicsScene,
                   old_item: Optional[QtWidgets.QGraphicsItem],
                   pos: Optional[Point]) -> Optional[QtWidgets.QGraphicsItem]:
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pos:
        return None
    item = make_robot_dot()
    item.setPos(pos[0], pos[1])
    scene.addItem(item)
    return item


def _make_goal_dot() -> QtWidgets.QGraphicsEllipseItem:
    r = 5
    item = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ff3b30")))
    item.setPen(QtGui.QPen(QtGui.QColor("#222"), 1))
    item.setZValue(120)
    return item


def draw_goal_marker(scene: QtWidgets.QGraphicsScene,
                     old_item: Optional[QtWidgets.QGraphicsItem],
                     pos: Optional[Point]) -> Optional[QtWidgets.QGraphicsItem]:
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pos:
        return None
    item = _make_goal_dot()
    item.setPos(pos[0], pos[1])
    scene.addItem(item)
    return item


def draw_flag(scene: QtWidgets.QGraphicsScene, pos_px, color: str = "#1e88e5"):
    if not scene or not pos_px:
        return None
    pen = QtGui.QPen(QtGui.QColor(color))
    pen.setWidth(2)
    pen.setCosmetic(True)
    brush = QtGui.QBrush(QtGui.QColor(color))
    size = 16
    poly = QtGui.QPolygonF([
        QtCore.QPointF(0, -size),
        QtCore.QPointF(size * 0.7, -size * 0.4),
        QtCore.QPointF(0, 0),
    ])
    item = scene.addPolygon(poly, pen, brush)
    item.setPos(pos_px[0], pos_px[1])
    item.setZValue(100)
    return item

# ---------- флаги / столкновение ----------
def check_flags_crossed_and_update(
    state,
    p0_px,
    p1_px,
    eps_px: float = 8.0,
    ui: Optional[QtWidgets.QMainWindow] = None,
):
    """Срабатывание КР-флажков при "скачке" позиции (snap).

    Когда позиция robot_px дискретно корректируется, можно перепрыгнуть флаг.
    Эта функция проверяет пересечение отрезком p0->p1 и применяет все
    пересечённые флаги в правильном порядке (по t вдоль движения).
    """
    if not p0_px or not p1_px:
        return

    try:
        x0, y0 = float(p0_px[0]), float(p0_px[1])
        x1, y1 = float(p1_px[0]), float(p1_px[1])
    except Exception:
        return

    # Список флагов и их типов
    flags = list(getattr(state, "control_pts_px", []))
    kinds = list(getattr(state, "control_pts_kind", []))
    if not flags:
        return

    vx = x1 - x0
    vy = y1 - y0
    vv = vx * vx + vy * vy
    if vv <= 1e-9:
        # почти нет движения -> смысла проверять "пересечение" нет
        return

    eps2 = float(eps_px) * float(eps_px)

    crossed = []  # (t, idx)
    for i, pt in enumerate(flags):
        try:
            px, py = float(pt[0]), float(pt[1])
        except Exception:
            continue

        # проекция точки на отрезок, параметр t в [0..1]
        wx = px - x0
        wy = py - y0
        t = (wx * vx + wy * vy) / vv
        if t < 0.0:
            t_clamped = 0.0
        elif t > 1.0:
            t_clamped = 1.0
        else:
            t_clamped = t

        # расстояние от точки до ближайшей точки на отрезке
        cx = x0 + t_clamped * vx
        cy = y0 + t_clamped * vy
        dx = px - cx
        dy = py - cy
        d2 = dx * dx + dy * dy

        if d2 <= eps2 and (0.0 <= t_clamped <= 1.0):
            crossed.append((t_clamped, i))

    if not crossed:
        return

    # Применяем в порядке движения
    crossed.sort(key=lambda z: z[0])

    # текущие PWM (L/R берём из state; B будем менять)
    l = int(getattr(state, "l_pwm", 1500) or 1500)
    r = int(getattr(state, "r_pwm", 1500) or 1500)

    applied_any = False
    to_remove = set()

    for t, idx in crossed:
        kind = kinds[idx] if idx < len(kinds) else None
        pt = flags[idx]

        # вычисляем PWM для B (как в check_flag_collision_and_update)
        if kind == "b_on":
            b_pwm = int(getattr(state, "b_on_pwm", 2000) or 2000)
        elif kind == "b_off":
            b_pwm = int(getattr(state, "b_off_pwm", 1500) or 1500)
        else:
            b_pwm = int(getattr(state, "b_pwm", 1500) or 1500)

        state.b_pwm = b_pwm

        try:
            from robot_cmd import motors_set
            motors_set(state, l, r, b_pwm)
            print(f"[FLAG-X] crossed t={t:.3f} idx={idx} pt={pt} kind={kind!r} -> B={b_pwm}", flush=True)
        except Exception as e:
            print("[FLAG-X] motors_set B error:", e, flush=True)

        applied_any = True
        to_remove.add(idx)

    if not applied_any:
        return

    # Удаляем сработавшие флаги (перестраиваем списки)
    new_flags = []
    new_kinds = []
    for i, pt in enumerate(flags):
        if i in to_remove:
            continue
        new_flags.append(pt)
        if i < len(kinds):
            new_kinds.append(kinds[i])

    state.control_pts_px = new_flags
    state.control_pts_kind = new_kinds

    # Перерисовать маркеры, чтобы флаги исчезли
    try:
        from graphics import redraw_markers
    except Exception:
        redraw_markers = None

    if redraw_markers:
        for sc in (
            getattr(state, "_idle_scene", None),
            getattr(state, "_drive_scene", None),
        ):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass

    if ui is not None and ui.statusBar():
        ui.statusBar().showMessage("Сработал(и) флажок(и) КР (snap)", 1200)
def check_flag_collision_and_update(
    state,
    eps_px: float = 8.0,
    ui: Optional[QtWidgets.QMainWindow] = None,
):
    rp = getattr(state, "robot_px", None)
    if not rp:
        return

    rx, ry = float(rp[0]), float(rp[1])

    flags = list(getattr(state, "control_pts_px", []))
    kinds = list(getattr(state, "control_pts_kind", []))

    if not flags:
        return

    heading = float(getattr(state, "robot_heading_rad", 0.0) or 0.0)
    cos_h, sin_h = math.cos(heading), math.sin(heading)

    front_boost = 1.4
    back_cutoff = 0.7

    best_idx = None
    best_dist2 = None

    # --- ищем ОДИН ближайший флаг в радиусе ---
    for i, pt in enumerate(flags):
        px, py = float(pt[0]), float(pt[1])
        dx = px - rx
        dy = py - ry
        dist2 = dx * dx + dy * dy

        forward_proj = dx * cos_h + dy * sin_h
        if forward_proj >= 0:
            r2 = (eps_px * front_boost) ** 2
        else:
            r2 = (eps_px * back_cutoff) ** 2

        if dist2 <= r2:
            if best_idx is None or dist2 < best_dist2:
                best_idx = i
                best_dist2 = dist2

    if best_idx is None:
        return  # ни один флаг не зацепили

    # --- сработал один флажок ---
    pt = flags[best_idx]
    kind = kinds[best_idx] if best_idx < len(kinds) else None
    print(f"[FLAG] reached #{best_idx} {pt} kind={kind!r}", flush=True)

    # Удаляем точку и её цвет по одному и тому же индексу
    del flags[best_idx]
    if best_idx < len(kinds):
        del kinds[best_idx]

    state.control_pts_px = flags
    state.control_pts_kind = kinds

    # --- вычисляем PWM для B ---
    if kind == "b_on":
        b_pwm = int(getattr(state, "b_on_pwm", 2000) or 2000)
    elif kind == "b_off":
        b_pwm = int(getattr(state, "b_off_pwm", 1500) or 1500)
    else:
        # если вдруг kind нет, оставляем текущее значение
        b_pwm = int(getattr(state, "b_pwm", 1500) or 1500)

    l = int(getattr(state, "l_pwm", 1500) or 1500)
    r = int(getattr(state, "r_pwm", 1500) or 1500)
    state.b_pwm = b_pwm

    try:
        from robot_cmd import motors_set
        motors_set(state, l, r, b_pwm)
        print(f"[FLAG] apply B={b_pwm} (L={l}, R={r})", flush=True)
    except Exception as e:
        print("[FLAG] motors_set B error:", e, flush=True)

    # Перерисовать маркеры, чтобы один флаг исчез, остальные остались со своими цветами
    try:
        from graphics import redraw_markers
    except Exception:
        redraw_markers = None

    if redraw_markers:
        for sc in (
            getattr(state, "_idle_scene", None),
            getattr(state, "_drive_scene", None),
        ):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass

    if ui is not None and ui.statusBar():
        ui.statusBar().showMessage("Сработал флажок КР", 1200)

def redraw_markers(state, scene: QtWidgets.QGraphicsScene):
    """
    Перерисовка маркеров (робот, цель, КР-флажки) для IDLE и DRIVE.

    ВАЖНО:
      - Цвет флажков КР берём ТОЛЬКО из state.control_pts_kind.
      - После старта движения (is_running/trajectory_mode) НИКОГДА
        не пересчитываем цвета, только отображаем то, что уже записано.
    """
    ctrl_pts = list(getattr(state, "control_pts_px", []) or [])
    kinds    = list(getattr(state, "control_pts_kind", []) or [])

    running = bool(getattr(state, "is_running", False)) or bool(
        getattr(state, "trajectory_mode", False)
    )

    # --- РЕДАКТОРСКАЯ ФАЗА (до старта движения) ---
    # Тут разрешаем автозаполнение kinds в стиле: зелёный, красный, зелёный, красный...
    if not running:
        if not kinds and ctrl_pts:
            # если ещё нет kinds, создаём с чередованием
            kinds = []
            next_on = True  # первый всегда b_on (зелёный)
            for _ in ctrl_pts:
                kinds.append("b_on" if next_on else "b_off")
                next_on = not next_on
            state.control_pts_kind = kinds
        else:
            # Длины немного расъехались — аккуратно добиваем хвост (в редакторском режиме это ок)
            if len(kinds) < len(ctrl_pts):
                # продолжаем чередование от последнего известного состояния
                if kinds:
                    last = kinds[-1]
                    next_on = (last != "b_on")
                else:
                    next_on = True
                for _ in range(len(ctrl_pts) - len(kinds)):
                    kinds.append("b_on" if next_on else "b_off")
                    next_on = not next_on
                state.control_pts_kind = kinds
            elif len(kinds) > len(ctrl_pts):
                # обрезаем лишние, если вдруг есть
                kinds = kinds[:len(ctrl_pts)]
                state.control_pts_kind = kinds

    # --- ФАЗА ДВИЖЕНИЯ ---
    # Здесь НИЧЕГО не пересчитываем для цветов.
    # Единственное, что допустимо — длина kinds не должна быть больше, чем точек.
    else:
        if len(kinds) > len(ctrl_pts):
            kinds = kinds[:len(ctrl_pts)]
            state.control_pts_kind = kinds

    def _color_for_idx(i: int) -> str:
        kind = None
        if 0 <= i < len(state.control_pts_kind or []):
            kind = state.control_pts_kind[i]

        if kind == "b_off":
            return "#e53935"  # красный — выключение B (1500)
        elif kind == "b_on":
            return "#43a047"  # зелёный — включение B (2000)
        else:
            return "#1e88e5"  # синий — дефолт (если вдруг нет информации)

    # ---------- IDLE-СЦЕНА ----------
    if scene is getattr(state, "_idle_scene", None):
        # Робот
        state.idle_robot_item = draw_robot_dot(
            scene,
            getattr(state, "idle_robot_item", None),
            getattr(state, "robot_px", None),
        )

        # Цель
        state.idle_goal_item = draw_goal_marker(
            scene,
            getattr(state, "idle_goal_item", None),
            getattr(state, "goal_px", None),
        )

        # КР-флажки: удалить старые
        for it in getattr(state, "idle_control_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.idle_control_items = []

        # Нарисовать заново с закреплёнными цветами
        for i, pt in enumerate(ctrl_pts):
            color = _color_for_idx(i)
            it = draw_flag(scene, pt, color=color)
            if it:
                state.idle_control_items.append(it)

    # ---------- DRIVE-СЦЕНА ----------
    elif scene is getattr(state, "_drive_scene", None):
        # Робот
        state.drive_robot_item = draw_robot_dot(
            scene,
            getattr(state, "drive_robot_item", None),
            getattr(state, "robot_px", None),
        )

        # Цель
        state.drive_goal_item = draw_goal_marker(
            scene,
            getattr(state, "drive_goal_item", None),
            getattr(state, "goal_px", None),
        )

        # КР-флажки: удалить старые
        for it in getattr(state, "drive_control_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.drive_control_items = []

        # Нарисовать заново
        for i, pt in enumerate(ctrl_pts):
            color = _color_for_idx(i)
            it = draw_flag(scene, pt, color=color)
            if it:
                state.drive_control_items.append(it)

def redraw_radar_points(scene: QtWidgets.QGraphicsScene,
                        pts_m,
                        meters_to_px: float = 80.0,
                        state=None):
    if not scene:
        return

    old = getattr(state, "_radar_items", None) if state else None
    if old:
        for it in old:
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
    if state:
        state._radar_items = []

    items = []
    for (x, y) in pts_m:
        xp = x * meters_to_px
        yp = -y * meters_to_px
        dot = QtWidgets.QGraphicsEllipseItem(xp - 1, yp - 1, 2, 2)
        dot.setBrush(QtGui.QBrush(QtGui.QColor("#00bcd4")))
        dot.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        dot.setZValue(50)
        scene.addItem(dot)
        items.append(dot)

    if state is not None:
        state._radar_items = items


# ===================== АНИМАЦИЯ МАРШРУТА =====================
def reset_route_progress(state):
    state.route_done_m = 0.0
    state._route_cum_m = None


LIDAR_SNAP_THRESHOLD = 0.85
LIDAR_SNAP_PERCENT = 0.10
LIDAR_SNAP_STEP_PX_DEF = 3.0

# graphics.py
import math
from PyQt5 import QtWidgets
from status import set_indicator

def _lidar_check(pts, half_deg, R, Rmin, need_n, yaw0_rad) -> bool:
    half_rad = math.radians(max(0.0, min(179.9, half_deg)))
    R2 = R * R
    Rmin2 = Rmin * Rmin

    near_cnt = 0

    if yaw0_rad != 0.0:
        cy, sy = math.cos(yaw0_rad), math.sin(yaw0_rad)
        it = ((x * cy - y * sy, x * sy + y * cy) for (x, y) in pts)
    else:
        it = pts

    for (xr, yr) in it:
        d2 = xr * xr + yr * yr
        if d2 < Rmin2 or d2 > R2:
            continue
        ang = math.atan2(yr, xr)
        if abs(ang) > half_rad:
            continue
        near_cnt += 1
        if near_cnt >= need_n:
            break

    return near_cnt >= need_n

from collections import defaultdict

def lidar_sector_solidness(
    pts,
    half_deg: float,
    R: float,
    Rmin: float,
    yaw0_rad: float = 0.0,       # если у лидара есть монтажный yaw (как у тебя)
    angle_bin_deg: float = 1.0,  # 1°–2° обычно ок
):
    """
    Возвращает solidness ∈ [0..1] внутри сектора [-half_deg..+half_deg]
    вокруг оси yaw0_rad (в радианах).
    Считаем "занятость" угловых бинов: есть ли хотя бы одно попадание в (Rmin..R).
    """
    if not pts:
        return 0.0

    bins = defaultdict(int)

    for x, y in pts:
        r = math.hypot(x, y)
        if r < Rmin or r > R:
            continue

        # угол точки в градусах относительно оси лидара
        ang = math.degrees(math.atan2(y, x) - yaw0_rad)

        # нормализация в [-180..180] (на всякий)
        while ang > 180:
            ang -= 360
        while ang < -180:
            ang += 360

        if ang < -half_deg or ang > half_deg:
            continue

        bin_id = int((ang + half_deg) // angle_bin_deg)  # 0..N-1 внутри сектора
        bins[bin_id] += 1

    sector_bins = max(1, int((2.0 * half_deg) / angle_bin_deg))
    occupied_bins = len(bins)

    solidness = occupied_bins / float(sector_bins)
    return max(0.0, min(1.0, solidness))
# -*- coding: utf-8 -*-
import numpy as np


def _front_sector_ranges(pts, *, half_deg: float, R: float, Rmin: float, yaw0_rad: float):
    """
    pts: list[(x_m,y_m)] в координатах лидара/робота.
    yaw0_rad: компенсируем поворот лидара (если нужно).
    Возвращает список дальностей r (м) в секторе +/- half_deg и диапазоне (Rmin, R].
    """
    if not pts:
        return []

    half = float(half_deg) * math.pi / 180.0
    out = []
    for (x, y) in pts:
        x = float(x); y = float(y)
        r = math.hypot(x, y)
        if r <= float(Rmin) or r > float(R):
            continue

        a = math.atan2(y, x) - float(yaw0_rad)
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        if abs(a) > half:
            continue

        out.append(r)
    return out


def _adaptive_cluster_max_frac(ranges_m, *, rel: float = 0.06, w_min: float = 0.03):
    """
    1D кластеризация по дальности:
      - сортируем r
      - объединяем если соседние <= max(w_min, rel*r_ref)
    Возвращает (max_cluster_frac, max_cluster_cnt, n_total)
    """
    n = len(ranges_m)
    if n == 0:
        return 0.0, 0, 0

    rs = np.sort(np.asarray(ranges_m, dtype=np.float32))
    max_cnt = 1
    cnt = 1
    r_ref = float(rs[0])

    for i in range(1, n):
        thr_w = max(float(w_min), float(rel) * max(0.01, r_ref))
        if float(rs[i]) - float(rs[i - 1]) <= thr_w:
            cnt += 1
        else:
            if cnt > max_cnt:
                max_cnt = cnt
            cnt = 1
            r_ref = float(rs[i])

    if cnt > max_cnt:
        max_cnt = cnt

    return float(max_cnt) / float(n), int(max_cnt), int(n)


def _hard_object_guard(ranges_m, *, hard_r: float = 1.15, hard_min_hits: int = 3):
    """
    Защита от твёрдого объекта при низкой общей плотности:
    если есть компактный кластер ближе hard_r (например ствол).
    """
    near = [r for r in ranges_m if float(r) <= float(hard_r)]
    if len(near) < int(hard_min_hits):
        return False, 0
    _, cnt, _ = _adaptive_cluster_max_frac(near, rel=0.02, w_min=0.02)
    return (cnt >= int(hard_min_hits)), int(cnt)


def lidar_front_stop(ui, state) -> bool:
    pts = getattr(state, "_lidar_front_last_pts", None) or []

    half_deg = float(getattr(state, "lidar_front_sector_half_deg", 15.0) or 15.0)
    R        = float(getattr(state, "lidar_front_stop_distance_m", 1.8) or 1.8)   # <= 1.8
    Rmin     = float(getattr(state, "lidar_front_ignore_radius_m", 0.7) or 0.7)  # ковш
    yaw0     = float(getattr(state, "lidar_front_mount_yaw_rad", 0.0) or 0.0)    # <-- проверь, реально ли нужен

    # СЛАЙДЕР: это и есть порог плотности (0.20..0.98)
    thr = float(getattr(state, "lidar_stop_solidness", 0.70) or 0.70)
    thr = 0.20 if thr < 0.20 else 0.98 if thr > 0.98 else thr

    # параметры адаптивного окна (тоже можно крутить)
    rel   = float(getattr(state, "lidar_density_rel", 0.06) or 0.06)
    w_min = float(getattr(state, "lidar_density_w_min", 0.03) or 0.03)
    rel   = 0.01 if rel < 0.01 else 0.20 if rel > 0.20 else rel
    w_min = 0.005 if w_min < 0.005 else 0.10 if w_min > 0.10 else w_min

    ranges = _front_sector_ranges(pts, half_deg=half_deg, R=R, Rmin=Rmin, yaw0_rad=yaw0)

    solid, max_cnt, n = _adaptive_cluster_max_frac(ranges, rel=rel, w_min=w_min)

    # плотность: требуем хотя бы минимум лучей, иначе шум
    n_min = int(getattr(state, "lidar_density_min_n", 6) or 6)
    n_min = 3 if n_min < 3 else 50 if n_min > 50 else n_min
    density_stop = (n >= n_min) and (solid >= thr)
    # hard-object guard (ствол/камень)
    hard_r = float(getattr(state, "lidar_hard_r", 1.15) or 1.15)
    hard_r = 0.20 if hard_r < 0.20 else R if hard_r > R else hard_r
    hard_min_hits = int(getattr(state, "lidar_hard_min_hits", 3) or 3)
    hard_min_hits = 2 if hard_min_hits < 2 else 12 if hard_min_hits > 12 else hard_min_hits
    hard_stop, hard_cnt = _hard_object_guard(ranges, hard_r=hard_r, hard_min_hits=hard_min_hits)

    stop = bool(density_stop or hard_stop)

    prev = bool(getattr(state, "safety_stop_front", False))
    state.safety_stop_front = stop

    # для HUD/дебага
    state.lidar_front_solidness = float(solid)
    state._lidar_front_n = int(n)
    state._lidar_front_cluster_cnt = int(max_cnt)
    state._lidar_front_thr = float(thr)
    state._lidar_front_hard = bool(hard_stop)
    state._lidar_front_hard_cnt = int(hard_cnt)

    # лампа переднего лидара
    try:
        indF = ui.findChild(QtWidgets.QLabel, "indLidar")
        if indF:
            set_indicator(indF, "bad" if stop else "ok")
    except Exception:
        pass

    # объединённый стоп (front+rear и т.п.) — твоя функция
    _apply_combined_safety(ui, state)

    if stop and not prev:
        print(f"[LIDAR FRONT STOP] solid={solid:.2f} thr={thr:.2f} n={n} cluster={max_cnt} hard={int(hard_stop)}", flush=True)
    if (not stop) and prev:
        print("[LIDAR FRONT STOP] cleared", flush=True)

    return stop

def lidar_rear_stop(ui, state) -> bool:
    
    # если заднего лидара нет — всегда зелёная лампа и нет стопа
    if not getattr(state, "has_rear_lidar", False):
        try:
            indR = ui.findChild(QtWidgets.QLabel, "indLidarRear")
            if indR:
                set_indicator(indR, "ok")
        except Exception:
            pass
        state.safety_stop_rear = False
        _apply_combined_safety(ui, state)
        return False

    pts = getattr(state, "_lidar_rear_last_pts", None) or []

    stop = _lidar_check(
        pts=pts,
        half_deg=float(getattr(state, "lidar_rear_sector_half_deg", 25.0) or 25.0),
        R=float(getattr(state, "lidar_rear_stop_distance_m", 1) or 1),
        Rmin=float(getattr(state, "lidar_rear_ignore_radius_m", 0.01) or 0.01),
        need_n=int(getattr(state, "lidar_rear_stop_min_points", 3) or 3),
        yaw0_rad=float(getattr(state, "lidar_rear_mount_yaw_rad", 3.0) or 3.0),
    )

    prev = bool(getattr(state, "safety_stop_rear", False))
    state.safety_stop_rear = stop

    # Лампа заднего лидара
    try:
        indR = ui.findChild(QtWidgets.QLabel, "indLidarRear")
        if indR:
            set_indicator(indR, "bad" if stop else "ok")
    except Exception:
        pass

    _apply_combined_safety(ui, state)

    if stop and not prev:
        print("[LIDAR REAR STOP] triggered", flush=True)
    if (not stop) and prev:
        print("[LIDAR REAR STOP] cleared", flush=True)

    return stop

def _lidar(ui, state):
    return lidar_front_stop(ui, state)

def recompute_route_metrics(state):
    pts_px = getattr(state, "route_pts_px", None) or []
    if len(pts_px) < 2:
        state.route_len_px = 0.0
        state.route_len_m = 0.0
        state.route_cum_px = [0.0]
        state.alpha_m_per_px = 1.0
        return False

    Lpx = 0.0
    cum = [0.0]
    for a, b in zip(pts_px, pts_px[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        Lpx += d
        cum.append(Lpx)

    mx = float(getattr(state, "m_per_px_x", 0.0) or 0.0)
    my = float(getattr(state, "m_per_px_y", 0.0) or 0.0)
    if mx > 0.0 and my > 0.0:
        Lm = 0.0
        for a, b in zip(pts_px, pts_px[1:]):
            dx, dy = (b[0] - a[0]), (b[1] - a[1])
            Lm += math.hypot(dx * mx, dy * my)
    else:
        pts_m = getattr(state, "route_pts_m", None) or []
        if len(pts_m) == len(pts_px) and len(pts_m) >= 2:
            Lm = 0.0
            for a, b in zip(pts_m, pts_m[1:]):
                Lm += math.hypot(b[0] - a[0], b[1] - a[1])
        else:
            mpp = float(getattr(state, "meters_per_pixel", 0.0) or 0.0)
            Lm = Lpx * (mpp if mpp > 0.0 else 1.0)

    state.route_len_px = Lpx
    state.route_len_m = Lm
    state.route_cum_px = cum
    state.alpha_m_per_px = (Lm / Lpx) if Lpx > 1e-9 else 1.0
    state._carry_step_px = float(getattr(state, "_carry_step_px", 0.0) or 0.0)
    return True


# --------- старт анимации маршрута (движение по метрам) ----------


def start_route_animation(ui: QtWidgets.QMainWindow, state, fps: int = 30):
    pts = getattr(state, "route_pts_px", None) or []
    if len(pts) < 2 or float(getattr(state, "route_len_m", 0.0) or 0.0) <= 0.0:
        print("[ANIM] нет маршрута", flush=True)
        return False

    if not hasattr(state, "_route_timer") or state._route_timer is None:
        state._route_timer = QtCore.QTimer(ui)
    state._route_timer.setInterval(max(1, int(1000 / fps)))

    Lm = float(state.route_len_m)
    Lpx = float(state.route_len_px)
    if getattr(state, "route_done_m", 0.0) > Lm:
        state.route_done_m = Lm
    if getattr(state, "route_progress_idx", 0) >= len(pts) - 1:
        state.route_progress_idx = len(pts) - 2

    state._last_route_tick_ts = time.time()
    state._carry_step_px = float(getattr(state, "_carry_step_px", 0.0) or 0.0)

    def _interp_by_s_px(s_px: float):
        cum = state.route_cum_px
        i = 0
        while i < len(cum) - 1 and cum[i + 1] < s_px:
            i += 1
        i = min(i, len(pts) - 2)
        seg_len = max(1e-9, cum[i + 1] - cum[i])
        t = (s_px - cum[i]) / seg_len
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        return i, ax + t * (bx - ax), ay + t * (by - ay)

    def _tick():
        try:
            from robot_cmd import update_drive_panel, note_robot_pose
        except Exception:
            update_drive_panel = None
            note_robot_pose = None

        base_v = float(getattr(state, "speed_mps", 0.0) or 0.0)

        lidar_stop = False
        try:
            lidar_stop = bool(_lidar(ui, state))
        except Exception as e:
            print("[ANIM] lidar check error:", e, flush=True)
            lidar_stop = False

        road_block = False  # TODO: road_guard сюда же, когда включишь

        if lidar_stop or road_block:
            v = 0.0
        else:
            v = base_v

        if v <= 1e-6:
            for sc in (getattr(state, "_idle_scene", None),
                       getattr(state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(state, sc)
                    except Exception:
                        pass
            if update_drive_panel:
                try:
                    update_drive_panel(ui, state)
                except Exception:
                    pass
            return

        now = time.time()
        last = float(getattr(state, "_last_route_tick_ts", now) or now)
        dt = now - last
        if dt <= 0:
            dt = state._route_timer.interval() / 1000.0
        dt = max(1e-3, min(dt, 0.2))
        state._last_route_tick_ts = now

        alpha = float(getattr(state, "alpha_m_per_px", 1.0) or 1.0)

        step_px = (v * dt) / alpha + float(getattr(state, "_carry_step_px", 0.0) or 0.0)

        done_m_old = float(getattr(state, "route_done_m", 0.0) or 0.0)
        s_px_old = (done_m_old / Lm) * Lpx
        s_px_new = s_px_old + max(0.0, step_px)

        if s_px_new >= Lpx - 1e-6:
            state.route_done_m = Lm
            state.route_progress_idx = len(pts) - 2
            state.robot_px = pts[-1]
            state.robot_heading_rad = math.atan2(pts[-1][1] - pts[-2][1],
                                                 pts[-1][0] - pts[-2][0])
            try:
                if state._route_timer.isActive():
                    state._route_timer.stop()
            except Exception:
                pass
            state.is_running = False
            btn = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
            if btn:
                btn.setChecked(False)
                btn.setText("Пуск")
            from routing import clear_goals_and_route
            clear_goals_and_route(state)
        else:
            i, x, y = _interp_by_s_px(s_px_new)
            state.route_progress_idx = i
            state.robot_px = (x, y)
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            state.robot_heading_rad = math.atan2(by - ay, bx - ax)
            state.route_done_m = min(Lm, done_m_old + v * dt)
            exact_step_px = (state.route_done_m / Lm) * Lpx - s_px_old
            state._carry_step_px = (step_px - exact_step_px)

        if note_robot_pose:
            try:
                note_robot_pose(state)
            except Exception as e:
                print("[ROBOT] note_robot_pose error:", e, flush=True)

        try:
            update_visited_track(ui, state)
            dataset_maybe_capture(ui, state)
        except Exception as e:
            print("[ANIM] aux error:", e, flush=True)

        for sc in (getattr(state, "_idle_scene", None),
                   getattr(state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass
        if update_drive_panel:
            try:
                update_drive_panel(ui, state)
            except Exception:
                pass

    try:
        state._route_timer.timeout.disconnect()
    except (TypeError, RuntimeError):
        pass
    state._route_timer.timeout.connect(_tick)
    if not state._route_timer.isActive():
        state._route_timer.start()
    return True


def stop_route_animation(state, keep_progress: bool = True):
    try:
        if hasattr(state, "_route_timer") and state._route_timer and state._route_timer.isActive():
            state._route_timer.stop()
    except Exception:
        pass
    if not keep_progress:
        state.route_progress_idx = 0
        state.route_seg_off_m = 0.0


def clear_route_visual(state, also_clear_data: bool = True):
    for attr_item, attr_scene in (("idle_route_item", "_idle_scene"),
                                  ("drive_route_item", "_drive_scene")):
        item = getattr(state, attr_item, None)
        sc = getattr(state, attr_scene, None)
        if item and sc and item.scene() is sc:
            try:
                sc.removeItem(item)
            except Exception:
                pass
        setattr(state, attr_item, None)

    if also_clear_data:
        state.route_pts_px = []
        state.route_pts_m = []
        state.route_len_m = 0.0
        state.route_done_m = 0.0
    state.route_finished = True


def update_visited_track(ui, state, min_seg_px: float = 2.0):
    if not bool(getattr(state, "dataset_mode", False)):
        return
    rp = getattr(state, "robot_px", None)
    if not rp:
        return
    pts = state.visited_path_px
    if pts:
        dx = rp[0] - pts[-1][0]
        dy = rp[1] - pts[-1][1]
        if (dx * dx + dy * dy) < (min_seg_px * min_seg_px):
            return
    pts.append((float(rp[0]), float(rp[1])))

    sc = getattr(state, "_drive_scene", None)
    if sc is None:
        return
    path = QtGui.QPainterPath()
    if not pts:
        return
    path.moveTo(pts[0][0], pts[0][1])
    for x, y in pts[1:]:
        path.lineTo(x, y)
    if state.drive_visited_item is None:
        item = QtWidgets.QGraphicsPathItem(path)
        pen = QtGui.QPen(QtGui.QColor("#22aa22"))
        pen.setWidthF(2.0)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setZValue(3)
        sc.addItem(item)
        state.drive_visited_item = item
    else:
        state.drive_visited_item.setPath(path)


# ==== dataset capture (камера + лидар) ====

def _dataset_dirs(state):
    map_path = getattr(state, "active_map_path", None) or "map"
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    root = getattr(state, "dataset_root", os.path.expanduser("~/datasets"))
    photos_dir = os.path.join(root, "photos", map_name)
    lidar_dir = os.path.join(root, "lidar", map_name)
    os.makedirs(photos_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)
    return photos_dir, lidar_dir


def _save_camera_frame(state, x_px, y_px):
    cap = getattr(state, "_cam", None)
    if cap is None or not cap.isOpened():
        return None

    ok, frame = cap.read()
    if not ok or frame is None:
        return None

    photos_dir, _ = _dataset_dirs(state)
    fname = f"{int(x_px)}_{int(y_px)}.png"
    path = os.path.join(photos_dir, fname)

    try:
        if cv2.imwrite(path, frame):
            return os.path.abspath(path)
    except Exception as e:
        print("[CAMERA SAVE] error:", e, flush=True)
    return None

def _ensure_dataset_dirs(state):
    root = getattr(state, "dataset_root", os.path.expanduser("~/datasets")) or os.path.expanduser("~/datasets")
    map_path = getattr(state, "active_map_path", None)
    if not map_path:
        map_name = "unnamed_map"
    else:
        map_name = os.path.splitext(os.path.basename(map_path))[0]
    state.map_name = map_name

    photos_dir = os.path.join(root, "photos", map_name)
    lidar_dir = os.path.join(root, "lidar", map_name)
    os.makedirs(photos_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)

    state.dataset_photos_dir = photos_dir
    state.dataset_lidar_dir = lidar_dir
    return photos_dir, lidar_dir


def _save_lidar_snapshot(state, x_px, y_px):
    pts = getattr(state, "_lidar_last_pts", None) or []
    if not pts:
        return None

    now_ms = int(time.time() * 1000)
    last_ts = int(getattr(state, "_lidar_last_ts", 0))
    if now_ms - last_ts > 1500:
        return None

    _, lidar_dir = _ensure_dataset_dirs(state)
    base = f"{int(x_px)}_{int(y_px)}"
    json_path = os.path.join(lidar_dir, base + ".json")
    png_path = os.path.join(lidar_dir, base + ".png")

    try:
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(
                {"px": [int(x_px), int(y_px)], "ts_ms": now_ms, "pts": pts},
                f, ensure_ascii=False, separators=(",", ":")
            )
    except Exception:
        return None

    try:
        size = int(getattr(state, "lidar_png_size_px", 400) or 400)
        rmax = float(getattr(state, "lidar_png_radius_m", 6.0) or 6.0)

        img = QtGui.QImage(size, size, QtGui.QImage.Format_RGB32)
        img.fill(QtGui.QColor("#0d1117"))

        qp = QtGui.QPainter(img)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)

        pen_axes = QtGui.QPen(QtGui.QColor("#334155"))
        pen_axes.setWidth(1)
        pen_axes.setCosmetic(True)
        cx = size * 0.5
        cy = size * 0.5
        qp.setPen(pen_axes)
        qp.drawLine(0, int(cy), size, int(cy))
        qp.drawLine(int(cx), 0, int(cx), size)

        pen_grid = QtGui.QPen(QtGui.QColor("#1f2937"))
        pen_grid.setWidth(1)
        pen_grid.setCosmetic(True)
        qp.setPen(pen_grid)
        ppm = (size * 0.5 - 4.0) / max(0.1, rmax)
        m = 1.0
        while m <= rmax + 1e-9:
            rad = m * ppm
            qp.drawEllipse(QtCore.QPointF(cx, cy), rad, rad)
            m += 1.0

        pen_pts = QtGui.QPen(QtGui.QColor("#00bcd4"))
        pen_pts.setWidth(0)
        pen_pts.setCosmetic(True)
        brush = QtGui.QBrush(QtGui.QColor("#00bcd4"))
        qp.setPen(pen_pts)
        qp.setBrush(brush)
        d = 2.0
        for (xm, ym) in pts:
            xp = cx + xm * ppm
            yp = cy - ym * ppm
            qp.drawEllipse(QtCore.QRectF(xp - d, yp - d, 2 * d, 2 * d))

        qp.end()

        if not img.save(png_path, "PNG"):
            png_path = None
    except Exception:
        png_path = None

    return (os.path.abspath(json_path), os.path.abspath(png_path) if png_path else None)


def dataset_maybe_capture(ui: QtWidgets.QMainWindow, state):
    if not getattr(state, "dataset_mode", False):
        return
    if not getattr(state, "is_running", False):
        return

    Lm = float(getattr(state, "route_len_m", 0.0) or 0.0)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)
    if Lm <= 0.0:
        return

    step = float(getattr(state, "dataset_step_m", 3.0) or 3.0)
    last = float(getattr(state, "dataset_last_snap_m", 0.0) or 0.0)
    if (done - last) < step:
        return

    rp = getattr(state, "robot_px", None)
    if not rp:
        return
    x_px, y_px = float(rp[0]), float(rp[1])

    _ensure_dataset_dirs(state)

    saved_parts = []

    lid = _save_lidar_snapshot(state, x_px, y_px)
    if lid:
        jp, pp = lid
        if jp:
            saved_parts.append(f"lidar-json -> {jp}")
        if pp:
            saved_parts.append(f"lidar-png  -> {pp}")

    cam = _save_camera_frame(state, x_px, y_px)
    if cam:
        saved_parts.append(f"photo -> {cam}")

    if saved_parts:
        state.dataset_last_snap_m = done
        print(f"[CAPTURE] done={done:.1f} m @ ({int(x_px)},{int(y_px)}) | " + ", ".join(saved_parts), flush=True)


def _apply_combined_safety(ui, state):
    """
    Обновляет state.safety_stop и HUD в зависимости от направления движения:
    - если едем вперёд (скорость >= 0) → учитываем только front
    - если едем назад (скорость < 0)   → учитываем только rear
    """
    v = get_speed(state)  # уже возвращает отрицательную скорость при заднем ходе
    if v >= 0:
        combined = bool(getattr(state, "safety_stop_front", False))
    else:
        combined = bool(getattr(state, "safety_stop_rear", False))

    state.safety_stop = combined
    set_safety_stop(ui, combined, state)  # тут и motors_stop, и "ПРЕПЯТСТВИЕ"
def _front_sector_ranges(pts, *, half_deg: float, R: float, Rmin: float, yaw0_rad: float):
    """
    pts: list[(x_m,y_m)] в координатах лидара/робота.
    yaw0_rad: компенсируем поворот лидара (если нужно).
    Возвращает список дальностей r (м) в секторе +/- half_deg и диапазоне (Rmin, R].
    """
    if not pts:
        return []

    half = float(half_deg) * math.pi / 180.0
    out = []
    for (x, y) in pts:
        x = float(x); y = float(y)
        r = math.hypot(x, y)
        if r <= float(Rmin) or r > float(R):
            continue

        a = math.atan2(y, x) - float(yaw0_rad)
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        if abs(a) > half:
            continue

        out.append(r)
    return out


def _adaptive_cluster_max_frac(ranges_m, *, rel: float = 0.06, w_min: float = 0.03):
    """
    1D кластеризация по дальности:
      - сортируем r
      - объединяем если соседние <= max(w_min, rel*r_ref)
    Возвращает (max_cluster_frac, max_cluster_cnt, n_total)
    """
    n = len(ranges_m)
    if n == 0:
        return 0.0, 0, 0

    rs = np.sort(np.asarray(ranges_m, dtype=np.float32))
    max_cnt = 1
    cnt = 1
    r_ref = float(rs[0])

    for i in range(1, n):
        thr_w = max(float(w_min), float(rel) * max(0.01, r_ref))
        if float(rs[i]) - float(rs[i - 1]) <= thr_w:
            cnt += 1
        else:
            if cnt > max_cnt:
                max_cnt = cnt
            cnt = 1
            r_ref = float(rs[i])

    if cnt > max_cnt:
        max_cnt = cnt

    return float(max_cnt) / float(n), int(max_cnt), int(n)


def _hard_object_guard(ranges_m, *, hard_r: float = 1.15, hard_min_hits: int = 3):
    """
    Защита от твёрдого объекта при низкой общей плотности:
    если есть компактный кластер ближе hard_r (например ствол).
    """
    near = [r for r in ranges_m if float(r) <= float(hard_r)]
    if len(near) < int(hard_min_hits):
        return False, 0
    _, cnt, _ = _adaptive_cluster_max_frac(near, rel=0.02, w_min=0.02)
    return (cnt >= int(hard_min_hits)), int(cnt)

def lidar_rear_stop(ui, state) -> bool:
    
    # если заднего лидара нет — всегда зелёная лампа и нет стопа
    if not getattr(state, "has_rear_lidar", False):
        try:
            indR = ui.findChild(QtWidgets.QLabel, "indLidarRear")
            if indR:
                set_indicator(indR, "ok")
        except Exception:
            pass
        state.safety_stop_rear = False
        _apply_combined_safety(ui, state)
        return False

    pts = getattr(state, "_lidar_rear_last_pts", None) or []

    stop = _lidar_check(
        pts=pts,
        half_deg=float(getattr(state, "lidar_rear_sector_half_deg", 25.0) or 25.0),
        R=float(getattr(state, "lidar_rear_stop_distance_m", 1) or 1),
        Rmin=float(getattr(state, "lidar_rear_ignore_radius_m", 0.01) or 0.01),
        need_n=int(getattr(state, "lidar_rear_stop_min_points", 3) or 3),
        yaw0_rad=float(getattr(state, "lidar_rear_mount_yaw_rad", 3.0) or 3.0),
    )

    prev = bool(getattr(state, "safety_stop_rear", False))
    state.safety_stop_rear = stop

    # Лампа заднего лидара
    try:
        indR = ui.findChild(QtWidgets.QLabel, "indLidarRear")
        if indR:
            set_indicator(indR, "bad" if stop else "ok")
    except Exception:
        pass

    _apply_combined_safety(ui, state)

    if stop and not prev:
        print("[LIDAR REAR STOP] triggered", flush=True)
    if (not stop) and prev:
        print("[LIDAR REAR STOP] cleared", flush=True)

    return stop

def _lidar(ui, state):
    return lidar_front_stop(ui, state)

def recompute_route_metrics(state):
    pts_px = getattr(state, "route_pts_px", None) or []
    if len(pts_px) < 2:
        state.route_len_px = 0.0
        state.route_len_m = 0.0
        state.route_cum_px = [0.0]
        state.alpha_m_per_px = 1.0
        return False

    Lpx = 0.0
    cum = [0.0]
    for a, b in zip(pts_px, pts_px[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        Lpx += d
        cum.append(Lpx)

    mx = float(getattr(state, "m_per_px_x", 0.0) or 0.0)
    my = float(getattr(state, "m_per_px_y", 0.0) or 0.0)
    if mx > 0.0 and my > 0.0:
        Lm = 0.0
        for a, b in zip(pts_px, pts_px[1:]):
            dx, dy = (b[0] - a[0]), (b[1] - a[1])
            Lm += math.hypot(dx * mx, dy * my)
    else:
        pts_m = getattr(state, "route_pts_m", None) or []
        if len(pts_m) == len(pts_px) and len(pts_m) >= 2:
            Lm = 0.0
            for a, b in zip(pts_m, pts_m[1:]):
                Lm += math.hypot(b[0] - a[0], b[1] - a[1])
        else:
            mpp = float(getattr(state, "meters_per_pixel", 0.0) or 0.0)
            Lm = Lpx * (mpp if mpp > 0.0 else 1.0)

    state.route_len_px = Lpx
    state.route_len_m = Lm
    state.route_cum_px = cum
    state.alpha_m_per_px = (Lm / Lpx) if Lpx > 1e-9 else 1.0
    state._carry_step_px = float(getattr(state, "_carry_step_px", 0.0) or 0.0)
    return True


# --------- старт анимации маршрута (движение по метрам) ----------


def start_route_animation(ui: QtWidgets.QMainWindow, state, fps: int = 30):
    pts = getattr(state, "route_pts_px", None) or []
    if len(pts) < 2 or float(getattr(state, "route_len_m", 0.0) or 0.0) <= 0.0:
        print("[ANIM] нет маршрута", flush=True)
        return False

    if not hasattr(state, "_route_timer") or state._route_timer is None:
        state._route_timer = QtCore.QTimer(ui)
    state._route_timer.setInterval(max(1, int(1000 / fps)))

    Lm = float(state.route_len_m)
    Lpx = float(state.route_len_px)
    if getattr(state, "route_done_m", 0.0) > Lm:
        state.route_done_m = Lm
    if getattr(state, "route_progress_idx", 0) >= len(pts) - 1:
        state.route_progress_idx = len(pts) - 2

    state._last_route_tick_ts = time.time()
    state._carry_step_px = float(getattr(state, "_carry_step_px", 0.0) or 0.0)

    def _interp_by_s_px(s_px: float):
        cum = state.route_cum_px
        i = 0
        while i < len(cum) - 1 and cum[i + 1] < s_px:
            i += 1
        i = min(i, len(pts) - 2)
        seg_len = max(1e-9, cum[i + 1] - cum[i])
        t = (s_px - cum[i]) / seg_len
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        return i, ax + t * (bx - ax), ay + t * (by - ay)

    def _tick():
        try:
            from robot_cmd import update_drive_panel, note_robot_pose
        except Exception:
            update_drive_panel = None
            note_robot_pose = None

        base_v = float(getattr(state, "speed_mps", 0.0) or 0.0)

        lidar_stop = False
        try:
            lidar_stop = bool(_lidar(ui, state))
        except Exception as e:
            print("[ANIM] lidar check error:", e, flush=True)
            lidar_stop = False

        road_block = False  # TODO: road_guard сюда же, когда включишь

        if lidar_stop or road_block:
            v = 0.0
        else:
            v = base_v

        if v <= 1e-6:
            for sc in (getattr(state, "_idle_scene", None),
                       getattr(state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(state, sc)
                    except Exception:
                        pass
            if update_drive_panel:
                try:
                    update_drive_panel(ui, state)
                except Exception:
                    pass
            return

        now = time.time()
        last = float(getattr(state, "_last_route_tick_ts", now) or now)
        dt = now - last
        if dt <= 0:
            dt = state._route_timer.interval() / 1000.0
        dt = max(1e-3, min(dt, 0.2))
        state._last_route_tick_ts = now

        alpha = float(getattr(state, "alpha_m_per_px", 1.0) or 1.0)

        step_px = (v * dt) / alpha + float(getattr(state, "_carry_step_px", 0.0) or 0.0)

        done_m_old = float(getattr(state, "route_done_m", 0.0) or 0.0)
        s_px_old = (done_m_old / Lm) * Lpx
        s_px_new = s_px_old + max(0.0, step_px)

        if s_px_new >= Lpx - 1e-6:
            state.route_done_m = Lm
            state.route_progress_idx = len(pts) - 2
            state.robot_px = pts[-1]
            state.robot_heading_rad = math.atan2(pts[-1][1] - pts[-2][1],
                                                 pts[-1][0] - pts[-2][0])
            try:
                if state._route_timer.isActive():
                    state._route_timer.stop()
            except Exception:
                pass
            state.is_running = False
            btn = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
            if btn:
                btn.setChecked(False)
                btn.setText("Пуск")
            from routing import clear_goals_and_route
            clear_goals_and_route(state)
        else:
            i, x, y = _interp_by_s_px(s_px_new)
            state.route_progress_idx = i
            state.robot_px = (x, y)
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            state.robot_heading_rad = math.atan2(by - ay, bx - ax)
            state.route_done_m = min(Lm, done_m_old + v * dt)
            exact_step_px = (state.route_done_m / Lm) * Lpx - s_px_old
            state._carry_step_px = (step_px - exact_step_px)

        if note_robot_pose:
            try:
                note_robot_pose(state)
            except Exception as e:
                print("[ROBOT] note_robot_pose error:", e, flush=True)

        try:
            update_visited_track(ui, state)
            dataset_maybe_capture(ui, state)
        except Exception as e:
            print("[ANIM] aux error:", e, flush=True)

        for sc in (getattr(state, "_idle_scene", None),
                   getattr(state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass
        if update_drive_panel:
            try:
                update_drive_panel(ui, state)
            except Exception:
                pass

    try:
        state._route_timer.timeout.disconnect()
    except (TypeError, RuntimeError):
        pass
    state._route_timer.timeout.connect(_tick)
    if not state._route_timer.isActive():
        state._route_timer.start()
    return True


def stop_route_animation(state, keep_progress: bool = True):
    try:
        if hasattr(state, "_route_timer") and state._route_timer and state._route_timer.isActive():
            state._route_timer.stop()
    except Exception:
        pass
    if not keep_progress:
        state.route_progress_idx = 0
        state.route_seg_off_m = 0.0


def clear_route_visual(state, also_clear_data: bool = True):
    for attr_item, attr_scene in (("idle_route_item", "_idle_scene"),
                                  ("drive_route_item", "_drive_scene")):
        item = getattr(state, attr_item, None)
        sc = getattr(state, attr_scene, None)
        if item and sc and item.scene() is sc:
            try:
                sc.removeItem(item)
            except Exception:
                pass
        setattr(state, attr_item, None)

    if also_clear_data:
        state.route_pts_px = []
        state.route_pts_m = []
        state.route_len_m = 0.0
        state.route_done_m = 0.0
    state.route_finished = True


def update_visited_track(ui, state, min_seg_px: float = 2.0):
    if not bool(getattr(state, "dataset_mode", False)):
        return
    rp = getattr(state, "robot_px", None)
    if not rp:
        return
    pts = state.visited_path_px
    if pts:
        dx = rp[0] - pts[-1][0]
        dy = rp[1] - pts[-1][1]
        if (dx * dx + dy * dy) < (min_seg_px * min_seg_px):
            return
    pts.append((float(rp[0]), float(rp[1])))

    sc = getattr(state, "_drive_scene", None)
    if sc is None:
        return
    path = QtGui.QPainterPath()
    if not pts:
        return
    path.moveTo(pts[0][0], pts[0][1])
    for x, y in pts[1:]:
        path.lineTo(x, y)
    if state.drive_visited_item is None:
        item = QtWidgets.QGraphicsPathItem(path)
        pen = QtGui.QPen(QtGui.QColor("#22aa22"))
        pen.setWidthF(2.0)
        pen.setCosmetic(True)
        item.setPen(pen)
        item.setZValue(3)
        sc.addItem(item)
        state.drive_visited_item = item
    else:
        state.drive_visited_item.setPath(path)


# ==== dataset capture (камера + лидар) ====

def _dataset_dirs(state):
    map_path = getattr(state, "active_map_path", None) or "map"
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    root = getattr(state, "dataset_root", os.path.expanduser("~/datasets"))
    photos_dir = os.path.join(root, "photos", map_name)
    lidar_dir = os.path.join(root, "lidar", map_name)
    os.makedirs(photos_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)
    return photos_dir, lidar_dir


def _save_camera_frame(state, x_px, y_px):
    cap = getattr(state, "_cam", None)
    if cap is None or not cap.isOpened():
        return None

    ok, frame = cap.read()
    if not ok or frame is None:
        return None

    photos_dir, _ = _dataset_dirs(state)
    fname = f"{int(x_px)}_{int(y_px)}.png"
    path = os.path.join(photos_dir, fname)

    try:
        if cv2.imwrite(path, frame):
            return os.path.abspath(path)
    except Exception as e:
        print("[CAMERA SAVE] error:", e, flush=True)
    return None

def _ensure_dataset_dirs(state):
    root = getattr(state, "dataset_root", os.path.expanduser("~/datasets")) or os.path.expanduser("~/datasets")
    map_path = getattr(state, "active_map_path", None)
    if not map_path:
        map_name = "unnamed_map"
    else:
        map_name = os.path.splitext(os.path.basename(map_path))[0]
    state.map_name = map_name

    photos_dir = os.path.join(root, "photos", map_name)
    lidar_dir = os.path.join(root, "lidar", map_name)
    os.makedirs(photos_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)

    state.dataset_photos_dir = photos_dir
    state.dataset_lidar_dir = lidar_dir
    return photos_dir, lidar_dir


def _save_lidar_snapshot(state, x_px, y_px):
    pts = getattr(state, "_lidar_last_pts", None) or []
    if not pts:
        return None

    now_ms = int(time.time() * 1000)
    last_ts = int(getattr(state, "_lidar_last_ts", 0))
    if now_ms - last_ts > 1500:
        return None

    _, lidar_dir = _ensure_dataset_dirs(state)
    base = f"{int(x_px)}_{int(y_px)}"
    json_path = os.path.join(lidar_dir, base + ".json")
    png_path = os.path.join(lidar_dir, base + ".png")

    try:
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(
                {"px": [int(x_px), int(y_px)], "ts_ms": now_ms, "pts": pts},
                f, ensure_ascii=False, separators=(",", ":")
            )
    except Exception:
        return None

    try:
        size = int(getattr(state, "lidar_png_size_px", 400) or 400)
        rmax = float(getattr(state, "lidar_png_radius_m", 6.0) or 6.0)

        img = QtGui.QImage(size, size, QtGui.QImage.Format_RGB32)
        img.fill(QtGui.QColor("#0d1117"))

        qp = QtGui.QPainter(img)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)

        pen_axes = QtGui.QPen(QtGui.QColor("#334155"))
        pen_axes.setWidth(1)
        pen_axes.setCosmetic(True)
        cx = size * 0.5
        cy = size * 0.5
        qp.setPen(pen_axes)
        qp.drawLine(0, int(cy), size, int(cy))
        qp.drawLine(int(cx), 0, int(cx), size)

        pen_grid = QtGui.QPen(QtGui.QColor("#1f2937"))
        pen_grid.setWidth(1)
        pen_grid.setCosmetic(True)
        qp.setPen(pen_grid)
        ppm = (size * 0.5 - 4.0) / max(0.1, rmax)
        m = 1.0
        while m <= rmax + 1e-9:
            rad = m * ppm
            qp.drawEllipse(QtCore.QPointF(cx, cy), rad, rad)
            m += 1.0

        pen_pts = QtGui.QPen(QtGui.QColor("#00bcd4"))
        pen_pts.setWidth(0)
        pen_pts.setCosmetic(True)
        brush = QtGui.QBrush(QtGui.QColor("#00bcd4"))
        qp.setPen(pen_pts)
        qp.setBrush(brush)
        d = 2.0
        for (xm, ym) in pts:
            xp = cx + xm * ppm
            yp = cy - ym * ppm
            qp.drawEllipse(QtCore.QRectF(xp - d, yp - d, 2 * d, 2 * d))

        qp.end()

        if not img.save(png_path, "PNG"):
            png_path = None
    except Exception:
        png_path = None

    return (os.path.abspath(json_path), os.path.abspath(png_path) if png_path else None)


def dataset_maybe_capture(ui: QtWidgets.QMainWindow, state):
    if not getattr(state, "dataset_mode", False):
        return
    if not getattr(state, "is_running", False):
        return

    Lm = float(getattr(state, "route_len_m", 0.0) or 0.0)
    done = float(getattr(state, "route_done_m", 0.0) or 0.0)
    if Lm <= 0.0:
        return

    step = float(getattr(state, "dataset_step_m", 3.0) or 3.0)
    last = float(getattr(state, "dataset_last_snap_m", 0.0) or 0.0)
    if (done - last) < step:
        return

    rp = getattr(state, "robot_px", None)
    if not rp:
        return
    x_px, y_px = float(rp[0]), float(rp[1])

    _ensure_dataset_dirs(state)

    saved_parts = []

    lid = _save_lidar_snapshot(state, x_px, y_px)
    if lid:
        jp, pp = lid
        if jp:
            saved_parts.append(f"lidar-json -> {jp}")
        if pp:
            saved_parts.append(f"lidar-png  -> {pp}")

    cam = _save_camera_frame(state, x_px, y_px)
    if cam:
        saved_parts.append(f"photo -> {cam}")

    if saved_parts:
        state.dataset_last_snap_m = done
        print(f"[CAPTURE] done={done:.1f} m @ ({int(x_px)},{int(y_px)}) | " + ", ".join(saved_parts), flush=True)


def _apply_combined_safety(ui, state):
    """
    Обновляет state.safety_stop и HUD в зависимости от направления движения:
    - если едем вперёд (скорость >= 0) → учитываем только front
    - если едем назад (скорость < 0)   → учитываем только rear
    """
    v = get_speed(state)  # уже возвращает отрицательную скорость при заднем ходе
    if v >= 0:
        combined = bool(getattr(state, "safety_stop_front", False))
    else:
        combined = bool(getattr(state, "safety_stop_rear", False))

    state.safety_stop = combined
    set_safety_stop(ui, combined, state)  # тут и motors_stop, и "ПРЕПЯТСТВИЕ"