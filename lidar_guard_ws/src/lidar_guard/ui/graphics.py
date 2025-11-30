# graphics.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtGui, QtCore
from robot_cmd import tool_pulse
import math
import os, json, time

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


def redraw_route(state, ui: Optional[QtWidgets.QMainWindow] = None) -> None:
    for sc, holder in ((getattr(state, "_idle_scene", None), "idle_route_item"),
                       (getattr(state, "_drive_scene", None), "drive_route_item")):
        if sc is None:
            continue
        item = getattr(state, holder, None)
        new_item = draw_polyline_path(sc, item,
                                      getattr(state, "route_pts_px", []) or [],
                                      "#e53935", 3)
        recompute_route_metrics(state)
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


def draw_flag(scene: QtWidgets.QGraphicsScene, pos_px, color="#1e88e5"):
    if not scene or not pos_px:
        return None
    pen = QtGui.QPen(QtGui.QColor(color))
    pen.setWidth(2)
    pen.setCosmetic(True)
    brush = QtGui.QBrush(QtGui.QColor(color))
    size = 12
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
def check_flag_collision_and_update(state,
                                    eps_px: float = 4.0,
                                    ui: Optional[QtWidgets.QMainWindow] = None):
    rp = getattr(state, "robot_px", None)
    if not rp:
        return

    rx, ry = float(rp[0]), float(rp[1])
    flags = list(getattr(state, "control_pts_px", []))
    kept = []
    removed_any = False

    heading = float(getattr(state, "robot_heading_rad", 0.0) or 0.0)
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    front_boost = 1.4
    back_cutoff = 0.7

    for pt in flags:
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
            print(f"[FLAG] reached ~ε={eps_px}px (adj) {pt} -> ВЫХОД 3", flush=True)
            removed_any = True
            try:
                if ui is not None and hasattr(ui, "_dump_state"):
                    ui._dump_state(f"after FLAG REMOVED at {pt}")
            except Exception as e:
                print("[FLAG] dump_state error:", e, flush=True)
            continue

        kept.append(pt)

    if removed_any:
        state.control_pts_px = kept
        # ВАЖНО: пинаем вспомогательный канал в 2000 мкс на 200 мс
        tool_pulse(ui, state, level_us=2000, ms=200)
        for sc in (getattr(state, "_idle_scene", None),
                   getattr(state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass


def redraw_markers(state, scene: QtWidgets.QGraphicsScene):
    if scene is getattr(state, "_idle_scene", None):
        state.idle_robot_item = draw_robot_dot(
            scene, getattr(state, "idle_robot_item", None),
            getattr(state, "robot_px", None)
        )
        state.idle_goal_item = draw_goal_marker(
            scene, getattr(state, "idle_goal_item", None),
            getattr(state, "goal_px", None)
        )
        for it in getattr(state, "idle_control_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.idle_control_items = []
        for pt in getattr(state, "control_pts_px", []):
            it = draw_flag(scene, pt, color="#1e88e5")
            if it:
                state.idle_control_items.append(it)

    elif scene is getattr(state, "_drive_scene", None):
        state.drive_robot_item = draw_robot_dot(
            scene, getattr(state, "drive_robot_item", None),
            getattr(state, "robot_px", None)
        )
        state.drive_goal_item = draw_goal_marker(
            scene, getattr(state, "drive_goal_item", None),
            getattr(state, "goal_px", None)
        )
        for it in getattr(state, "drive_control_items", []):
            try:
                if it and it.scene() is scene:
                    scene.removeItem(it)
            except Exception:
                pass
        state.drive_control_items = []
        for pt in getattr(state, "control_pts_px", []):
            it = draw_flag(scene, pt, color="#1e88e5")
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
def lidar_front_stop(ui, state) -> bool:

    pts = getattr(state, "_lidar_front_last_pts", None) or []

    stop = _lidar_check(
        pts=pts,
        half_deg=float(getattr(state, "lidar_front_sector_half_deg", 15.0) or 15.0),
        R=float(getattr(state, "lidar_front_stop_distance_m", 2.0) or 2.0),
        Rmin=float(getattr(state, "lidar_front_ignore_radius_m", 0.7) or 0.7),
        need_n=int(getattr(state, "lidar_front_stop_min_points", 3) or 3),
        yaw0_rad=float(getattr(state, "lidar_front_mount_yaw_rad", 0.0) or 0.0),
    )

    prev = bool(getattr(state, "safety_stop_front", False))
    state.safety_stop_front = stop

    # Лампа переднего лидара
    try:
        indF = ui.findChild(QtWidgets.QLabel, "indLidar")
        if indF:
            set_indicator(indF, "bad" if stop else "ok")
    except Exception:
        pass

    _apply_combined_safety(ui, state)

    if stop and not prev:
        print("[LIDAR FRONT STOP] triggered", flush=True)
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
from PyQt5 import QtWidgets  # уже импортирован, но пусть будет явно


def start_route_animation(ui: QtWidgets.QMainWindow, state, fps: int = 30):
    print(f"[CALIB] Lm={state.route_len_m:.3f} m  |  Lpx={state.route_len_px:.3f} px  |  alpha={state.alpha_m_per_px:.6f} m/px")
    print(f"[STEP] done_m={state.route_done_m:.2f}/{state.route_len_m:.2f}  s_px={(state.route_done_m / max(1e-9, state.route_len_m)) * state.route_len_px:.1f}/{state.route_len_px:.1f}")
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
import cv2  # вверху, но оставим


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

from robot_cmd import set_safety_stop, get_speed
from status import set_indicator
from PyQt5 import QtWidgets

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