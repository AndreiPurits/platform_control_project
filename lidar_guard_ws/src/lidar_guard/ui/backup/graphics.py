# graphics.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtGui, QtCore
import math

Point = Tuple[float, float]

# ------------------ угол/поворот (плавная стрелка) ------------------
_DEG = math.pi / 180.0
_HEADING_DEADBAND = 7.0 * _DEG      # мёртвая зона ±3°
_MAX_TURN_RATE_DPS = 120.0          # макс. скорость поворота, °/с
AHEAD_PX_DEFAULT = 40.0  # насколько «вперёд» смотреть по маршруту, в пикселях

def _heading_from_vector(dx: float, dy: float) -> float:
    """Экранный угол (рад): 0 = вправо, +вниз = +90°."""
    return math.atan2(dy, dx)

def _target_point_ahead_px(cur_pos: Tuple[float,float],
                           poly_px: List[Tuple[float,float]],
                           start_i: int,
                           ahead_px: float) -> Tuple[float,float]:
    """
    Возвращает точку на ломаной, отстоящую от cur_pos примерно на ahead_px по дуге.
    Идём от сегмента start_i → вперёд, накапливая пиксельную длину.
    Если до конца осталось меньше — вернём последний доступный.
    """
    if not poly_px or start_i >= len(poly_px)-1:
        return poly_px[-1] if poly_px else cur_pos

    # начинаем от ТЕКУЩЕЙ позиции (не от вершины), чтобы вектор не дёргался
    px, py = cur_pos
    left = float(ahead_px)

    # текущий сегмент i: от вершины i до i+1
    i = max(0, min(start_i, len(poly_px)-2))
    # сначала учитываем остаток текущего сегмента от cur_pos до конца сегмента
    ax, ay = poly_px[i]
    bx, by = poly_px[i+1]
    # проекция текущей позиции на сегмент i → т, точка q
    vx, vy = bx-ax, by-ay
    wx, wy = px-ax, py-ay
    vv = vx*vx+vy*vy
    if vv <= 1e-12:
        t0 = 0.0
        qx, qy = ax, ay
    else:
        t0 = (wx*vx + wy*vy)/vv
        t0 = 0.0 if t0 < 0.0 else (1.0 if t0 > 1.0 else t0)
        qx, qy = ax + t0*vx, ay + t0*vy

    # пробуем доползти внутри текущего сегмента
    seg_rest = math.hypot(bx-qx, by-qy)
    if left <= seg_rest:
        # точка впереди внутри текущего сегмента
        tt = 0.0 if seg_rest <= 1e-9 else (left/seg_rest)
        return (qx + tt*(bx-qx), qy + tt*(by-qy))
    left -= seg_rest
    i += 1

    # дальше шагами по полилинии
    while i < len(poly_px)-1 and left > 0.0:
        ax, ay = poly_px[i]
        bx, by = poly_px[i+1]
        seg = math.hypot(bx-ax, by-ay)
        if seg <= 1e-9:
            i += 1
            continue
        if left <= seg:
            t = left/seg
            return (ax + t*(bx-ax), ay + t*(by-ay))
        left -= seg
        i += 1

    # не хватило длины — берём последний
    return poly_px[-1]

def _angdiff(a, b):
    """Нормализованная разность углов b-a в диапазон [-pi, pi]."""
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return d

def _step_heading(prev, target, dt):
    """Плавный шаг к target: мёртвая зона + ограничение скорости поворота."""
    if prev is None:
        return target
    diff = _angdiff(prev, target)
    if abs(diff) <= _HEADING_DEADBAND:
        return prev
    max_step = _MAX_TURN_RATE_DPS * _DEG * max(0.0, float(dt))
    if abs(diff) > max_step:
        return prev + (max_step if diff > 0.0 else -max_step)
    return target

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
    # очистим прошлые items
    for name in ("idle_route_item","drive_route_item",
                 "idle_robot_item","drive_robot_item",
                 "idle_goal_item","drive_goal_item"):
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
        new_item = draw_polyline_path(sc, item, getattr(state, "route_pts_px", []) or [], "#e53935", 3)
        setattr(state, holder, new_item)

# ---------- маркеры ----------

def _make_goal_dot() -> QtWidgets.QGraphicsEllipseItem:
    r = 5
    item = QtWidgets.QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ff3b30")))
    item.setPen(QtGui.QPen(QtGui.QColor("#222"), 1))
    item.setZValue(120)
    return item

def _make_arrow() -> QtWidgets.QGraphicsPolygonItem:
    # стрелка вдоль +X (вправо)
    poly = QtGui.QPolygonF([
        QtCore.QPointF( 12,  0),  # нос вправо
        QtCore.QPointF(-10,  6),
        QtCore.QPointF( -6,  0),
        QtCore.QPointF(-10, -6),
    ])
    item = QtWidgets.QGraphicsPolygonItem(poly)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ffffff")))
    item.setPen(QtGui.QPen(QtGui.QColor("#333333"), 1))
    item.setZValue(120)
    return item

def draw_robot_arrow(scene, old_item, pos, yaw_rad=None):
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pos:
        return None
    it = _make_arrow()
    it.setPos(pos[0], pos[1])
    if yaw_rad is not None:
        it.setRotation(- (yaw_rad * 180.0 / math.pi))  # 0рад=вправо; вниз=+90° → поворачиваем на -90
    scene.addItem(it)
    return it

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
    """Простой флажок для контрольных точек (по умолчанию синий)."""
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
    """
    Удаляет синие флажки, если позиция робота достаточно близко (<= eps_px).
    Для КАЖДОГО удалённого флажка печатает дамп состояния (если ui передан).
    """
    rp = getattr(state, "robot_px", None)
    if not rp:
        return

    rx, ry = float(rp[0]), float(rp[1])
    flags = list(getattr(state, "control_pts_px", []))
    kept = []
    removed_any = False
    r2 = eps_px * eps_px

    for pt in flags:
        px, py = float(pt[0]), float(pt[1])
        dx = px - rx
        dy = py - ry
        if dx*dx + dy*dy <= r2:
            print(f"[FLAG] reached ~ε={eps_px}px: {pt} -> ВЫХОД 3", flush=True)
            removed_any = True
            # дамп состояния после удаления каждого флага
            try:
                if ui is not None and hasattr(ui, "_dump_state"):
                    ui._dump_state(f"after FLAG REMOVED at {pt}")
            except Exception as e:
                print("[FLAG] dump_state error:", e, flush=True)
            continue
        kept.append(pt)

    if removed_any:
        state.control_pts_px = kept
        # перерисуем маркеры (чтобы удалённые флажки исчезли)
        for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass

def _on_flag_reached(self, pt):
    print(f"[DRIVE] КОМАНДА: достигнута КР {pt} → ВЫХОД 3 (stub)", flush=True)

def redraw_markers(state, scene: QtWidgets.QGraphicsScene):
    """Перерисовать стрелку робота, цель и синие флаги на указанной сцене."""
    yaw = getattr(state, "robot_heading_rad", None)

    if scene is getattr(state, "_idle_scene", None):
        state.idle_robot_item = draw_robot_arrow(
            scene, getattr(state, "idle_robot_item", None),
            getattr(state, "robot_px", None), yaw
        )
        state.idle_goal_item  = draw_goal_marker(
            scene, getattr(state, "idle_goal_item",  None),
            getattr(state, "goal_px",  None)
        )
        # перерисовать флаги
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
        state.drive_robot_item = draw_robot_arrow(
            scene, getattr(state, "drive_robot_item", None),
            getattr(state, "robot_px", None), yaw
        )
        state.drive_goal_item  = draw_goal_marker(
            scene, getattr(state, "drive_goal_item",  None),
            getattr(state, "goal_px",  None)
        )
        # перерисовать флаги
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

# (опционально) точки лидара
def draw_radar_points(scene: QtWidgets.QGraphicsScene, pts_m, meters_to_px: float = 40.0):
    for (x, y) in pts_m:
        xp = x * meters_to_px
        yp = -y * meters_to_px
        dot = QtWidgets.QGraphicsEllipseItem(xp - 1, yp - 1, 2, 2)
        dot.setBrush(QtGui.QBrush(QtGui.QColor("#00bcd4")))
        dot.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        dot.setZValue(50)
        scene.addItem(dot)

# ===================== АНИМАЦИЯ МАРШРУТА =====================
# ВАЖНО: никаких импортов robot_cmd сверху — чтобы не ловить циклический импорт.
def reset_route_progress(state):
    state.route_done_m = 0.0
    state._route_cum_m = None

def _nearest_index_on_poly(poly, pxy):
    if not poly:
        return 0
    px, py = float(pxy[0]), float(pxy[1])
    best_i, best_d2 = 0, float("inf")
    for i, (x, y) in enumerate(poly):
        d2 = (x - px)*(x - px) + (y - py)*(y - py)
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i

def start_route_animation(ui: QtWidgets.QMainWindow, state, fps: int = 30):
    # дамп состояния перед стартом (если есть метод)
    try:
        if hasattr(ui, "_dump_state"):
            ui._dump_state("before start_route_animation")
    except Exception:
        pass

    pts_px = getattr(state, "route_pts_px", None) or []
    pts_m  = getattr(state, "route_pts_m",  None) or []

    if len(pts_px) < 2 or len(pts_m) != len(pts_px):
        print("[ANIM] Нет валидного маршрута для анимации", flush=True)
        return False

    # если таймер есть — останавливаем перед перенастройкой
    try:
        if hasattr(state, "_route_timer") and state._route_timer and state._route_timer.isActive():
            state._route_timer.stop()
    except Exception:
        pass

    # НЕ сбрасываем прогресс, только нормализуем индекс
    i = int(getattr(state, "route_progress_idx", 0) or 0)
    if i < 0:
        i = 0
    if i >= len(pts_px) - 1:
        i = len(pts_px) - 2
    state.route_progress_idx = i
    state.route_finished = False

    # таймер
    if not hasattr(state, "_route_timer") or state._route_timer is None:
        state._route_timer = QtCore.QTimer(ui)
    state._route_timer.setInterval(max(1, int(1000 / fps)))

    def _tick():
        # ленивый импорт, чтобы не ловить циклические
        try:
            from robot_cmd import update_drive_panel
        except Exception:
            update_drive_panel = None

        v = float(getattr(state, "speed_mps", 0.0) or 0.0)
        pts_px = getattr(state, "route_pts_px", []) or []
        pts_m  = getattr(state, "route_pts_m",  []) or []

        # валидация маршрута
        if len(pts_px) < 2 or len(pts_m) != len(pts_px):
            try:
                if state._route_timer and state._route_timer.isActive():
                    state._route_timer.stop()
            except Exception:
                pass
            return

        i = int(getattr(state, "route_progress_idx", 0) or 0)
        if i < 0:
            i = 0
        if i >= len(pts_px) - 1:
            # уже на финише
            try:
                if state._route_timer and state._route_timer.isActive():
                    state._route_timer.stop()
            except Exception:
                pass
            state.route_finished = True
            return

        # === ЕСЛИ СТОИМ ===
        if v <= 1e-6:
            # НЕ меняем угол (robot_heading_rad)
            for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(state, sc)
                    except Exception:
                        pass
            try:
                check_flag_collision_and_update(state, eps_px=4.0, ui=ui)
            except Exception:
                pass
            if update_drive_panel:
                try:
                    update_drive_panel(ui, state)
                except Exception:
                    pass
            return

        # === ДВИЖЕНИЕ ===
        moved = False
        done  = float(getattr(state, "route_done_m", 0.0) or 0.0)
        L     = float(getattr(state, "route_len_m", 0.0) or 0.0)
        dt    = max(1e-3, state._route_timer.interval() / 1000.0)
        step_m = v * dt

        while step_m > 0 and i < len(pts_m) - 1:
            ax, ay = pts_m[i]; bx, by = pts_m[i+1]
            seg = math.hypot(bx - ax, by - ay)
            if seg <= 1e-9:
                i += 1
                continue

            if step_m < seg:
                # внутри текущего сегмента
                t = step_m / seg
                axp, ayp = pts_px[i]; bxp, byp = pts_px[i+1]
                nx = axp + t * (bxp - axp)
                ny = ayp + t * (byp - ayp)

                state.route_progress_idx = i
                state.robot_px = (nx, ny)

                # ЖЁСТКИЙ КУРС по текущему сегменту — без сглаживания
                desired = math.atan2(byp - ayp, bxp - axp)
                state.robot_heading_rad = desired

                moved = True
                done += step_m
                step_m = 0.0
                break
            else:
                # доходим до вершины i+1
                step_m -= seg
                done   += seg
                i += 1
                state.route_progress_idx = i
                state.robot_px = pts_px[i]
                moved = True

                # финиш?
                if i >= len(pts_m) - 1:
                    state.route_progress_idx = len(pts_m) - 1
                    state.robot_px = pts_px[-1]
                    done = L
                    try:
                        if state._route_timer and state._route_timer.isActive():
                            state._route_timer.stop()
                    except Exception:
                        pass

                    clear_route_visual(state, also_clear_data=True)  # убрать красный путь, флаги не трогаем
                    state.route_finished = True

                    # кнопка -> «Пуск»
                    try:
                        state.is_running = False
                        btn = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
                        if btn:
                            btn.setChecked(False)
                            btn.setText("Пуск")
                    except Exception:
                        pass

                    # перерисовка и HUD
                    for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
                        if sc is not None:
                            try:
                                redraw_markers(state, sc)
                            except Exception:
                                pass
                    state.route_done_m = float(done)
                    if update_drive_panel:
                        try:
                            update_drive_panel(ui, state)
                        except Exception:
                            pass
                    print("[ANIM] finish", flush=True)
                    return

                # не финиш — сразу выставим курс вдоль следующего сегмента (мгновенно)
                a = pts_px[i]; b = pts_px[i+1]
                state.robot_heading_rad = math.atan2(b[1] - a[1], b[0] - a[0])

        # фиксируем прогресс
        state.route_done_m = float(min(done, L))

        # перерисовка и служебные проверки каждый тик
        for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(state, sc)
                except Exception:
                    pass
        try:
            check_flag_collision_and_update(state, eps_px=4.0, ui=ui)
        except Exception:
            pass
        if update_drive_panel:
            try:
                update_drive_panel(ui, state)
            except Exception:
                pass
        # --- подписка и запуск таймера (обязательно!) ---
    try:
        # если раньше кто-то уже был подписан — убираем, чтобы не было дублей
        state._route_timer.timeout.disconnect()
    except (TypeError, RuntimeError):
        pass

    state._route_timer.timeout.connect(_tick)

    if not state._route_timer.isActive():
        state._route_timer.start()

    print(f"[ANIM] timer started, interval={state._route_timer.interval()} ms", flush=True)
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
        sc   = getattr(state, attr_scene, None)
        if item and sc and item.scene() is sc:
            try: sc.removeItem(item)
            except Exception: pass
        setattr(state, attr_item, None)

    if also_clear_data:
        state.route_pts_px = []
        state.route_pts_m  = []
        state.route_len_m  = 0.0
        state.route_done_m = 0.0
    state.route_finished = True