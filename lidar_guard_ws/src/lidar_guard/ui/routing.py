# -*- coding: utf-8 -*-
"""
== API INDEX ================================================================
PUBLIC:
- ensure_scene(view, pixmap=None) -> QGraphicsScene
    Создаёт/возвращает сцену у QGraphicsView. Если дан pixmap — кладёт как фон.
- prepare_view(view) -> None
    Базовая настройка QGraphicsView (перетаскивание, антиалиасинг, якорь зума).
- show_map_on_views(png_path, idle_view, drive_view, state) -> None
    Грузит PNG в оба вида, сохраняет сцены в state, поднимает HUD, сбрасывает старый маршрут.
- ensure_hud(scene) -> (group, text_item)
    Создаёт (если нет) HUD-плашку «Маршрут / Пройдено / Осталось».
- update_hud_text(scene, text) -> None
    Обновляет текст HUD.
- draw_polyline_path(scene, old_item, pts, color="#e53935", width=3) -> QGraphicsPathItem|None
    Рисует/перерисовывает ломаную (маршрут) поверх карты.
- redraw_route(state, ui=None) -> None
    Главный метод отрисовки маршрута ГРАФА: берёт state.route_pts_px, рисует в обеих сценах,
    обновляет HUD строкой "Маршрут: X м" (если доступно state.route_len_m).
- redraw_idle_overlays(state, scene) -> None
    Совместимый хук под старый вызов: теперь просто вызывает графовую отрисовку для idle-сцены.
COMPAT HELPERS (маркеры робота/цели):
- _flag_item(scene, old_item, pos_qt, color="#ffffff") -> QGraphicsItemGroup
- _cross_item(scene, old_item, pos_qt, color="#d64545") -> QGraphicsItemGroup

КОММЕНТАРИЙ ПО ЛОГИКЕ ГРАФОВ:
- Мы предполагаем, что построение маршрута выполнено в routing.build_route_snap_pixels(...)
  и в state заполнены:
    state.route_pts_px : List[Tuple[float,float]]  # путь в пикселях, в системе PNG (Y вниз)
    state.route_len_m  : float                      # длина в метрах (для HUD)
  Дополнительно (для маркеров) могут быть заданы:
    state.robot_px : Tuple[float,float]            # положение робота в px
    state.goal_px  : Tuple[float,float]            # цель в px
- Отрисовка НЕ использует сплайны и индексы; всё завязано на граф.
============================================================================
"""

from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtCore, QtGui

Point = Tuple[float, float]


# ======================================================================
# БАЗОВАЯ РАБОТА СО СЦЕНОЙ / КАРТОЙ
# ======================================================================

def ensure_scene(view: QtWidgets.QGraphicsView,
                 pixmap: Optional[QtGui.QPixmap] = None) -> QtWidgets.QGraphicsScene:
    """
    Гарантирует наличие QGraphicsScene у view.
    Если передан pixmap, очищает сцену и кладёт его как фон.
    """
    sc = view.scene()
    if sc is None:
        sc = QtWidgets.QGraphicsScene()
        view.setScene(sc)
    if pixmap is not None:
        sc.clear()
        sc.addPixmap(pixmap)
    view.setRenderHint(QtGui.QPainter.Antialiasing, True)
    return sc


def prepare_view(view: QtWidgets.QGraphicsView) -> None:
    """
    Лёгкая дефолтная подготовка вида (чтобы внешние импорты не падали).
    Если у тебя есть свой «умный» класс вида — эту функцию можно оставить пустой.
    """
    view.setRenderHint(QtGui.QPainter.Antialiasing, True)
    view.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
    view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)


def show_map_on_views(png_path: str,
                      idle_view: Optional[QtWidgets.QGraphicsView],
                      drive_view: Optional[QtWidgets.QGraphicsView],
                      state) -> None:
    """
    Грузит PNG и показывает в обоих QGraphicsView.
    - Создаёт/обновляет сцены.
    - Поднимает HUD.
    - Сбрасывает прошлые items маршрута и маркеров в state.
    """
    pm = QtGui.QPixmap(png_path)

    if idle_view is not None:
        sc_idle = ensure_scene(idle_view, pm)
        ensure_hud(sc_idle)
    else:
        sc_idle = None

    if drive_view is not None:
        sc_drive = ensure_scene(drive_view, pm)
        ensure_hud(sc_drive)
    else:
        sc_drive = None

    # сохраняем ссылки на сцены и «держатели» отрисованных items
    state._idle_scene = sc_idle
    state._drive_scene = sc_drive
    state.idle_route_item = None
    state.drive_route_item = None
    state.idle_robot_item = None
    state.idle_goal_item = None
    state.drive_robot_item = None
    state.drive_goal_item = None


# ======================================================================
# HUD: «Маршрут / Пройдено / Осталось»
# ======================================================================

def ensure_hud(scene: QtWidgets.QGraphicsScene):
    """
    Создаёт (если нет) полупрозрачную плашку HUD в левом верхнем углу и текст.
    Возвращает (group, text_item).
    """
    if hasattr(scene, "_route_hud_group") and scene._route_hud_group:
        return scene._route_hud_group, scene._route_hud_text

    rect = QtCore.QRectF(8, 8, 460, 46)
    bg = scene.addRect(rect, QtGui.QPen(QtCore.Qt.NoPen),
                       QtGui.QBrush(QtGui.QColor(0, 0, 0, 120)))
    text = scene.addText("Маршрут не задан", QtGui.QFont("Inter", 12))
    text.setDefaultTextColor(QtGui.QColor("#ffffff"))
    text.setPos(16, 14)

    group = scene.createItemGroup([bg, text])
    group.setZValue(9999)

    scene._route_hud_group = group
    scene._route_hud_text = text
    return group, text


def update_hud_text(scene: QtWidgets.QGraphicsScene, text: str):
    """Обновляет текст HUD, поднимая плашку при необходимости."""
    _, t = ensure_hud(scene)
    t.setPlainText(text)


# ======================================================================
# ОТРИСОВКА МАРШРУТА И МАРКЕРОВ
# ======================================================================

def draw_polyline_path(scene: QtWidgets.QGraphicsScene,
                       old_item: Optional[QtWidgets.QGraphicsItem],
                       pts: List[Point],
                       color: str = "#e53935",
                       width: int = 3) -> Optional[QtWidgets.QGraphicsPathItem]:
    """
    Перерисовывает ломаную (маршрут). Возвращает добавленный item.
    Если old_item задан и принадлежит сцене — удаляется.
    """
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pts:
        return None

    path = QtGui.QPainterPath(QtCore.QPointF(pts[0][0], pts[0][1]))
    for (x, y) in pts[1:]:
        path.lineTo(x, y)

    pen = QtGui.QPen(QtGui.QColor(color))
    pen.setWidth(width)
    pen.setCosmetic(True)

    item = scene.addPath(path, pen)
    item.setZValue(100)  # поверх карты, но под HUD
    return item


def _flag_item(scene: QtWidgets.QGraphicsScene,
               old_item: Optional[QtWidgets.QGraphicsItem],
               pos: QtCore.QPointF,
               color: str = "#ffffff") -> QtWidgets.QGraphicsItemGroup:
    """Маркер робота (флажок)."""
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)

    pen = QtGui.QPen(QtGui.QColor("#333"))
    pen.setWidth(2)

    mast = scene.addLine(pos.x(), pos.y(), pos.x(), pos.y() - 20, pen)
    flag = scene.addPolygon(
        QtGui.QPolygonF([
            QtCore.QPointF(pos.x(), pos.y() - 20),
            QtCore.QPointF(pos.x() + 16, pos.y() - 16),
            QtCore.QPointF(pos.x(), pos.y() - 12),
        ]),
        QtGui.QPen(QtGui.QColor("#333")),
        QtGui.QBrush(QtGui.QColor(color))
    )
    base = scene.addEllipse(pos.x() - 2, pos.y() - 2, 4, 4, pen, QtGui.QBrush(QtGui.QColor("#333")))

    group = scene.createItemGroup([mast, flag, base])
    group.setZValue(200)
    return group


def _cross_item(scene: QtWidgets.QGraphicsScene,
                old_item: Optional[QtWidgets.QGraphicsItem],
                pos: QtCore.QPointF,
                color: str = "#d64545") -> QtWidgets.QGraphicsItemGroup:
    """Маркер цели (крестик)."""
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)

    size = 10
    pen = QtGui.QPen(QtGui.QColor(color))
    pen.setWidth(2)

    l1 = scene.addLine(pos.x() - size, pos.y(), pos.x() + size, pos.y(), pen)
    l2 = scene.addLine(pos.x(), pos.y() - size, pos.x(), pos.y() + size, pen)
    dot = scene.addEllipse(pos.x() - 3, pos.y() - 3, 6, 6, pen, QtGui.QBrush(QtGui.QColor(color)))

    group = scene.createItemGroup([l1, l2, dot])
    group.setZValue(200)
    return group


def _draw_robot_and_goal(state) -> None:
    """
    Рисует/обновляет маркеры робота и цели в обеих сценах (idle/drive), если заданы:
      state.robot_px, state.goal_px : Tuple[float,float]
    Удаляет маркеры, если соответствующих значений нет.
    """
    for scene_name, robot_attr, goal_attr in (
        ("_idle_scene",  "idle_robot_item",  "idle_goal_item"),
        ("_drive_scene", "drive_robot_item", "drive_goal_item"),
    ):
        sc = getattr(state, scene_name, None)
        if sc is None:
            continue

        # ROBOT
        if getattr(state, "robot_px", None) is not None:
            rx, ry = state.robot_px
            prev = getattr(state, robot_attr, None)
            new_item = _flag_item(sc, prev, QtCore.QPointF(rx, ry), "#ffffff")
            setattr(state, robot_attr, new_item)
        else:
            prev = getattr(state, robot_attr, None)
            if prev:
                sc.removeItem(prev)
            setattr(state, robot_attr, None)

        # GOAL
        if getattr(state, "goal_px", None) is not None:
            gx, gy = state.goal_px
            prev = getattr(state, goal_attr, None)
            new_item = _cross_item(sc, prev, QtCore.QPointF(gx, gy), "#d64545")
            setattr(state, goal_attr, new_item)
        else:
            prev = getattr(state, goal_attr, None)
            if prev:
                sc.removeItem(prev)
            setattr(state, goal_attr, None)


def redraw_route(state, ui: Optional[QtWidgets.QMainWindow] = None) -> None:
    """
    Рисует текущий маршрут (ГРАФ) в обеих сценах и обновляет HUD/статус-бар.
    Ожидает, что routing уже записал в state:
      - route_pts_px : последовательность точек пути в пикселях PNG (Y вниз),
      - route_len_m  : длина маршрута в метрах (для HUD, опционально).
    Также, если заданы robot_px / goal_px — нарисуем маркеры.
    """
    route_pts = getattr(state, "route_pts_px", None)

    # 1) Отрисовать линию маршрута
    for sc, holder in (
        (getattr(state, "_idle_scene", None),  "idle_route_item"),
        (getattr(state, "_drive_scene", None), "drive_route_item"),
    ):
        if sc is None:
            continue
        old = getattr(state, holder, None)
        new = draw_polyline_path(sc, old, route_pts or [], "#e53935", 3)
        setattr(state, holder, new)

    # 2) Маркеры робота/цели
    _draw_robot_and_goal(state)

    # 3) HUD/Status: пишем длину маршрута (если есть) или заглушку
    if getattr(state, "route_len_m", 0.0) > 0.0:
        hud_text = f"Маршрут: {state.route_len_m:.1f} м"
    else:
        hud_text = "Маршрут не задан"

    for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
        if sc is not None:
            update_hud_text(sc, hud_text)

    if ui and hasattr(ui, "statusBar"):
        try:
            ui.statusBar().showMessage(hud_text, 3000)
        except Exception:
            pass


# ======================================================================
# СОВМЕСТИМОСТЬ: старый хук под IDLE
# ======================================================================

def redraw_idle_overlays(state, scene: QtWidgets.QGraphicsScene) -> None:
    """
    Совместимый «старый» хук для IDLE.
    Раньше он рисовал по сплайну; теперь всегда работаем с ГРАФОМ:
      - фиксируем idle-сцену в state (если ещё не сохранена),
      - вызываем графовую отрисовку маршрута/маркеров.
    """
    if getattr(state, "_idle_scene", None) is None and scene is not None:
        state._idle_scene = scene
    redraw_route(state)