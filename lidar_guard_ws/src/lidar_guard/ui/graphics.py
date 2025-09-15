# graphics.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtGui, QtCore
from state import AppState


# ----------- подготовка видов -----------

def fit_view_to_pixmap(view: QtWidgets.QGraphicsView, pixmap_item: QtWidgets.QGraphicsPixmapItem):
    if not view or not pixmap_item: return
    br = pixmap_item.boundingRect()
    if br.isEmpty(): return
    view.fitInView(br, QtCore.Qt.KeepAspectRatio)

# ----------- загрузка карты -----------
def show_map_on_views(path: str,
                      idle_view: QtWidgets.QGraphicsView,
                      drive_view: QtWidgets.QGraphicsView,
                      state: AppState):
    pm = QtGui.QPixmap(path)
    if pm.isNull(): return

    # Idle
    if idle_view and idle_view.scene():
        sc = idle_view.scene()
        sc.clear()
        sc.setSceneRect(0, 0, pm.width(), pm.height())
        state.idle_map_pixmap_item = sc.addPixmap(pm)
        fit_view_to_pixmap(idle_view, state.idle_map_pixmap_item)
        # перерисуем оверлеи из состояния
        redraw_idle_overlays(state, idle_view.scene())

    # Drive
    if drive_view and drive_view.scene():
        sc = drive_view.scene()
        sc.clear()
        sc.setSceneRect(0, 0, pm.width(), pm.height())
        state.drive_map_pixmap_item = sc.addPixmap(pm)
        fit_view_to_pixmap(drive_view, state.drive_map_pixmap_item)
        redraw_drive_overlays(state, drive_view.scene())

# ----------- маркеры/маршрут -----------
def _flag_item(scene: QtWidgets.QGraphicsScene, old_item, pos: QtCore.QPointF, color: str):
    if old_item is not None:
        scene.removeItem(old_item)
    pen = QtGui.QPen(QtGui.QColor("#333")); pen.setWidth(2)
    mast = scene.addLine(pos.x(), pos.y(), pos.x(), pos.y()-20, pen)
    flag = scene.addPolygon(QtGui.QPolygonF([
        QtCore.QPointF(pos.x(), pos.y()-20),
        QtCore.QPointF(pos.x()+16, pos.y()-16),
        QtCore.QPointF(pos.x(), pos.y()-12),
    ]), QtGui.QPen(QtGui.QColor("#333")), QtGui.QBrush(QtGui.QColor(color)))
    base = scene.addEllipse(pos.x()-2, pos.y()-2, 4, 4, pen, QtGui.QBrush(QtGui.QColor("#333")))
    return scene.createItemGroup([mast, flag, base])

def _cross_item(scene, old_item, pos: QtCore.QPointF, color="#d64545"):
    if old_item is not None:
        scene.removeItem(old_item)
    size = 10
    pen  = QtGui.QPen(QtGui.QColor(color)); pen.setWidth(2)
    l1 = scene.addLine(pos.x()-size, pos.y(), pos.x()+size, pos.y(), pen)
    l2 = scene.addLine(pos.x(), pos.y()-size, pos.x(), pos.y()+size, pen)
    dot= scene.addEllipse(pos.x()-3, pos.y()-3, 6, 6, pen, QtGui.QBrush(QtGui.QColor(color)))
    return scene.createItemGroup([l1, l2, dot])

def draw_polyline_path(scene: QtWidgets.QGraphicsScene, old_item, pts: List[Tuple[float,float]], color="#e53935"):
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pts: return None
    p = QtGui.QPainterPath(QtCore.QPointF(pts[0][0], pts[0][1]))
    for (x,y) in pts[1:]:
        p.lineTo(x, y)
    pen = QtGui.QPen(QtGui.QColor(color)); pen.setWidth(3); pen.setCosmetic(True)
    return scene.addPath(p, pen)

# ----------- перерисовка из состояния (по индексам) -----------
def redraw_idle_overlays(state: AppState, scene: QtWidgets.QGraphicsScene):
    # флаг робота
    if state.robot_idx is not None and state.spline_polyline:
        x,y = state.spline_polyline[state.robot_idx]
        state.idle_robot_item = _flag_item(scene, state.idle_robot_item, QtCore.QPointF(x,y), color="#ffffff")
    else:
        if state.idle_robot_item: scene.removeItem(state.idle_robot_item); state.idle_robot_item = None

    # цель
    if state.goal_idx is not None and state.spline_polyline:
        x,y = state.spline_polyline[state.goal_idx]
        state.idle_goal_item = _cross_item(scene, state.idle_goal_item, QtCore.QPointF(x,y), color="#d64545")
    else:
        if state.idle_goal_item: scene.removeItem(state.idle_goal_item); state.idle_goal_item = None

    # маршрут
    if state.spline_polyline and state.robot_idx is not None and state.goal_idx is not None:
        path_pts = _path_from_indices(state.spline_polyline, state.robot_idx, state.goal_idx)
        state.idle_route_item = draw_polyline_path(scene, state.idle_route_item, path_pts, color="#e53935")
    else:
        if state.idle_route_item: scene.removeItem(state.idle_route_item); state.idle_route_item = None

def redraw_drive_overlays(state: AppState, scene: QtWidgets.QGraphicsScene):
    if state.robot_idx is not None and state.spline_polyline:
        x,y = state.spline_polyline[state.robot_idx]
        state.drive_robot_item = _flag_item(scene, state.drive_robot_item, QtCore.QPointF(x,y), color="#ffffff")
    else:
        if state.drive_robot_item: scene.removeItem(state.drive_robot_item); state.drive_robot_item = None

    if state.goal_idx is not None and state.spline_polyline:
        x,y = state.spline_polyline[state.goal_idx]
        state.drive_goal_item = _cross_item(scene, state.drive_goal_item, QtCore.QPointF(x,y), color="#d64545")
    else:
        if state.drive_goal_item: scene.removeItem(state.drive_goal_item); state.drive_goal_item = None

    if state.spline_polyline and state.robot_idx is not None and state.goal_idx is not None:
        path_pts = _path_from_indices(state.spline_polyline, state.robot_idx, state.goal_idx)
        state.drive_route_item = draw_polyline_path(scene, state.drive_route_item, path_pts, color="#e53935")
    else:
        if state.drive_route_item: scene.removeItem(state.drive_route_item); state.drive_route_item = None

def _path_from_indices(poly: List[Tuple[float,float]], ia: int, ib: int) -> List[Tuple[float,float]]:
    # локальный простой расчёт (чтобы не тянуть routing сюда)
    if ia == ib: return [poly[ia]]
    if ia < ib:
        forward  = poly[ia:ib+1]
        backward = poly[ib:] + poly[:ia+1]
    else:
        forward  = poly[ia:] + poly[:ib+1]
        backward = poly[ib:ia+1]
    return forward if len(forward) <= len(backward) else backward

# Удобно вызывать после смены экрана или выбора карты:
def refresh_all_overlays(state: AppState,
                         idle_view: QtWidgets.QGraphicsView,
                         drive_view: QtWidgets.QGraphicsView):
    if idle_view and idle_view.scene():
        redraw_idle_overlays(state, idle_view.scene())
    if drive_view and drive_view.scene():
        redraw_drive_overlays(state, drive_view.scene())
        
def ensure_scene(view: QtWidgets.QGraphicsView):
    if not view: return
    if not isinstance(view.scene(), QtWidgets.QGraphicsScene):
        sc = QtWidgets.QGraphicsScene(view)
        view.setScene(sc)
    # дефолтный прямоугольник сцены (в пикселях)
    view.scene().setSceneRect(0, 0, 800, 600)

def prepare_view(view: QtWidgets.QGraphicsView):
    if not view: return
    view.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
    view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setDragMode(QtWidgets.QGraphicsView.NoDrag)

def clear_layer(scene: QtWidgets.QGraphicsScene, tag: str):
    for it in list(scene.items()):
        try:
            if it.data(0) == tag:
                scene.removeItem(it)
        except Exception:
            pass

def draw_radar_points(scene: QtWidgets.QGraphicsScene, pts: List[Tuple[float,float]],
                      meters_to_px: float = 40.0, tag: str = "radar"):
    """
    pts: в метрах, (x вправо, y вверх).
    meters_to_px: 40 => 1 м = 40 пикселей.
    """
    if scene is None: return
    clear_layer(scene, tag)

    # центр сцены
    br = scene.sceneRect()
    cx = br.center().x()
    cy = br.center().y()

    # оси
    ax_pen = QtGui.QPen(QtGui.QColor("#cccccc")); ax_pen.setStyle(QtCore.Qt.DotLine)
    ax_h = scene.addLine(br.left(), cy, br.right(), cy, ax_pen); ax_h.setData(0, tag)
    ax_v = scene.addLine(cx, br.top(), cx, br.bottom(), ax_pen); ax_v.setData(0, tag)

    if not pts: return

    pen = QtGui.QPen(QtGui.QColor("#00aaff")); pen.setWidth(0)
    brush = QtGui.QBrush(QtGui.QColor("#00aaff"))
    r = 2.0  # радиус точки в пикселях

    for (mx, my) in pts:
        x = cx + mx * meters_to_px
        y = cy - my * meters_to_px  # инвертируем ось Y под экран
        dot = scene.addEllipse(x-r, y-r, 2*r, 2*r, pen, brush)
        dot.setData(0, tag)
