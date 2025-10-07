# graphics.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtGui, QtCore

Point = Tuple[float, float]

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
    if not view: return
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
def _make_arrow() -> QtWidgets.QGraphicsPolygonItem:
    poly = QtGui.QPolygonF([
        QtCore.QPointF( 0, -12),  # нос
        QtCore.QPointF( 7,  10),
        QtCore.QPointF( 0,   6),
        QtCore.QPointF(-7,  10),
    ])
    item = QtWidgets.QGraphicsPolygonItem(poly)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ffffff")))
    item.setPen(QtGui.QPen(QtGui.QColor("#333333"), 1))
    item.setZValue(120)
    return item

def _make_goal_dot() -> QtWidgets.QGraphicsEllipseItem:
    r = 5
    item = QtWidgets.QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
    item.setBrush(QtGui.QBrush(QtGui.QColor("#ff3b30")))
    item.setPen(QtGui.QPen(QtGui.QColor("#222"), 1))
    item.setZValue(120)
    return item

def draw_robot_arrow(scene: QtWidgets.QGraphicsScene,
                     old_item: Optional[QtWidgets.QGraphicsItem],
                     pos: Optional[Point],
                     yaw_rad: Optional[float] = None) -> Optional[QtWidgets.QGraphicsItem]:
    if old_item is not None and old_item.scene() is scene:
        scene.removeItem(old_item)
    if not pos:
        return None
    item = _make_arrow()
    item.setPos(pos[0], pos[1])
    if yaw_rad is not None:
        item.setRotation(-yaw_rad * 180.0 / 3.141592653589793)  # ось Y вниз
    scene.addItem(item)
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

def redraw_markers(state, scene: QtWidgets.QGraphicsScene):
    # робот
    if scene is getattr(state, "_idle_scene", None):
        state.idle_robot_item = draw_robot_arrow(scene, getattr(state, "idle_robot_item", None), getattr(state, "robot_px", None), None)
        state.idle_goal_item  = draw_goal_marker(scene,  getattr(state, "idle_goal_item",  None), getattr(state, "goal_px",  None))
    elif scene is getattr(state, "_drive_scene", None):
        state.drive_robot_item = draw_robot_arrow(scene, getattr(state, "drive_robot_item", None), getattr(state, "robot_px", None), None)
        state.drive_goal_item  = draw_goal_marker(scene,  getattr(state, "drive_goal_item",  None), getattr(state, "goal_px",  None))

# (опционально) точки лидара
def draw_radar_points(scene: QtWidgets.QGraphicsScene, pts_m, meters_to_px: float = 40.0):
    # простая отрисовка точек без накопления
    for (x,y) in pts_m:
        xp = x*meters_to_px
        yp = -y*meters_to_px
        dot = QtWidgets.QGraphicsEllipseItem(xp-1, yp-1, 2, 2)
        dot.setBrush(QtGui.QBrush(QtGui.QColor("#00bcd4")))
        dot.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        dot.setZValue(50)
        scene.addItem(dot)