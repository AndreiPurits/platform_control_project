# -*- coding: utf-8 -*-
"""
== API INDEX ================================================================
PUBLIC:
- ensure_scene(view, pixmap=None) -> QGraphicsScene
    Создаёт/возвращает сцену у QGraphicsView. Если дан pixmap — кладёт как фон.
- prepare_view(view) -> None
    Базовая настройка QGraphicsView (зум/смягчение/якоря/скроллбары).
- show_map_on_views(png_path, idle_view, drive_view, state) -> None
    Грузит PNG в оба вида, создаёт HUD, сбрасывает старые items маршрута.
- ensure_hud(scene) -> (group, text_item)
    Создаёт (если нет) HUD-плашку «Маршрут …».
- update_hud_text(scene, text) -> None
    Обновляет текст HUD.
- draw_polyline_path(scene, old_item, pts, color="#e53935", width=3)
    Рисует/перерисовывает линию маршрута.
- redraw_route(state, ui=None) -> None
    Берёт state.route_pts_px/route_len_m и рисует маршрут в обеих сценах + HUD.

INTERACTION:
- attach_click_router(view, state, ui, on_after_route=None) -> QObject
    Вешает фильтр на view.viewport(): два клика → построить маршрут по графу.

NOTES:
- Всё ориентировано на графовую логику: маршруты готовит routing.build_route_snap_pixels,
  а этот модуль отвечает только за визуализацию и клики.
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


def show_map_on_views(png_path: str,
                      idle_view: Optional[QtWidgets.QGraphicsView],
                      drive_view: Optional[QtWidgets.QGraphicsView],
                      state) -> None:
    """
    Грузит PNG и показывает в обоих QGraphicsView.
    - Создаёт/обновляет сцены.
    - Поднимает HUD.
    - Сбрасывает прошлые items маршрута в state.
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

    state._idle_scene = sc_idle
    state._drive_scene = sc_drive
    state.idle_route_item = None
    state.drive_route_item = None


# ======================================================================
# HUD
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
# ОТРИСОВКА МАРШРУТА
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


def redraw_route(state, ui: Optional[QtWidgets.QMainWindow] = None) -> None:
    """
    Рисует текущий маршрут (px) в обеих сценах и обновляет HUD/статус-бар.
    Ожидает, что state.route_pts_px и state.route_len_m подготовлены routing’ом.
    """
    for sc, holder in (
        (getattr(state, "_idle_scene", None),  "idle_route_item"),
        (getattr(state, "_drive_scene", None), "drive_route_item"),
    ):
        if sc is None:
            continue
        item = getattr(state, holder, None)
        new_item = draw_polyline_path(sc, item, getattr(state, "route_pts_px", []) or [], "#e53935", 3)
        setattr(state, holder, new_item)

    # HUD + статус-бар
    if getattr(state, "route_len_m", 0.0) > 0.0:
        hud_text = f"Маршрут: {state.route_len_m:.1f} м"
    else:
        hud_text = "Маршрут не задан"

    for sc in (getattr(state, "_idle_scene", None), getattr(state, "_drive_scene", None)):
        if sc is not None:
            try:
                update_hud_text(sc, hud_text)
            except Exception:
                pass

    if ui and hasattr(ui, "statusBar"):
        try:
            ui.statusBar().showMessage(hud_text, 3000)
        except Exception:
            pass


# ======================================================================
# ВЗАИМОДЕЙСТВИЕ: два клика → построить маршрут
# ======================================================================

class _ClickRouter(QtCore.QObject):
    """
    Фильтр событий мыши для QGraphicsView:
    два клика → построение маршрута через routing.build_route_snap_pixels.
    Важно: фильтр вешается на view.viewport(), т.к. клики приходят именно туда.
    """
    def __init__(self, view: QtWidgets.QGraphicsView, state, ui, on_after_route=None):
        super().__init__(view)
        self.view = view
        self.state = state
        self.ui = ui
        self._pending: List[Point] = []
        self.on_after_route = on_after_route

    def eventFilter(self, obj, ev):
        # Принимаем события как от viewport, так и от самого view — на всякий случай.
        is_target = (obj is self.view) or (self.view and obj is self.view.viewport())
        if is_target and ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
            # граф должен быть загружен
            if getattr(self.state, "graph", None) is None:
                QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф карты не загружен")
                return True

            sp = self.view.mapToScene(ev.pos())
            self._pending.append((float(sp.x()), float(sp.y())))

            if len(self._pending) == 1:
                # подсказка после первого клика
                if self.ui and hasattr(self.ui, "statusBar"):
                    self.ui.statusBar().showMessage("Старт выбран. Выберите точку цели.", 1500)
                return True

            if len(self._pending) >= 2:
                a, b = self._pending[0], self._pending[1]
                self._pending.clear()
                try:
                    from routing import build_route_snap_pixels, update_progress_text_for_robot
                    ok = build_route_snap_pixels(a, b, self.state)
                    if ok:
                        redraw_route(self.state, self.ui)
                        # стартовое «пройдено/осталось» от точки А
                        try:
                            text = update_progress_text_for_robot(a, self.state)
                            for sc in (getattr(self.state, "_idle_scene", None),
                                       getattr(self.state, "_drive_scene", None)):
                                if sc is not None:
                                    update_hud_text(sc, text)
                            if self.ui and hasattr(self.ui, "statusBar"):
                                self.ui.statusBar().showMessage(text, 2000)
                        except Exception:
                            pass
                        if self.on_after_route:
                            try:
                                self.on_after_route()
                            except Exception:
                                pass
                    else:
                        QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Не удалось построить маршрут")
                except Exception as e:
                    print("[GRAPHICS] route build error:", e)
            return True
        return False


def attach_click_router(view: Optional[QtWidgets.QGraphicsView], state, ui, on_after_route=None):
    """
    Ставит фильтр событий на view.viewport(), чтобы пара кликов строила маршрут по графу.
    Возвращает ссылку на фильтр (держим её у view, чтобы GC не прибрал).
    """
    if view is None:
        return None
    cr = _ClickRouter(view, state, ui, on_after_route=on_after_route)
    # важно: именно viewport получает клики мыши
    view.viewport().installEventFilter(cr)
    # храним ссылку на объект в экземпляре view, чтобы не уничтожился
    if not hasattr(view, "_click_router"):
        view._click_router = []
    view._click_router.append(cr)
    return cr


def prepare_view(view: QtWidgets.QGraphicsView):
    """
    Минимальная «приятная» конфигурация QGraphicsView для карт:
    - смягчение/антиалиасинг
    - якоря трансформации по центру
    - скрытые скроллбары
    - запрет перетаскивания (если нужно — поменяй на ScrollHandDrag)
    """
    if not view:
        return
    view.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
    view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
    view.setDragMode(QtWidgets.QGraphicsView.NoDrag)