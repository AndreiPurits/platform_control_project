# radar_heatmap.py
import math, time
from typing import Iterable, Optional, Sequence, Tuple
from PyQt5 import QtCore, QtGui, QtWidgets

class RadarHeatmap(QtCore.QObject):
    """
    Тепловая карта для QGraphicsView c фиксированным масштабом в метрах.
    Главное свойство: meters_span — сколько метров по ширине видим.
    Поворот сцены rotation_deg (по часовой), «машина» по центру.
    """
    def __init__(self,
                 view: QtWidgets.QGraphicsView,
                 meters_span: float = 12.0,
                 rotation_deg: float = 0.0,
                 decay: float = 0.92,
                 dot_radius_px: int = 2):
        super().__init__(view)
        self.view = view
        self.meters_span = float(meters_span)
        self.rotation_deg = float(0)
        self.decay = float(decay)
        self.dot_r = int(dot_radius_px)

        # сцена
        self.scene = view.scene() or QtWidgets.QGraphicsScene(view)
        if view.scene() is None:
            view.setScene(self.scene)

        view.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
        view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        view.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)

        # внутреннее состояние
        self._img_size_px = 0
        self._ppm = 1.0
        self._cx = 0.0
        self._cy = 0.0
        self._rot_rad = math.radians(0)

        self._heat_qimg: Optional[QtGui.QImage] = None
        self._heat_pm_item: Optional[QtWidgets.QGraphicsPixmapItem] = None
        self._grid_group: Optional[QtWidgets.QGraphicsItemGroup] = None
        self._robot_item: Optional[QtWidgets.QGraphicsItemGroup] = None
        self._robot_pixmap_item: Optional[QtWidgets.QGraphicsPixmapItem] = None
        self._trail_img: Optional[QtGui.QImage] = None
        self._trail_item: Optional[QtWidgets.QGraphicsPixmapItem] = None
        self._last_ts = time.perf_counter()
        self.half_life_s = 0.3  # полужизнь шлейфа, сек (можешь поменять)
        # реагируем на ресайз
        self._rf = _ResizeFilter(self._on_resized)
        view.viewport().installEventFilter(self._rf)

        self._recreate_canvas()

    # публичное
    def set_robot_pixmap(self, pm: QtGui.QPixmap):
        if self._robot_item:
            self.scene.removeItem(self._robot_item)
            self._robot_item = None
        self._robot_pixmap_item = None
        if pm.isNull():
            self._draw_robot_arrow()
            return
        item = self.scene.addPixmap(pm)
        item.setOffset(-pm.width()/2.0, -pm.height()/2.0)
        item.setPos(self._cx, self._cy)
        item.setRotation(0)  # «нос» вперёд
        self._robot_pixmap_item = item

    def update_from_xy(self, points_xy_m: Sequence[Tuple[float, float]]):
        # гарантируем, что есть куда рисовать
        self._ensure_canvas()
        if self._trail_img is None or self._trail_img.isNull():
            return

        TRAIL_COLOR = QtGui.QColor(255, 80, 0, 200)

        # --- 1) время с последнего кадра ---
        now = time.perf_counter()
        dt = max(0.0, now - self._last_ts)
        self._last_ts = now

        # коэффициент затухания (half-life)
        if self.half_life_s > 0:
            alpha_scale = 0.5 ** (dt / self.half_life_s)
        else:
            alpha_scale = 0.0  # мгновенное исчезновение

        # --- 2) затухание существующего шлейфа ---
        fade = QtGui.QPainter(self._trail_img)
        fade.setCompositionMode(QtGui.QPainter.CompositionMode_DestinationIn)
        fade.fillRect(self._trail_img.rect(),
                    QtGui.QColor(0, 0, 0, int(255 * alpha_scale)))
        fade.end()

        # --- 3) рисуем новые красные точки поверх ---
        if points_xy_m:
            p = QtGui.QPainter(self._trail_img)
            p.setRenderHint(QtGui.QPainter.Antialiasing, True)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(TRAIL_COLOR)
            p.setCompositionMode(QtGui.QPainter.CompositionMode_SourceOver)

            # границы видимой области (метры)
            vp = self.view.viewport().rect()
            half_x = self.meters_span * 0.5
            span_y_m = self.meters_span * (vp.height() / max(1, vp.width()))
            half_y = span_y_m * 0.5

            # поворот: rotation_deg трактуем как "по часовой" => мат. угол отрицательный
            cosr = math.cos(-math.radians(self.rotation_deg))
            sinr = math.sin(-math.radians(self.rotation_deg))

            for x_m, y_m in points_xy_m:
                xr = x_m * cosr - y_m * sinr
                yr = x_m * sinr + y_m * cosr

                if xr < -half_x or xr > half_x or yr < -half_y or yr > half_y:
                    continue

                x_px = self._cx + xr * self._ppm
                y_px = self._cy - yr * self._ppm  # метрический y вверх -> экран вниз

                p.drawEllipse(QtCore.QPointF(x_px, y_px), self.dot_r, self.dot_r)

            p.end()

        # --- 4) обновить item на сцене (исправлено: setPixmap) ---
        if self._trail_item:
            self._trail_item.setPixmap(QtGui.QPixmap.fromImage(self._trail_img))

    # внутреннее
    def _on_resized(self):
        self._recreate_canvas()
        
    def _ensure_canvas(self):
        """Создаёт холст шлейфа, если его ещё нет (например, до первого resize/show)."""
        if self._trail_img is None or self._trail_img.isNull():
            # берём квадрат по минимальной стороне viewport'а (как у тебя)
            vp = self.view.viewport().rect()
            s = max(200, min(vp.width(), vp.height()))
            if s <= 0:
                return
            # пересчёт метрик
            self._ppm = s / max(1e-6, self.meters_span)

            # центр viewport'а в координатах сцены
            center_scene = self.view.mapToScene(vp.center())
            self._cx, self._cy = center_scene.x(), center_scene.y()

            # создаём холст
            self._trail_img = QtGui.QImage(s, s, QtGui.QImage.Format_ARGB32_Premultiplied)
            self._trail_img.fill(QtCore.Qt.transparent)

            # позиция — так, чтобы холст оказался по центру viewport'а
            top_left_scene = self.view.mapToScene(vp.center() - QtCore.QPoint(s // 2, s // 2))
            if self._trail_item is None:
                self._trail_item = self.scene.addPixmap(QtGui.QPixmap.fromImage(self._trail_img))
            else:
                self._trail_item.setPixmap(QtGui.QPixmap.fromImage(self._trail_img))
            self._trail_item.setPos(top_left_scene)

            self._last_ts = time.perf_counter()

    def _recreate_canvas(self):
        vp = self.view.viewport().rect()
        s = max(200, min(vp.width(), vp.height()))  # квадрат без скроллов
        if s == self._img_size_px and self._heat_qimg is not None:
            self._fit()
            return

        self._img_size_px = s
        self._cx = s/2.0
        self._cy = s/2.0
        self._ppm = s / self.meters_span  # фиксированный масштаб [px/м]

        self.scene.setSceneRect(0, 0, s, s)

        self._heat_qimg = QtGui.QImage(s, s, QtGui.QImage.Format_ARGB32_Premultiplied)
        self._heat_qimg.fill(QtCore.Qt.transparent)

        if self._heat_pm_item is None:
            self._heat_pm_item = self.scene.addPixmap(QtGui.QPixmap.fromImage(self._heat_qimg))
            self._heat_pm_item.setZValue(-1)
        else:
            self._heat_pm_item.setPixmap(QtGui.QPixmap.fromImage(self._heat_qimg))

        if self._grid_group:
            self.scene.removeItem(self._grid_group)
            self._grid_group = None
        self._draw_grid()

        if self._robot_pixmap_item:
            self._robot_pixmap_item.setPos(self._cx, self._cy)
            self._robot_pixmap_item.setRotation(0)
        else:
            if self._robot_item:
                self.scene.removeItem(self._robot_item)
            self._draw_robot_arrow()

        self._fit()

    def _fit(self):
        self.view.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)


    def _draw_grid(self):
        items = []
        pen_grid = QtGui.QPen(QtGui.QColor("#dcdcdc")); pen_grid.setWidth(1); pen_grid.setCosmetic(True)
        pen_axis = QtGui.QPen(QtGui.QColor("#bbbbbb")); pen_axis.setWidth(1); pen_axis.setCosmetic(True)

        # оси
        items.append(self.scene.addLine(0, self._cy, self._img_size_px, self._cy, pen_axis))
        items.append(self.scene.addLine(self._cx, 0, self._cx, self._img_size_px, pen_axis))

        # концентрические круги каждый метр
        max_r_m = self.meters_span * 0.5
        r = 1.0
        while r <= max_r_m + 1e-6:
            rr = r * self._ppm
            items.append(self.scene.addEllipse(self._cx - rr, self._cy - rr, 2*rr, 2*rr, pen_grid))
            r += 1.0

        self._grid_group = self.scene.createItemGroup(items)
        self._grid_group.setZValue(10)

        # подписи (1м, 2м…)
        font = QtGui.QFont(); font.setPointSize(8)
        for i in range(1, int(max_r_m)+1):
            y = self._cy - i * self._ppm
            t = self.scene.addText(f"{i} m", font)
            t.setDefaultTextColor(QtGui.QColor("#888"))
            t.setPos(self._cx + 4, y - 8)
            t.setZValue(11)

    def _draw_robot_arrow(self):
        pen = QtGui.QPen(QtGui.QColor("#222")); pen.setWidth(2); pen.setCosmetic(True)
        brush = QtGui.QBrush(QtGui.QColor("#ffffff"))
        sz = 18
        poly = QtGui.QPolygonF([
            QtCore.QPointF(0, -sz),
            QtCore.QPointF(sz*0.7, sz),
            QtCore.QPointF(0, sz*0.4),
            QtCore.QPointF(-sz*0.7, sz),
        ])
        item = self.scene.addPolygon(poly, pen, brush)
        item.setPos(self._cx, self._cy)
        item.setRotation(0)
        item.setZValue(20)
        self._robot_item = item


class _ResizeFilter(QtCore.QObject):
    def __init__(self, cb): super().__init__(); self.cb = cb
    def eventFilter(self, obj, ev):
        if ev.type() == QtCore.QEvent.Resize:
            QtCore.QTimer.singleShot(0, self.cb)
        return False