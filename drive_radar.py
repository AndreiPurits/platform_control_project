# -*- coding: utf-8 -*-
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets, QtGui

from lidar_udp_rx import LidarUdpReceiver
from graphics import _lidar
from robot_cmd import motors_set
import numpy as np

PPM = 80.0  # pixels per meter


class RadarController(QtCore.QObject):
    """Лидарный радар в QGraphicsView (UDP -> pointsReady -> dots)."""

    def __init__(
        self,
        ui: QtWidgets.QMainWindow,
        state,
        radarViewDrive: QtWidgets.QGraphicsView = None,
        radar_view: QtWidgets.QGraphicsView = None,
        host: str = "127.0.0.1",
        port_front: int = 10000,
        port_rear: int = 10001,
    ):
        super().__init__(ui)
        self.ui = ui
        self.state = state
        self.view = radarViewDrive if radarViewDrive is not None else radar_view
        self._grid_group = None
        # ports (как в старом драйве)
        self.host = host
        self.port_front = int(port_front)
        self.port_rear = int(port_rear)

        self._scene: QtWidgets.QGraphicsScene | None = None
        self._dots_group: QtWidgets.QGraphicsItemGroup | None = None
        self._cam_pixmap_item = None

        self._rx_front: LidarUdpReceiver | None = None
        self._rx_rear: LidarUdpReceiver | None = None

        if self.view:
            self._ensure_scene()
            self._draw_grid()

    # ------------------------------------------------------------------
    # Public API (совместимость с page_drive.py)
    # ------------------------------------------------------------------
    def start(self):
        """Alias для setup(), чтобы page_drive.py не ломался."""
        return self.setup()

    def setup(self):
        # не стартуем второй раз
        if self._rx_front and self._rx_front.isRunning():
            pass
        else:
            self._start_front()

        if self._rx_rear and self._rx_rear.isRunning():
            pass
        else:
            self._start_rear()
    def stop(self):
        for rx, handler in (
            (self._rx_front, self._on_front),
            (self._rx_rear, self._on_rear),
        ):
            if not rx:
                continue
            try:
                if rx.isRunning():
                    try:
                        rx.pointsReady.disconnect(handler)
                    except Exception:
                        pass
                    rx.stop()
                    rx.wait(500)
            except Exception:
                pass
        self._rx_front = None
        self._rx_rear = None

    def set_mode(self, mode: str):
        """
        camera: скрываем радарный оверлей (сетка + точки), ничего не удаляем
        radar : показываем оверлей, гарантируем корректную сцену/rect/fit
        """
        mode = "camera" if mode == "camera" else "radar"
        self.state.video_mode = mode

        if mode == "camera":
            # просто прячем радарные элементы, чтобы камера могла рисоваться поверх
            self.set_radar_overlay_visible(False)

            # на всякий случай: если камера-слой есть, радар его не трогает
            return

        # -------------------------
        # mode == "radar"
        # -------------------------
        self.set_radar_overlay_visible(True)

        # гарантируем, что _scene соответствует текущей view.scene()
        self._ensure_scene()
        sc = self._scene
        if sc is None:
            return

        # нормальный rect для радара
        r_def = QtCore.QRectF(-300, -300, 600, 600)
        sc.setSceneRect(r_def)

        # если сетку снесли/она потеряла сцену — восстановим
        if self._grid_group is None or self._grid_group.scene() is None:
            self._draw_grid()

        if self._dots_group is not None:
            try:
                sc_item = self._dots_group.scene()
                if sc_item is not None:
                    sc_item.removeItem(self._dots_group)
            except Exception:
                pass
            self._dots_group = None

        # fit
        try:
            if self.view:
                self.view.resetTransform()
                self.view.fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
        except Exception:
            pass

    def set_cam_pixmap_item(self, item):
        """Если VideoController хочет отдать ссылку на item."""
        self._cam_pixmap_item = item

    def clear_scene(self):
        if self._scene:
            self._scene.clear()
        self._dots_group = None

    def restore_radar_view(self):
        if not self.view:
            return

        self._ensure_scene()
        sc = self._scene
        if sc is None:
            return

        r_def = QtCore.QRectF(-300, -300, 600, 600)
        sc.setSceneRect(r_def)

        # если сетка пропала — восстановим
        if self._grid_group is None or self._grid_group.scene() is None:
            self._draw_grid()

        # fit
        try:
            self.view.resetTransform()
            self.view.fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _ensure_scene(self):
        self._scene = self.view.scene() if self.view else None
        return self._scene

    def _draw_grid(self):
        sc = self._scene
        if sc is None:
            return

        r = sc.sceneRect()
        items = []

        pen_axis = QtGui.QPen(QtGui.QColor("#bbbbbb"))
        pen_axis.setWidth(1)
        pen_axis.setCosmetic(True)
        items.append(sc.addLine(r.left(), 0, r.right(), 0, pen_axis))
        items.append(sc.addLine(0, r.top(), 0, r.bottom(), pen_axis))

        pen_grid = QtGui.QPen(QtGui.QColor("#e0e0e0"))
        pen_grid.setWidth(1)
        pen_grid.setCosmetic(True)

        max_r_px = min(r.width(), r.height()) * 0.5
        step_px = PPM
        rad = step_px
        while rad <= max_r_px + 1e-6:
            items.append(sc.addEllipse(-rad, -rad, 2 * rad, 2 * rad, pen_grid))
            rad += step_px

        grp = sc.createItemGroup(items)
        grp.setZValue(1)
        self._grid_group = grp

    def set_radar_overlay_visible(self, vis: bool):
        for it in (getattr(self, "_grid_group", None), getattr(self, "_dots_group", None)):
            if it is None:
                continue
            try:
                it.setVisible(bool(vis))
            except Exception:
                pass
    def _start_front(self):
        # уже работает — ничего не делаем
        if self._rx_front and self._rx_front.isRunning():
            return

        # если объект есть, но не работает — подчистим
        if self._rx_front:
            try:
                self._rx_front.pointsReady.disconnect(self._on_front)
            except Exception:
                pass
            try:
                self._rx_front.stop()
                self._rx_front.wait(200)
            except Exception:
                pass
            self._rx_front = None

        try:
            self._rx_front = LidarUdpReceiver(host=self.host, port=self.port_front, parent=self)
            self._rx_front.pointsReady.connect(self._on_front, QtCore.Qt.QueuedConnection)
            self._rx_front.start()
            print(f"[RADAR] FRONT listening on {self.host}:{self.port_front}", flush=True)
        except Exception as e:
            print("[RADAR] front start error:", e, flush=True)
            self._rx_front = None
    def _start_rear(self):
        # 1) уже работает — ничего не делаем
        if self._rx_rear and self._rx_rear.isRunning():
            return

        # 2) если объект есть, но поток не запущен/умер — подчистим
        if self._rx_rear:
            try:
                self._rx_rear.pointsReady.disconnect(self._on_rear)
            except Exception:
                pass
            try:
                self._rx_rear.stop()
                self._rx_rear.wait(200)
            except Exception:
                pass
            self._rx_rear = None

        # 3) стартуем новый receiver
        try:
            self._rx_rear = LidarUdpReceiver(host=self.host, port=self.port_rear, parent=self)
            self._rx_rear.pointsReady.connect(self._on_rear, QtCore.Qt.QueuedConnection)
            self._rx_rear.start()
            print(f"[RADAR] REAR  listening on {self.host}:{self.port_rear}", flush=True)
        except Exception as e:
            print("[RADAR] rear start error:", e, flush=True)
            self._rx_rear = None
        
    def _draw_points(self, pts_xy_m):
        if getattr(self.state, "video_mode", "radar") == "camera":
            return

        sc = self._scene
        if sc is None:
            return

        r_def = QtCore.QRectF(-300, -300, 600, 600)
        if sc.sceneRect() != r_def:
            self.restore_radar_view()

        if self._dots_group is not None:
            try:
                sc.removeItem(self._dots_group)
            except Exception:
                pass
            self._dots_group = None

        items = []
        pen = QtGui.QPen(QtCore.Qt.NoPen)
        brush = QtGui.QBrush(QtGui.QColor("#00bcd4"))
        d = 2
        r = sc.sceneRect()

        for (x_m, y_m) in pts_xy_m:
            xp = x_m * PPM
            yp = -y_m * PPM
            if not r.contains(xp, yp):
                continue
            dot = sc.addEllipse(xp - d, yp - d, 2 * d, 2 * d, pen, brush)
            dot.setZValue(5)
            items.append(dot)

        if items:
            self._dots_group = sc.createItemGroup(items)
            self._dots_group.setZValue(5)

        # если вдруг висит камера item — прячем (камера управляется VideoController)
        if self._cam_pixmap_item:
            try:
                self._cam_pixmap_item.setVisible(False)
            except Exception:
                pass

    def _on_front(self, pts_xy_m):
        if not pts_xy_m:
            return

        st = self.state
        st._lidar_front_last_pts = list(pts_xy_m)
        st._lidar_last_pts = list(pts_xy_m)

        prev_stop = bool(getattr(st, "safety_stop", False))

        # ЕДИНСТВЕННЫЙ расчёт стопа (в graphics.py)
        try:
            from graphics import lidar_front_stop
            lidar_front_stop(self.ui, st)
        except Exception as e:
            print("[RADAR FRONT] lidar_front_stop error:", e, flush=True)

        cur_stop = bool(getattr(st, "safety_stop", False))

        # resume если препятствие исчезло и режим RUN
        if prev_stop and not cur_stop and getattr(st, "is_running", False):
            try:
                base = int(getattr(st, "pwm_base_us", 1700) or 1700)
                base = max(1500, min(2000, base))
                print(f"[RADAR FRONT] obstacle cleared -> resume {base}", flush=True)
                motors_set(st, base, base, None)
            except Exception as e:
                print("[RADAR FRONT] resume error:", e, flush=True)

        self._draw_points(pts_xy_m)

    def _on_rear(self, pts_xy_m):
        if not pts_xy_m:
            return
        self.state._lidar_rear_last_pts = list(pts_xy_m)
        try:
            from graphics import lidar_rear_stop
            lidar_rear_stop(self.ui, self.state)
        except Exception as e:
            print("[RADAR REAR] lidar_rear_stop error:", e, flush=True)

