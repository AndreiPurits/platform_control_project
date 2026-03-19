# page_drive.py
# -*- coding: utf-8 -*-
from typing import Optional, Tuple
from PyQt5 import QtWidgets, QtCore, QtGui
from routing import set_goal_px, build_route_from_robot_to_goal
from graphics import redraw_route, redraw_markers, ensure_radar_scene, draw_radar_points
from robot_cmd import update_drive_panel
from lidar_udp_rx import LidarUdpReceiver
from radar_heatmap import RadarHeatmap

Point = Tuple[float, float]

class DrivePage(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__()
        self.ui = ui
        self.state = state

        self.pageDrive: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageDrive")
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")

        self._heatmap = None  # объявим поле, чтобы не ловить AttributeError
        # цетр «радара» в середину вьюпорта
        if self.radarViewDrive and self.radarViewDrive.scene():
            sc = self.radarViewDrive.scene()
            w = self.radarViewDrive.viewport().width()
            h = self.radarViewDrive.viewport().height()
            sc._radar_origin_px = (w * 0.5, h * 0.5)

        self.btnStartStop: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnSelectGoalDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalDrive")
    
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.clicked.connect(lambda: self._on_points([
                (0.0, 0.0), (0.2, 0.0), (0.0, 0.2), (-0.2, 0.0), (0.0, -0.2)
            ]))

        self.state.is_running = getattr(self.state, "is_running", False)
        self.state.select_goal_mode_drive = getattr(self.state, "select_goal_mode_drive", False)

        if self.btnStartStop:
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)
            self._refresh_startstop_caption()
        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_drive_pick_map)
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.toggled.connect(self._on_select_goal_drive_toggled)
            self._refresh_select_goal_caption()

        if self.mapViewDrive is not None:
            self.mapViewDrive.setMouseTracking(True)
            self.mapViewDrive.viewport().installEventFilter(self)
        if self.radarViewDrive:
            ensure_radar_scene(self.radarViewDrive)

  
            self._lidar_rx = LidarUdpReceiver(host="0.0.0.0", port=10000)
            self._lidar_rx.pointsReady.connect(self._on_points)
            self._lidar_rx.start()
            print("[DRIVE] lidar rx started on 10000")

        self._update_controls()


    def _ensure_heatmap(self):
        if self._heatmap is None and self.radarViewDrive and RadarHeatmap:
            self._heatmap = RadarHeatmap(self.radarViewDrive)
                    
    def shutdown(self):
        try:
            if getattr(self, "_lidar_rx", None):
                self._lidar_rx.stop()
                self._lidar_rx.wait(500)
        except Exception:
            pass
    def eventFilter(self, obj, ev):
        if self.mapViewDrive and obj is self.mapViewDrive.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewDrive.mapToScene(ev.pos())
                self._on_drive_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

    # верхний бар
    def _on_startstop_toggled(self, checked: bool):
        self.state.is_running = bool(checked)
        self._refresh_startstop_caption(); self._update_controls()
        print(f"[DRIVE] StartStop -> {'START' if self.state.is_running else 'STOP'}")

    def _on_drive_pick_map(self):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала Стоп → затем карта.")
            return
        if hasattr(self.ui, "to_maps"): self.ui.to_maps()

    def _on_select_goal_drive_toggled(self, enabled: bool):
        self.state.select_goal_mode_drive = bool(enabled)
        self._refresh_select_goal_caption()

    def _refresh_startstop_caption(self):
        if self.btnStartStop:
            self.btnStartStop.setText("Стоп" if self.state.is_running else "Пуск")

    def _refresh_select_goal_caption(self):
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setText("Цель" + (" (вкл.)" if self.state.select_goal_mode_drive else ""))

    def _update_controls(self):
        running = bool(self.state.is_running)
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not running)
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setEnabled(not running)
            if running and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
                self._refresh_select_goal_caption()

    # клик по карте
    def _on_drive_map_click(self, x_px: float, y_px: float):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Остановите движение")
            return
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите режим «Цель»")
            return
        if not getattr(self.state, "graph", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф карты не загружен")
            return

        # 1) цель — со снэпом к points_pixels.json
        try:
            set_goal_px((x_px, y_px), self.state)
        except Exception:
            self.state.goal_px = (float(x_px), float(y_px))

        # 2) перерисовать маркеры на обеих сценах
        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(self.state, sc)
                except Exception as e:
                    print("[DRIVE] redraw_markers error:", e)

        # 3) если известна позиция робота — строим маршрут, рисуем, обновляем панель
        if getattr(self.state, "robot_px", None):
            ok = False
            try:
                ok = build_route_from_robot_to_goal(self.state)
            except Exception as e:
                print("[DRIVE] route build error:", e)

            if ok:
                try:
                    redraw_route(self.state, self.ui)          # линия маршрута
                except Exception as e:
                    print("[DRIVE] redraw_route error:", e)
                try:
                    update_drive_panel(self.ui, self.state)    # lblMinDist и т.п.
                except Exception as e:
                    print("[DRIVE] panel update error:", e)
                    
    def _on_points(self, pts):
        if self.radarViewDrive and self.radarViewDrive.scene():
            try:
                from graphics import draw_radar_points
                draw_radar_points(self.radarViewDrive.scene(), pts, meters_to_px=40.0)
            except Exception:
                pass
            try:
                self._ensure_heatmap()
                if self._heatmap:
                    self._heatmap.update_from_xy(pts)
            except Exception as e:
                print("[LIDAR] heatmap error:", e)
        # панель справа (скорость/статус) обновляем как и раньше
        try:
            from robot_cmd import update_drive_panel
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

class _RadarResizeFilter(QtCore.QObject):
    def __init__(self, view):
        super().__init__(view)
        self.view = view
    def eventFilter(self, obj, ev):
        if obj is self.view.viewport() and ev.type() == QtCore.QEvent.Resize:
            sc = self.view.scene()
            if sc is not None:
                w = self.view.viewport().width() or 1
                h = self.view.viewport().height() or 1
                sc._radar_origin_px = (w * 0.5, h * 0.5)
        return False