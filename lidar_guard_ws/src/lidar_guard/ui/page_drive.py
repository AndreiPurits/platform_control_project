# page_drive.py
# -*- coding: utf-8 -*-
"""
== API INDEX ================================================================
CLASS: DrivePage(ui, state)
- eventFilter(obj, ev) -> bool
    Ловит клики по mapViewDrive и делегирует в _on_drive_map_click().
- _on_startstop_toggled(checked: bool) -> None
    Старт/стоп (локально обновляет UI; публикацию в ROS оставили как TODO).
- _on_drive_pick_map() -> None
    Переход на страницу MAPS (только в стопе).
- _on_select_goal_drive_toggled(enabled: bool) -> None
    Режим выбора цели на DRIVE.
- _on_drive_map_click(x_px: float, y_px: float) -> None
    Логика клика: если robot_px известен — строим от него; иначе режим «два клика».
- _build_and_show_route(start_px: Point, goal_px: Point) -> None
    Построить маршрут (routing.build_route_snap_pixels) → перерисовать (graphics.redraw_route)
    → обновить HUD/status.
- _on_points(pts) -> None
    Отрисовка точек лидара (оставлено как раньше).
- update_robot_pose_px(x_px, y_px) -> None
    Обновление позиции робота в пикселях с обновлением HUD/status.
- shutdown() -> None
    Корректная остановка потоков при закрытии.
============================================================================
"""

import os
from typing import Optional, Tuple
from PyQt5 import QtWidgets, QtCore, QtGui

# граф/маршрут + отрисовка
from routing import build_route_snap_pixels, update_progress_text_for_robot
from graphics import redraw_route, update_hud_text

Point = Tuple[float, float]


class DrivePage(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__()
        self.ui = ui
        self.state = state

        # виджеты из .ui
        self.pageDrive: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageDrive")
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")

        self.btnStartStop: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnSelectGoalDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalDrive")

        # состояния
        self.state.is_running = getattr(self.state, "is_running", False)
        self.state.select_goal_mode_drive = getattr(self.state, "select_goal_mode_drive", False)

        # локальная «буферная» стартовая точка (если робота нет)
        self._pending_start_px: Optional[Point] = None

        # кнопки
        if self.btnStartStop:
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)
            self._refresh_startstop_caption()
        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_drive_pick_map)
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.toggled.connect(self._on_select_goal_drive_toggled)
            self._refresh_select_goal_caption()

        # клик мышью по карте DRIVE
        if self.mapViewDrive is not None:
            self.mapViewDrive.setMouseTracking(True)
            self.mapViewDrive.viewport().installEventFilter(self)

        self._update_controls()
        self._lidar_rx = None
        self._heatmap = getattr(self, "_heatmap", None)

    # --- eventFilter: ловим клик по DRIVE-карте ---
    def eventFilter(self, obj, ev):
        if self.mapViewDrive and obj is self.mapViewDrive.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewDrive.mapToScene(ev.pos())
                self._on_drive_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

    # --- верхний бар ---
    def _on_startstop_toggled(self, checked: bool):
        self.state.is_running = bool(checked)
        self._refresh_startstop_caption()
        self._update_controls()
        print(f"[DRIVE] StartStop -> {'START' if self.state.is_running else 'STOP'}")
        # TODO: publish в ROS2 команду на запуск/останов

    def _on_drive_pick_map(self):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала остановите движение (Стоп), затем выбирайте карту.")
            return
        if hasattr(self.ui, "to_maps"):
            self.ui.to_maps()

    def _on_estop_clicked(self):
        print("[DRIVE] E-STOP pressed")
        # TODO: publish аварийный стоп

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

    # --- клик по DRIVE карте ---
    def _on_drive_map_click(self, x_px: float, y_px: float):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Остановите движение, чтобы выбрать цель")
            return
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите режим «Цель»")
            return
        if getattr(self.state, "graph", None) is None or self.mapViewDrive is None or self.mapViewDrive.scene() is None:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф карты не загружен")
            return

        goal_px: Point = (x_px, y_px)
        robot_px: Optional[Point] = getattr(self.state, "robot_px", None)

        if robot_px is not None:
            start_px: Point = (float(robot_px[0]), float(robot_px[1]))
            self._build_and_show_route(start_px, goal_px)
            return

        # два клика: первый — старт, второй — цель
        if self._pending_start_px is None:
            self._pending_start_px = goal_px
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Старт выбран. Выберите точку цели.")
            return
        else:
            start_px = self._pending_start_px
            self._pending_start_px = None
            self._build_and_show_route(start_px, goal_px)
            return

    def _build_and_show_route(self, start_px: Point, goal_px: Point):
        ok = False
        try:
            ok = build_route_snap_pixels(start_px, goal_px, self.state)
        except Exception as e:
            print("[DRIVE] route build error:", e)
            ok = False

        if not ok:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Не удалось построить маршрут")
            return

        # отрисовать путь в обеих сценах
        try:
            redraw_route(self.state, self.ui)
        except Exception as e:
            print("[DRIVE] redraw_route error:", e)

        # обновить HUD / статус-бар
        try:
            text = update_progress_text_for_robot(start_px, self.state)
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    update_hud_text(sc, text)
            if hasattr(self.ui, "statusBar"):
                self.ui.statusBar().showMessage(text, 2500)
        except Exception as e:
            print("[DRIVE] HUD update error:", e)

    # --- точки лидара (оставлено) ---
    def _on_points(self, pts):
        """pts — список (x,y) в метрах, в системе (x вправо, y вверх)."""
        if self.radarViewDrive and self.radarViewDrive.scene():
            try:
                from graphics import draw_radar_points
                draw_radar_points(self.radarViewDrive.scene(), pts, meters_to_px=40.0)
            except Exception:
                pass
            try:
                if self._heatmap:
                    self._heatmap.update_from_xy(pts)
            except Exception:
                pass

    # --- публичный апдейт позиции робота (px) ---
    def update_robot_pose_px(self, x_px: float, y_px: float):
        try:
            self.state.robot_px = (float(x_px), float(y_px))
            text = update_progress_text_for_robot(self.state.robot_px, self.state)
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    update_hud_text(sc, text)
            if hasattr(self.ui, "statusBar"):
                self.ui.statusBar().showMessage(text, 1500)
        except Exception as e:
            print("[DRIVE] progress update failed:", e)

    # --- shutdown ---
    def shutdown(self):
        try:
            if getattr(self, "_lidar_rx", None) and self._lidar_rx.isRunning():
                self._lidar_rx.stop()
                self._lidar_rx.wait(500)
        except Exception:
            pass