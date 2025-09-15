# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore, QtGui
from state import AppState
from radar_heatmap import RadarHeatmap
from graphics import (
    ensure_scene, prepare_view,
    redraw_drive_overlays,         # рисует флаги/маршрут на DRIVE
    draw_radar_points,             # рисует облако точек лидара
)
from routing import nearest_index
from lidar_udp_rx import LidarUdpReceiver


class DrivePage:
    """Экран DRIVE: верхний бар + сплит Карта|Радар, выбор цели со сцены и приём лидара по UDP."""
    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState):
        self.ui = ui
        self.state = state

        # --- Виджеты карты/радара ---
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")
        ensure_scene(self.mapViewDrive);  prepare_view(self.mapViewDrive)
        ensure_scene(self.radarViewDrive);prepare_view(self.radarViewDrive)
        self._heatmap = RadarHeatmap(
            self.radarViewDrive,
            meters_span=12.0,   # тут задаёшь «сколько метров по ширине»
            rotation_deg=90.0,  # развернуть по часовой
            decay=0.92,
            dot_radius_px=2
        )
        # --- Кнопки верхнего бара ---
        self.btnStartStop: QtWidgets.QPushButton      = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton      = ui.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnEStop: QtWidgets.QPushButton          = ui.findChild(QtWidgets.QPushButton, "btnEStop")
        self.btnSelectGoalDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalDrive")  # опционально

        # Старт/Стоп (если логика не в Main — сделаем здесь, мягко)
        if self.btnStartStop:
            self.btnStartStop.setCheckable(True)
            self.btnStartStop.setChecked(bool(self.state.is_running))
            self._refresh_startstop_caption()
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)

        # Кнопка «Карты» — доступна только при Стоп
        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_drive_pick_map)

        # Аварийный стоп (заглушка на публикацию)
        if self.btnEStop:
            self.btnEStop.clicked.connect(self._on_estop_clicked)

        # Режим выбора цели в DRIVE — кликом по карте (только при Стоп)
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setCheckable(True)
            self.btnSelectGoalDrive.setChecked(bool(self.state.select_goal_mode_drive))
            self._refresh_select_goal_caption()
            self.btnSelectGoalDrive.toggled.connect(self._on_select_goal_drive_toggled)

        # Перехват кликов по DRIVE-карте
        if self.mapViewDrive:
            self.mapViewDrive.setMouseTracking(True)
            self._drive_click_filter = self._ClickFilter(self._on_drive_map_click)
            self.mapViewDrive.viewport().installEventFilter(self._drive_click_filter)

        # --- Приём лидара по UDP и отрисовка точек в radarViewDrive ---
        # Храним поток в атрибуте, чтобы GC не прибил; на выходе корректно гасим.
        self._lidar_rx = LidarUdpReceiver('127.0.0.1', 9999, parent=self.ui)
        self._lidar_rx.pointsReady.connect(self._on_points)
        self._lidar_rx.start()

        # Синхронизируем доступность контролов
        self._update_controls()

        # Если уже есть выбрана карта/цель/поза — перерисуем оверлеи
        if self.mapViewDrive and self.mapViewDrive.scene():
            redraw_drive_overlays(self.state, self.mapViewDrive.scene())

    # -------------------- внутренняя утилита: фильтр кликов --------------------
    class _ClickFilter(QtCore.QObject):
        def __init__(self, on_click):
            super().__init__()
            self.on_click = on_click

        def eventFilter(self, obj, ev):
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                self.on_click(float(ev.pos().x()), float(ev.pos().y()))
                return True
            return False
# -------------------- обработчики верхнего бара --------------------
    def _on_startstop_toggled(self, checked: bool):
        """Локально поддерживаем состояние, чтобы UI был консистентен,
        даже если основная логика старт/стоп живёт в Main."""
        self.state.is_running = bool(checked)
        self._refresh_startstop_caption()
        self._update_controls()
        print(f"[DRIVE] StartStop -> {'START' if self.state.is_running else 'STOP'}")
        # TODO: publish в ROS2 команду на запуск/останов

    def _on_drive_pick_map(self):
        """Кнопка 'Карты' на DRIVE: разрешаем переход в MAPS только при Стоп."""
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала остановите движение (Стоп), затем выбирайте карту.")
            return
        # просим главный вид переключить страницу (если у него есть метод)
        if hasattr(self.ui, "to_maps"):
            self.ui.to_maps()

    def _on_estop_clicked(self):
        print("[DRIVE] E-STOP pressed")
        # TODO: publish аварийный стоп в ROS2

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
        """Включить/выключить элементы на DRIVE в зависимости от self.state.is_running."""
        running = bool(self.state.is_running)
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not running)
        if self.btnSelectGoalDrive:
            # Режим выбора цели допустим только когда стоим
            self.btnSelectGoalDrive.setEnabled(not running)
            if running and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
                self._refresh_select_goal_caption()

    # -------------------- клики по карте DRIVE (выбор цели) --------------------
    def _on_drive_map_click(self, x_px: float, y_px: float):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Остановите движение, чтобы выбрать цель")
            return
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите режим «Цель»")
            return
        if not self.state.spline_polyline or not self.mapViewDrive or not self.mapViewDrive.scene():
            return

        sp = self.mapViewDrive.mapToScene(int(x_px), int(y_px))
        idx = nearest_index(self.state.spline_polyline, sp)
        self.state.goal_idx = idx

        # Перерисовать оверлеи на DRIVE, чтобы показать красный флаг и путь
        redraw_drive_overlays(self.state, self.mapViewDrive.scene())

    # -------------------- приём точек лидара и отрисовка --------------------
    def _on_points(self, pts):
        """pts — список (x,y) в метрах, в системе (x вправо, y вверх)."""
        if self.radarViewDrive and self.radarViewDrive.scene():
            # старый слой точек (оставляем)
            draw_radar_points(self.radarViewDrive.scene(), pts, meters_to_px=40.0)
            # новый heatmap-слой
            self._heatmap.update_from_xy(pts)

    # -------------------- shutdown (вызывай из Main.closeEvent) --------------------
    def shutdown(self):
        try:
            if getattr(self, "_lidar_rx", None) and self._lidar_rx.isRunning():
                self._lidar_rx.stop()
                self._lidar_rx.wait(500)
        except Exception:
            pass

