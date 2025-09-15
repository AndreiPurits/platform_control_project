# page_idle.py
# -*- coding: utf-8 -*-
from typing import List, Tuple
from PyQt5 import QtWidgets, QtCore, QtGui
from state import AppState
from graphics import ensure_scene, prepare_view, redraw_idle_overlays
from routing import nearest_index

class IdlePage:
    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState):
        self.ui = ui
        self.state = state

        # Виджеты
        self.pageIdle: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageIdle")
        self.mapViewIdle: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        self.btnChooseMap: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnChooseMap")
        self.btnLocalization: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnLocalization")
        self.btnManualLocalization: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnManualLocalization")
        self.btnStart: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStart")
        self.btnSelectGoalIdle: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalIdle")  # НОВОЕ

        if self.mapViewIdle is None:
            raise RuntimeError("QGraphicsView 'mapViewIdle' не найден в .ui")

        # Навигация
        if self.btnChooseMap:
            self.btnChooseMap.clicked.connect(self._go_maps)
        if self.btnStart:
            self.btnStart.clicked.connect(self._go_drive)

        # Подготовка карты
        ensure_scene(self.mapViewIdle)
        prepare_view(self.mapViewIdle)
        self.mapViewIdle.setMouseTracking(True)
        self._idle_click_filter = self._ClickFilter(self._on_map_click)
        self.mapViewIdle.viewport().installEventFilter(self._idle_click_filter)

        # Кнопки локализации/цели
        if self.btnLocalization:
            self.btnLocalization.clicked.connect(self._on_localization_clicked)  # заглушка
        if self.btnManualLocalization:
            self.btnManualLocalization.setCheckable(True)
            self.btnManualLocalization.toggled.connect(self._on_manual_loc_toggled)

        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setCheckable(True)
            self.btnSelectGoalIdle.toggled.connect(self._on_select_goal_idle_toggled)

    class _ClickFilter(QtCore.QObject):
        def __init__(self, on_click): super().__init__(); self.on_click = on_click
        def eventFilter(self, obj, ev):
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                self.on_click(float(ev.pos().x()), float(ev.pos().y()))
                return True
            return False

    # Навигация
    def _go_maps(self):
        if hasattr(self.ui, "to_maps"):
            self.ui.to_maps()

    def _go_drive(self):
        if hasattr(self.ui, "to_drive"):
            self.ui.to_drive()

    # Локализация
    def _on_localization_clicked(self):
        QtWidgets.QMessageBox.information(self.ui, "Локализация", "Пока заглушка.")

    def _on_manual_loc_toggled(self, enabled: bool):
        self.state.manual_loc_mode = bool(enabled)
        if self.btnManualLocalization:
            self.btnManualLocalization.setText("Локализация вручную" + (" (вкл.)" if enabled else ""))
        # выключим режим выбора цели, чтобы не конфликтовали
        if enabled and self.btnSelectGoalIdle and self.btnSelectGoalIdle.isChecked():
            self.btnSelectGoalIdle.setChecked(False)

    def _on_select_goal_idle_toggled(self, enabled: bool):
        self.state.select_goal_mode_idle = bool(enabled)
        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setText("Выбрать цель" + (" (вкл.)" if enabled else ""))
        # выключим ручную локализацию, чтобы не конфликтовали
        if enabled and self.btnManualLocalization and self.btnManualLocalization.isChecked():
            self.btnManualLocalization.setChecked(False)

    # Клик по карте
    def _on_map_click(self, x_px: float, y_px: float):
        view = self.mapViewIdle
        if not view or not view.scene(): return
        sp = view.mapToScene(int(x_px), int(y_px))

        if not self.state.spline_polyline:
            # Без сплайна ставить нечего
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Нет сплайна (.json рядом с картой)")
            return

        # Находим ближайший индекс на сплайне (привязка к дороге)
        idx = nearest_index(self.state.spline_polyline, sp)

        if self.state.manual_loc_mode:
            # белый флаг = поза робота
            self.state.robot_idx = idx
        elif self.state.select_goal_mode_idle:
            # красная цель
            self.state.goal_idx = idx
        else:
            # если режим не выбран — подскажем
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите «Локализация вручную» или «Выбрать цель»")
            return

        # Перерисовать оверлеи на IDLE
        redraw_idle_overlays(self.state, view.scene())