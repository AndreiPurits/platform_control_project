# page_idle.py
# -*- coding: utf-8 -*-
"""
== API INDEX ================================================================
CLASS: IdlePage(ui, state)
- _go_maps() -> None
    Переключить в MAPS.
- _go_drive() -> None
    Переключить в DRIVE.
- _on_localization_clicked() -> None
    Заглушка под автолокализацию.
- _on_manual_loc_toggled(enabled: bool) -> None
    Режим «Локализация вручную» (ставим robot_px по клику).
- _on_select_goal_idle_toggled(enabled: bool) -> None
    Режим «Выбрать цель» (ставим goal_px по клику).
- _on_map_click(x_px: float, y_px: float) -> None
    Обработка клика по карте: граф (новый режим) или сплайн (fallback).
- _build_and_show_route(start_px: Point, goal_px: Point) -> None
    Построить маршрут через routing + перерисовать + HUD/статус.

ПОВЕДЕНИЕ:
- Если загружен граф (state.graph), клики обрабатываются через граф:
    * manual_loc_mode: клик задаёт state.robot_px; если уже есть goal_px — строим маршрут.
    * select_goal_mode_idle: клик задаёт state.goal_px; если есть robot_px — строим маршрут.
- Если графа нет, работаем по старому сплайну (nearest_index + redraw_idle_overlays).
============================================================================
"""

from typing import List, Tuple, Optional
from PyQt5 import QtWidgets, QtCore, QtGui
from state import AppState

# Графика
from graphics import ensure_scene, prepare_view, redraw_idle_overlays, update_hud_text, redraw_route
# Старый режим (сплайн)
from routing import nearest_index
# Новый режим (граф): построение маршрута и текст прогресса
from routing import build_route_snap_pixels, update_progress_text_for_robot

Point = Tuple[float, float]


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
        self.btnSelectGoalIdle: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalIdle")

        if self.mapViewIdle is None:
            raise RuntimeError("QGraphicsView 'mapViewIdle' не найден в .ui")

        # Навигация
        if self.btnChooseMap:
            self.btnChooseMap.clicked.connect(self._go_maps)
        if self.btnStart:
            self.btnStart.clicked.connect(self._go_drive)

        # Подготовка карты
        ensure_scene(self.mapViewIdle)
        try:
            prepare_view(self.mapViewIdle)  # твой зум/скролл и т.п.
        except Exception:
            pass
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

        # служебное (для fallback-режима со сплайном)
        self._pending_goal_idx: Optional[int] = None

    # ---------- фильтр кликов ----------
    class _ClickFilter(QtCore.QObject):
        def __init__(self, on_click):
            super().__init__()
            self.on_click = on_click

        def eventFilter(self, obj, ev):
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                # передаём координаты виджета; внутри переведём в scene
                self.on_click(float(ev.pos().x()), float(ev.pos().y()))
                return True
            return False

    # ---------------- Навигация между страницами ----------------
    def _go_maps(self):
        if hasattr(self.ui, "to_maps"):
            self.ui.to_maps()

    def _go_drive(self):
        if hasattr(self.ui, "to_drive"):
            self.ui.to_drive()

    # ---------------- Локализация ----------------
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

    # ---------------- Клик по карте (IDLE) ----------------
    def _on_map_click(self, x_px: float, y_px: float):
        view = self.mapViewIdle
        if not view or not view.scene():
            return

        # координаты сцены
        sp = view.mapToScene(int(x_px), int(y_px))

        # === Новый режим: если есть graph — строим всё по графу ===
        if getattr(self.state, "graph", None) is not None:
            click_pt: Point = (float(sp.x()), float(sp.y()))

            if getattr(self.state, "manual_loc_mode", False):
                # ручная локализация → обновляем позицию робота (в пикселях)
                self.state.robot_px = click_pt

                # если уже есть цель — перестроим маршрут
                if getattr(self.state, "goal_px", None) is not None:
                    self._build_and_show_route(self.state.robot_px, self.state.goal_px)
                else:
                    # просто обновим HUD: маршрут ещё не задан
                    text = update_progress_text_for_robot(click_pt, self.state)
                    for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                        if sc is not None:
                            update_hud_text(sc, text)
                    if hasattr(self.ui, "statusBar"):
                        self.ui.statusBar().showMessage(text, 1500)
                return

            if getattr(self.state, "select_goal_mode_idle", False):
                # выбор цели
                self.state.goal_px = click_pt
                # если есть позиция робота — строим маршрут
                if getattr(self.state, "robot_px", None) is not None:
                    self._build_and_show_route(self.state.robot_px, self.state.goal_px)
                else:
                    QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала укажите позицию робота (Локализация вручную)")
                return

            # если ни один режим не включён
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите «Локализация вручную» или «Выбрать цель»")
            return

        # === Fallback: старый сплайн ===
        if not getattr(self.state, "spline_polyline", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Нет сплайна (.json рядом с картой)")
            return

        idx = nearest_index(self.state.spline_polyline, sp)

        if getattr(self.state, "manual_loc_mode", False):
            # белый флаг = поза робота
            self.state.robot_idx = idx
        elif getattr(self.state, "select_goal_mode_idle", False):
            # красная цель
            self.state.goal_idx = idx
        else:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите «Локализация вручную» или «Выбрать цель»")
            return

        # Перерисовать оверлеи на IDLE (старый режим)
        redraw_idle_overlays(self.state, view.scene())

    # ---------------- Вспомогательное: построить маршрут, показать HUD ----------------
    def _build_and_show_route(self, start_px: Point, goal_px: Point):
        ok = False
        try:
            ok = build_route_snap_pixels(start_px, goal_px, self.state)
        except Exception as e:
            print("[IDLE] route build error:", e)
            ok = False

        if not ok:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Не удалось построить маршрут")
            return

        # перерисовать маршрут (в обеих сценах, если DRIVE открыт)
        try:
            redraw_route(self.state, self.ui)
        except Exception as e:
            print("[IDLE] redraw_route error:", e)

        # HUD и статус-бар: «Пройдено / Осталось» от старта
        try:
            text = update_progress_text_for_robot(start_px, self.state)
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    update_hud_text(sc, text)
            if hasattr(self.ui, "statusBar"):
                self.ui.statusBar().showMessage(text, 2500)
        except Exception as e:
            print("[IDLE] HUD update error:", e)