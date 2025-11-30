# page_idle.py
# -*- coding: utf-8 -*-
from typing import Tuple, Optional
from PyQt5 import QtWidgets, QtCore, QtGui
from state import AppState
from graphics import ensure_scene, prepare_view, redraw_markers, redraw_route, reset_route_progress
from routing import set_robot_pose_px, set_goal_px, build_route_from_robot_to_goal, set_control_item_px
from robot_cmd import update_drive_panel   # для lblMinDist

Point = Tuple[float, float]

class IdlePage(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState):
        super().__init__()
        self.ui = ui
        self.state = state

        self.pageIdle: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageIdle")
        self.mapViewIdle: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        self.btnChooseMap: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnChooseMap")
        self.btnLocalization: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnLocalization")
        self.btnManualLocalization: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnManualLocalization")
        self.btnStart: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStart")
        self.btnSelectGoalIdle: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalIdle")
        self.btnDataset: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnDataset")
        if self.btnChooseMap:
            self.btnChooseMap.clicked.connect(self._on_choose_map_clicked)
        if self.btnLocalization:
            self.btnLocalization.setCheckable(True)
            self.btnLocalization.toggled.connect(self._on_loc_toggled)
        if self.btnManualLocalization:
            self.btnManualLocalization.setCheckable(True)
            self.btnManualLocalization.toggled.connect(self._on_manual_loc_toggled)
        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setCheckable(True)
            self.btnSelectGoalIdle.toggled.connect(self._on_select_goal_idle_toggled)
        if self.btnDataset:
                self.btnDataset.clicked.connect(self._on_dataset_clicked)
        if self.btnStart:      
            self.btnStart.clicked.connect(self._go_drive)

        ensure_scene(self.mapViewIdle); 
        try: prepare_view(self.mapViewIdle)
        except: pass
        self.mapViewIdle.setMouseTracking(True)
        self.mapViewIdle.viewport().installEventFilter(self)

    def _on_choose_map_clicked(self):
        """Открыть диалог выбора карты и показать её в IDLE (без перехода на DRIVE)."""
        if hasattr(self.ui, "pick_map_and_load"):
            try:
                self.ui.pick_map_and_load()
            except Exception as e:
                print("[IDLE] pick_map_and_load error:", e, flush=True)
        else:
            QtWidgets.QMessageBox.warning(self.ui, "Ошибка", "В Main нет метода pick_map_and_load().")
        # ---- IDLE: Датасет -> выбрать карту -> перейти на DRIVE

    def _on_dataset_clicked(self):
        # 1) выбрать карту
        if hasattr(self.ui, "pick_map_and_load"):
            try:
                self.ui.pick_map_and_load()
            except Exception as e:
                print("[IDLE] pick_map_and_load error:", e, flush=True)
                return
        else:
            QtWidgets.QMessageBox.warning(self.ui, "Ошибка", "В Main нет метода pick_map_and_load().")
            return

        # если карту не выбрали (диалог закрыли), выходим
        if not getattr(self.state, "active_map_path", None):
            return

        # 2) помечаем режим датасета
        self.state.dataset_mode = True

        # 3) переходим на DRIVE
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        pageDrive = self.ui.findChild(QtWidgets.QWidget, "pageDrive")
        if stack and pageDrive:
            stack.setCurrentWidget(pageDrive)

        # 4) индикаторы по умолчанию
        try:
            from status import set_indicator
            set_indicator(self.ui.findChild(QtWidgets.QLabel, "indBattery"), "ok")
            set_indicator(self.ui.findChild(QtWidgets.QLabel, "indLidar"),   "ok")
        except Exception:
            pass
            
    # события мыши
    def eventFilter(self, obj, ev):
        if obj is self.mapViewIdle.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewIdle.mapToScene(ev.pos())
                self._on_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

    def _go_drive(self):
        if hasattr(self.ui, "to_drive"): self.ui.to_drive()

    def _on_loc_toggled(self, enabled: bool):
        # Режим установки СИНИХ контрольных точек
        self.state.loc_mode_idle = bool(enabled)
        if self.btnLocalization:
            self.btnLocalization.setText("Выбрать контрольную точку" + (" (вкл.)" if enabled else ""))
    
        if enabled:
            # выключаем остальные режимы
            if self.btnManualLocalization and self.btnManualLocalization.isChecked():
                self.btnManualLocalization.setChecked(False)
            if self.btnSelectGoalIdle and self.btnSelectGoalIdle.isChecked():
                self.btnSelectGoalIdle.setChecked(False)


    def _on_manual_loc_toggled(self, enabled: bool):
        # Режим ручной локализации (позиция робота)
        self.state.manual_loc_mode = bool(enabled)
        if self.btnManualLocalization:
            self.btnManualLocalization.setText("Локализация вручную" + (" (вкл.)" if enabled else ""))

        if enabled:
            # выключаем остальные режимы
            if self.btnLocalization and self.btnLocalization.isChecked():
                self.btnLocalization.setChecked(False)
            if self.btnSelectGoalIdle and self.btnSelectGoalIdle.isChecked():
                self.btnSelectGoalIdle.setChecked(False)


    def _on_select_goal_idle_toggled(self, enabled: bool):
        # Режим выбора цели (красный флаг / цель маршрута)
        self.state.select_goal_mode_idle = bool(enabled)
        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setText("Выбрать цель" + (" (вкл.)" if enabled else ""))

        if enabled:
            # выключаем остальные режимы
            if self.btnManualLocalization and self.btnManualLocalization.isChecked():
                self.btnManualLocalization.setChecked(False)
            if self.btnLocalization and self.btnLocalization.isChecked():
                self.btnLocalization.setChecked(False)

    def _on_map_click(self, x_px: float, y_px: float):
        if not getattr(self.state, "graph", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф не загружен")
            return

        clicked: Point = (x_px, y_px)
        changed = False

        if getattr(self.state, "manual_loc_mode", False):
            set_robot_pose_px(clicked, self.state)
            changed = True
        elif getattr(self.state, "select_goal_mode_idle", False):
            set_goal_px(clicked, self.state)
            changed = True
        elif getattr(self.state, "loc_mode_idle", False):
            set_control_item_px(clicked, self.state)
            changed = True
        else:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите «Локализация вручную» или «Выбрать цель».")
            return

        # перерисовать маркеры в обоих видах
        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                try: redraw_markers(self.state, sc)
                except Exception as e: print("[IDLE] redraw_markers error:", e)

        # если есть и робот, и цель — строим маршрут, рисуем, обновляем панель
        if getattr(self.state, "robot_px", None) and getattr(self.state, "goal_px", None):
            ok = False
            try:
                ok = build_route_from_robot_to_goal(self.state)
            except Exception as e:
                print("[IDLE] route build error:", e)
            if ok:
                try:
                    redraw_route(self.state, self.ui)
                except Exception as e:
                    print("[IDLE] redraw_route error:", e)
                reset_route_progress(self.state)
                # обновить верхнюю панель Drive (lblMinDist и т.д.)
                try:
                    from robot_cmd import update_drive_panel
                    update_drive_panel(self.ui, self.state)
                except Exception as e:
                    print("[IDLE] panel update error:", e)