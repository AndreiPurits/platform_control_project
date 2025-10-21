# page_idle.py
# -*- coding: utf-8 -*-
from typing import Tuple, Optional
from PyQt5 import QtWidgets, QtCore, QtGui
from state import AppState
from graphics import ensure_scene, prepare_view, redraw_markers, redraw_route
from routing import set_robot_pose_px, set_goal_px, build_route_from_robot_to_goal
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

        if self.btnChooseMap: self.btnChooseMap.clicked.connect(self._go_maps)
        if self.btnStart:      self.btnStart.clicked.connect(self._go_drive)

        ensure_scene(self.mapViewIdle); 
        try: prepare_view(self.mapViewIdle)
        except: pass
        self.mapViewIdle.setMouseTracking(True)
        self.mapViewIdle.viewport().installEventFilter(self)

        if self.btnLocalization:
            self.btnLocalization.setCheckable(True)
            self.btnLocalization.clicked.connect(self._on_control_point_clicked)
        if self.btnManualLocalization:
            self.btnManualLocalization.setCheckable(True)
            self.btnManualLocalization.toggled.connect(self._on_manual_loc_toggled)
        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setCheckable(True)
            self.btnSelectGoalIdle.toggled.connect(self._on_select_goal_idle_toggled)

        # события мыши
    def _on_control_point_clicked(self):
        """Добавляет синюю контрольную точку на карту."""
        view = self.mapViewIdle
        if not view or not view.scene():
            return
        QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Кликните на карте для установки точки.")
        # активируем «режим постановки»
        self.state.manual_loc_mode = True
        # когда пользователь кликнет — обработается в _on_map_click()

    def eventFilter(self, obj, ev):
        if obj is self.mapViewIdle.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewIdle.mapToScene(ev.pos())
                self._on_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

    def _go_maps(self):
        if hasattr(self.ui, "to_maps"): self.ui.to_maps()

    def _go_drive(self):
        if hasattr(self.ui, "to_drive"): self.ui.to_drive()

    def _on_manual_loc_toggled(self, enabled: bool):
        self.state.manual_loc_mode = bool(enabled)
        if self.btnManualLocalization:
            self.btnManualLocalization.setText("Локализация вручную" + (" (вкл.)" if enabled else ""))
        if enabled and self.btnSelectGoalIdle and self.btnSelectGoalIdle.isChecked():
            self.btnSelectGoalIdle.setChecked(False)

    def _on_select_goal_idle_toggled(self, enabled: bool):
        self.state.select_goal_mode_idle = bool(enabled)
        if self.btnSelectGoalIdle:
            self.btnSelectGoalIdle.setText("Выбрать цель" + (" (вкл.)" if enabled else ""))
        if enabled and self.btnManualLocalization and self.btnManualLocalization.isChecked():
            self.btnManualLocalization.setChecked(False)

    def _on_map_click(self, x_px: float, y_px: float):
        # 1) Проверки
        if not getattr(self.state, "graph", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф не загружен")
            return
        view = self.mapViewIdle
        if not view or not view.scene():
            return

        clicked: Point = (float(x_px), float(y_px))

        # 2) Режим «Контрольная точка» — ставим синий флажок (со снэпом к дороге)
        if getattr(self.state, "manual_loc_mode", False):
            try:
                # снэп к ближайшей точке из *_points_pixels.json — флажок будет на дороге
                from routing import _nearest_point_from_list  # уже есть в routing
                pts = getattr(self.state, "points_px", []) or []
                pt_snapped = _nearest_point_from_list(clicked, pts) if pts else clicked
            except Exception:
                pt_snapped = clicked

            # подготовим контейнер под контрольные точки
            if not hasattr(self.state, "control_pts_px") or self.state.control_pts_px is None:
                self.state.control_pts_px = []

            self.state.control_pts_px.append((float(pt_snapped[0]), float(pt_snapped[1])))

            # перерисовать маркеры на обеих сценах
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        from graphics import redraw_markers
                        redraw_markers(self.state, sc)
                    except Exception as e:
                        print("[IDLE] redraw_markers error:", e)

            # по ТЗ — после клика в этом режиме выходим из режима
            self.state.manual_loc_mode = False
            if self.btnManualLocalization and self.btnManualLocalization.isChecked():
                self.btnManualLocalization.setChecked(False)

            return

        # 3) Режим «Выбрать цель» — ставим goal (со снэпом к дороге)
        if getattr(self.state, "select_goal_mode_idle", False):
            try:
                from routing import set_goal_px
                set_goal_px(clicked, self.state)  # внутри сделает снэп
            except Exception:
                # запасной путь — без снэпа
                self.state.goal_px = (clicked[0], clicked[1])

            # перерисовать маркеры в обеих сценах
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        from graphics import redraw_markers
                        redraw_markers(self.state, sc)
                    except Exception as e:
                        print("[IDLE] redraw_markers error:", e)

            # если есть и robot_px, и goal_px — строим маршрут
            if getattr(self.state, "robot_px", None) and getattr(self.state, "goal_px", None):
                ok = False
                try:
                    from routing import build_route_from_robot_to_goal
                    ok = build_route_from_robot_to_goal(self.state)
                except Exception as e:
                    print("[IDLE] route build error:", e)

                if ok:
                    # отрисовать путь + обновить подписи/панель DRIVE
                    try:
                        from graphics import redraw_route
                        redraw_route(self.state, self.ui)
                    except Exception as e:
                        print("[IDLE] redraw_route error:", e)

                    try:
                        from robot_cmd import update_drive_panel
                        update_drive_panel(self.ui, self.state)
                    except Exception as e:
                        print("[IDLE] panel update error:", e)
            return

        # 4) Ни один режим не включён
        QtWidgets.QToolTip.showText(QtGui.QCursor.pos(),
                                    "Включите «Контрольная точка» или «Выбрать цель».")