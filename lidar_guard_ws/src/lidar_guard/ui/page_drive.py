# -*- coding: utf-8 -*-
from typing import Tuple
from PyQt5 import QtWidgets, QtCore, QtGui
from lidar_udp_rx import LidarUdpReceiver  
from robot_cmd import update_drive_panel, set_speed

# ROUTING / GRAPHICS
from routing import (
    set_goal_px,
    set_control_item_px,
    set_robot_pose_px,        
    build_route_from_robot_to_goal,
    compute_controls_on_route,
)
from graphics import (
    redraw_route,
    redraw_markers,
    start_route_animation,
    stop_route_animation,
    reset_route_progress,
    check_flag_collision_and_update,
    redraw_radar_points,
    ensure_scene, 
    prepare_view
)

Point = Tuple[float, float]

PPM = 400.0  # пикселей в метре для радара

class DrivePage(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__(ui)          # ОБЯЗАТЕЛЬНО сначала
        self.ui = ui
        self.state = state

        # ---- Виджеты DRIVE ----
        self.pageDrive: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageDrive")
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")

        if self.radarViewDrive:
            ensure_scene(self.radarViewDrive)
            prepare_view(self.radarViewDrive)
            self.radarViewDrive.setRenderHints(
                QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
            )
            self.radarViewDrive.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.radarViewDrive.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.radarViewDrive.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
            self.radarViewDrive.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)

        # ---- Сцена радара ----
        self._radar_scene = self.radarViewDrive.scene() if self.radarViewDrive else None
        if self._radar_scene is None and self.radarViewDrive is not None:
            self._radar_scene = QtWidgets.QGraphicsScene(self.radarViewDrive)
            self.radarViewDrive.setScene(self._radar_scene)
        if self._radar_scene is not None:
            self._radar_scene.setSceneRect(-300, -300, 600, 600)
            self._draw_radar_grid()

        # контейнер для быстрого удаления старых точек
        self._radar_dots_group = None

        # ---- UDP-приёмник лидара ----
        self._radar_rx = None
        self._setup_radar()
    
        self.btnStartStop: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnSelectGoalDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalDrive")
        self.btnKR: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnKR")
        self.btnRobotDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnRobotDrive")  # ← НОВОЕ

        # --- Состояние ---
        self.state.is_running = bool(getattr(self.state, "is_running", False))
        self.state.select_goal_mode_drive = bool(getattr(self.state, "select_goal_mode_drive", False))
        self.state.kr_mode_drive = bool(getattr(self.state, "kr_mode_drive", False))
        self.state.manual_loc_drive = bool(getattr(self.state, "manual_loc_drive", False))  # ← НОВОЕ

        # --- Подключения ---
        if self.btnStartStop:
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)
            self._refresh_startstop_caption()

        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setCheckable(True)
            self.btnSelectGoalDrive.toggled.connect(self._on_select_goal_drive_toggled)
            self._refresh_select_goal_caption()

        if self.btnKR:
            self.btnKR.setCheckable(True)
            self.btnKR.toggled.connect(self._on_kr_toggled)
            self._refresh_kr_caption()

        if self.btnRobotDrive:
            self.btnRobotDrive.setCheckable(True)
            self.btnRobotDrive.toggled.connect(self._on_robot_mode_toggled)   # ← НОВОЕ
            self._refresh_robot_caption()

        # Клики по карте
        if self.mapViewDrive is not None:
            self.mapViewDrive.setMouseTracking(True)
            self.mapViewDrive.viewport().installEventFilter(self)

        # Таймер для проверки КР и обновления панели
        self._progress_timer = QtCore.QTimer(self)
        self._progress_timer.setInterval(200)  # мс
        self._progress_timer.timeout.connect(self._on_progress_tick)
        if self.state.is_running:
            self._progress_timer.start()

        self._update_controls()

    # ---------- РАДАР ----------
    def _setup_radar(self):
        """Старт UDP-приёмника и связываем сигнал с отрисовкой."""
        if self._radar_scene is None:
            return
        try:
            self._radar_rx = LidarUdpReceiver(host='127.0.0.1', port=10000, parent=self)
            self._radar_rx.pointsReady.connect(self._on_radar_points)
            self._radar_rx.start()
            print("[RADAR] listening on 127.0.0.1:10000", flush=True)
        except Exception as e:
            print("[RADAR] failed to start UDP receiver:", e, flush=True)

    def _on_radar_points(self, pts_xy_m):
        """Базовая отрисовка точек-лидара (x,y в метрах) в radarViewDrive."""
        sc = self._radar_scene
        if sc is None or not pts_xy_m:
            # можно ещё очистить, если хочется пустой кадр
            return

        # очистить прошлую группу точек (но не сетку)
        if self._radar_dots_group is not None:
            try:
                sc.removeItem(self._radar_dots_group)
            except Exception:
                pass
            self._radar_dots_group = None

        items = []
        pen = QtGui.QPen(QtCore.Qt.NoPen)
        brush = QtGui.QBrush(QtGui.QColor("#00bcd4"))
        d = 2
        # границы сцены чтобы не рисовать лишнего
        r = sc.sceneRect()
        for (x_m, y_m) in pts_xy_m:
            xp = x_m * PPM
            yp = -y_m * PPM  # экранная Y вниз
            if not r.contains(xp, yp):
                continue
            dot = sc.addEllipse(xp - d, yp - d, 2 * d, 2 * d, pen, brush)
            dot.setZValue(5)
            items.append(dot)

        if items:
            self._radar_dots_group = sc.createItemGroup(items)
            self._radar_dots_group.setZValue(5)

    def _draw_radar_grid(self):
        """Лёгкая сетка и оси (необязательно)."""
        sc = self._radar_scene
        if sc is None:
            return
        r = sc.sceneRect()
        cx, cy = 0.0, 0.0
        items = []

        # Оси
        pen_axis = QtGui.QPen(QtGui.QColor("#bbbbbb")); pen_axis.setWidth(1); pen_axis.setCosmetic(True)
        items.append(sc.addLine(r.left(), 0, r.right(), 0, pen_axis))
        items.append(sc.addLine(0, r.top(), 0, r.bottom(), pen_axis))

        # Круги по метрам
        pen_grid = QtGui.QPen(QtGui.QColor("#e0e0e0")); pen_grid.setWidth(1); pen_grid.setCosmetic(True)
        max_r_px = min(r.width(), r.height()) * 0.5
        step_px = PPM  # 1м
        rad = step_px
        while rad <= max_r_px + 1e-6:
            items.append(sc.addEllipse(cx - rad, cy - rad, 2 * rad, 2 * rad, pen_grid))
            rad += step_px

        grp = sc.createItemGroup(items)
        grp.setZValue(1)

    # ---------- Завершение ----------
    def shutdown(self):
        """Остановить UDP-приёмник аккуратно (звать при закрытии)."""
        try:
            if self._radar_rx and self._radar_rx.isRunning():
                self._radar_rx.stop()
                self._radar_rx.wait(500)
        except Exception:
            pass
 

    # ---------------------------------------------------------------------
    # Фильтр событий — клики по карте
    # ---------------------------------------------------------------------
    def eventFilter(self, obj, ev):
        if self.mapViewDrive and obj is self.mapViewDrive.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewDrive.mapToScene(ev.pos())
                self._on_drive_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

    # ---------------------------------------------------------------------
    # Верхний бар (ПУСК/СТОП)
    # ---------------------------------------------------------------------
    def _on_startstop_toggled(self, checked: bool):
        self.state.is_running = bool(checked)
        self._refresh_startstop_caption()
        self._update_controls()
        print(f"[DRIVE] StartStop -> {'START' if self.state.is_running else 'STOP'}", flush=True)

        if self.state.is_running:
            # НЕ сбрасываем прогресс: продолжаем с сохранённых route_done_m / route_progress_idx
            set_speed(self.state, 100.0)            # заглушка скорости
            start_route_animation(self.ui, self.state)  # таймер сам подхватит текущий прогресс
            if self._progress_timer and not self._progress_timer.isActive():
                self._progress_timer.start()
        else:
            # Стоп: останавливаем анимацию, но сохраняем прогресс (keep_progress=True)
            if self._progress_timer and self._progress_timer.isActive():
                self._progress_timer.stop()
            stop_route_animation(self.state, keep_progress=True)

        # Обновляем HUD — он отобразит сохранённые route_done_m и остальное
        try:
            compute_controls_on_route(self.state, eps_px=2.0)   # пересчитать КР по текущему маршруту
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _on_drive_pick_map(self):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала Стоп → затем карта.")
            return
        if hasattr(self.ui, "pick_map_and_load"):
            self.ui.pick_map_and_load()
        else:
            QtWidgets.QMessageBox.information(self.ui, "Выбор карты", "Функция pick_map_and_load не найдена в Main.")

    # ---------------------------------------------------------------------
    # Переключатели режимов (ЦЕЛЬ / КР / РОВЕР)
    # ---------------------------------------------------------------------
    def _on_select_goal_drive_toggled(self, enabled: bool):
        self.state.select_goal_mode_drive = bool(enabled)
        if enabled:
            # взаимоисключение
            if self.btnKR and self.btnKR.isChecked():
                self.btnKR.setChecked(False)
                self.state.kr_mode_drive = False
            if self.btnRobotDrive and self.btnRobotDrive.isChecked():
                self.btnRobotDrive.setChecked(False)
                self.state.manual_loc_drive = False
        self._refresh_select_goal_caption()

    def _on_kr_toggled(self, enabled: bool):
        self.state.kr_mode_drive = bool(enabled)
        if enabled:
            if self.btnSelectGoalDrive and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
            if self.btnRobotDrive and self.btnRobotDrive.isChecked():
                self.btnRobotDrive.setChecked(False)
                self.state.manual_loc_drive = False
        self._refresh_kr_caption()

    def _on_robot_mode_toggled(self, enabled: bool):
        """Режим РОВЕР: клик по карте = задать позу робота (ручная локализация)."""
        self.state.manual_loc_drive = bool(enabled)
        if enabled:
            # взаимоисключение
            if self.btnSelectGoalDrive and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
            if self.btnKR and self.btnKR.isChecked():
                self.btnKR.setChecked(False)
                self.state.kr_mode_drive = False
        self._refresh_robot_caption()

    def _refresh_startstop_caption(self):
        if self.btnStartStop:
            self.btnStartStop.setText("Стоп" if self.state.is_running else "Пуск")

    def _refresh_select_goal_caption(self):
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setText("Цель" + (" (вкл.)" if self.state.select_goal_mode_drive else ""))

    def _refresh_kr_caption(self):
        if self.btnKR:
            self.btnKR.setText("КР" + (" (вкл.)" if self.state.kr_mode_drive else ""))

    def _refresh_robot_caption(self):
        if self.btnRobotDrive:
            self.btnRobotDrive.setText("Ровер" + (" (вкл.)" if self.state.manual_loc_drive else ""))

    def _update_controls(self):
        running = bool(self.state.is_running)
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not running)

        # ЦЕЛЬ
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setEnabled(not running)
            if running and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
                self._refresh_select_goal_caption()

        # КР
        if self.btnKR:
            self.btnKR.setEnabled(not running)
            if running and self.btnKR.isChecked():
                self.btnKR.setChecked(False)
                self.state.kr_mode_drive = False
                self._refresh_kr_caption()

        # РОВЕР (ручная локализация)
        if self.btnRobotDrive:
            self.btnRobotDrive.setEnabled(not running)
            if running and self.btnRobotDrive.isChecked():
                self.btnRobotDrive.setChecked(False)
                self.state.manual_loc_drive = False
                self._refresh_robot_caption()

    # ---------------------------------------------------------------------
    # Клик по карте
    # ---------------------------------------------------------------------
    def _on_drive_map_click(self, x_px: float, y_px: float):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Остановите движение")
            return
        if not getattr(self.state, "graph", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф карты не загружен")
            return

        # 1) Режим РОВЕР: задать позу робота (ручная локализация)
        if getattr(self.state, "manual_loc_drive", False):
            set_robot_pose_px((x_px, y_px), self.state)  # снап по points_px
            # перерисовать обе сцены
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(self.state, sc)
                    except Exception:
                        pass
            # если есть цель — можно сразу перестроить маршрут
            if getattr(self.state, "goal_px", None):
                if build_route_from_robot_to_goal(self.state):
                    reset_route_progress(self.state)
                    try:
                        compute_controls_on_route(self.state, eps_px=2.0)
                    except Exception:
                        pass
                    redraw_route(self.state, self.ui)
            try:
                update_drive_panel(self.ui, self.state)
            except Exception:
                pass
            return

        # 2) Режим КР: ставим синий флаг
        if getattr(self.state, "kr_mode_drive", False):
            set_control_item_px((x_px, y_px), self.state)
            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(self.state, sc)
                    except Exception:
                        pass
            if self.ui.statusBar():
                n = len(getattr(self.state, "control_pts_px", []))
                self.ui.statusBar().showMessage(f"Добавлена контрольная точка #{n}", 1200)
            return

        # 3) Режим Цель: выбираем цель и строим маршрут
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите режим «Ровер», «КР» или «Цель».")
            return

        set_goal_px((x_px, y_px), self.state)

        # Обновить маркеры
        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                redraw_markers(self.state, sc)

        # Построить маршрут (если есть позиция робота)
        if getattr(self.state, "robot_px", None):
            ok = build_route_from_robot_to_goal(self.state)
            if ok:
                reset_route_progress(self.state)
                try:
                    compute_controls_on_route(self.state, eps_px=2.0)
                except Exception:
                    pass
                redraw_route(self.state, self.ui)
                update_drive_panel(self.ui, self.state)

    def _on_progress_tick(self):
        """Периодический тик: обновить панель и проверить достижение КР."""
        try:
            check_flag_collision_and_update(self.state)
        except Exception:
            pass
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass