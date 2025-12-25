# -*- coding: utf-8 -*-
from typing import Tuple
from PyQt5 import QtWidgets, QtCore, QtGui
from lidar_udp_rx import LidarUdpReceiver
import numpy as np
import cv2
import os
import sys
import collections
from lidar_matcher import snap_robot_by_lidar, LidarMatcher
import time, math
# ---- Доступ к roadseg_work/roadseg.py ----
HERE = os.path.dirname(__file__)
ROADSEG_DIR = os.path.join(HERE, "roadseg_work")
if ROADSEG_DIR not in sys.path:
    sys.path.append(ROADSEG_DIR)

try:
    from roadseg import RoadSeg
    print("[SEG] roadseg module available", flush=True)
except Exception as e:
    RoadSeg = None
    print("[SEG] roadseg module not available:", e, flush=True)

# ROUTING / GRAPHICS
from routing import (
    set_goal_px,
    set_control_item_px,
    set_robot_pose_px,
    build_route_from_robot_to_goal,
    compute_controls_on_route,
    update_junction_turn_hint,
    maybe_snap_robot_to_junction_by_camera
)
from graphics import (
    redraw_route,
    redraw_markers,
    start_route_animation,
    stop_route_animation,
    reset_route_progress,
    check_flag_collision_and_update,
    ensure_scene,
    prepare_view,
    _lidar,
    recompute_route_metrics,
)
from robot_cmd import update_drive_panel, route_caption_text, apply_start_defaults, motors_set, update_autoturn_fsm, PWM_NEUTRAL


Point = Tuple[float, float]

PPM = 80.0  # пикселей в метре для радара


class DrivePage(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__(ui)
        self.ui = ui
        self.state = state

        # --- VIDEO / SEG ---
        self._video_mode = "radar"   # "radar" | "camera"
        self._cam_timer = None
        self._cam = None
        self._cam_pixmap_item = None
        self._mask_ema = None

        self._seg = None                  # RoadSeg-инстанс или None
        self._seg_enabled = False
        self.state.road_guard_enabled = False
        self.state.road_state = "no_road"
        self._last_road_state = None
        self._cam_tick_counter = 0

        self.state.road_on = False
        self.state.road_frac = 0.0
        self.state.stripe_frac = 0.0
        self._road_log = collections.deque(maxlen=500)
        self.state.road_frac_ema = None
        self.state.stripe_frac_ema = None
        self._radar_rx_front = None
        self._radar_rx_rear  = None
        # --- Инициализация сегментации ---

        self._seg_onnx_path = os.path.join(
            HERE,
            "roadseg_work",
            "roadseg_asphalt.onnx",
        )
        if RoadSeg is not None:
            try:
                self._seg = RoadSeg(
                    onnx_path=self._seg_onnx_path,
                    input_size=(512, 512),  # сейчас модель 512x512
                )
                if getattr(self._seg, "ok", False):
                    print(f"[SEG] RoadSeg ONNX ready: {self._seg_onnx_path}", flush=True)
                else:
                    print(f"[SEG] RoadSeg fallback (stub), no ONNX: {self._seg_onnx_path}", flush=True)
            except Exception as e:
                print("[SEG] init error, using stub:", e, flush=True)
                try:
                    self._seg = RoadSeg(onnx_path=None)
                except Exception:
                    self._seg = None
        else:
            print("[SEG] RoadSeg class not available, segmentation disabled", flush=True)

        # --- LIDAR MATCHER 


        self._lid_matcher = LidarMatcher()
        self._lid_matcher.build_from_dir(
        os.path.expanduser("~/roadseg_work/datasets/lidar/Poly_asf")
       )


        # ---- Виджеты DRIVE ----
        self.pageDrive: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageDrive")
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")

        # Зум карты
        self._map_zoom = 1.0
        self.btnZoomIn = ui.findChild(QtWidgets.QToolButton, "btnZoomIn")
        self.btnZoomOut = ui.findChild(QtWidgets.QToolButton, "btnZoomOut")

        # Верхний бар + лейблы
        self.btnStartStop: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnSelectGoalDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnSelectGoalDrive")
        self.btnKR: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnKR")
        self.btnRobotDrive: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnRobotDrive")
        self.btnVideoMode: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnVideoMode")
        self.btnEStop: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnEStop")

        self.lblSpeed: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblSpeed")
        self.lblMinDist: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblMinDist")

        self.indLidar: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "indLidar")
        self.indBattery: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "indBattery")
        self._update_road_indicator()
#
        # Целевая скорость (Drive)
        self.btnSpeedMinusDrive: QtWidgets.QToolButton = ui.findChild(QtWidgets.QToolButton, "btnSpeedMinusDrive")
        self.btnSpeedPlusDrive: QtWidgets.QToolButton = ui.findChild(QtWidgets.QToolButton, "btnSpeedPlusDrive")
        # --- Отдельное окно мануального управления ---
        self.btnManualDialog: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnManualDialog")
        self._manual_window = None

        # ---- Настройка mapView ----
        if self.mapViewDrive:
            self.mapViewDrive.setRenderHints(
                QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
            )
            self.mapViewDrive.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.mapViewDrive.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        # Подготовка радара
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

        self._radar_scene = self.radarViewDrive.scene() if self.radarViewDrive else None
        if self._radar_scene is None and self.radarViewDrive is not None:
            self._radar_scene = QtWidgets.QGraphicsScene(self.radarViewDrive)
            self.radarViewDrive.setScene(self._radar_scene)
        if self._radar_scene is not None:
            self._radar_scene.setSceneRect(-300, -300, 600, 600)
            self._draw_radar_grid()

        self._radar_dots_group = None
        self._radar_rx = None

        # --- Состояние ---
        self.state.is_running = bool(getattr(self.state, "is_running", False))
        self.state.select_goal_mode_drive = bool(getattr(self.state, "select_goal_mode_drive", False))
        self.state.kr_mode_drive = bool(getattr(self.state, "kr_mode_drive", False))
        self.state.manual_loc_drive = bool(getattr(self.state, "manual_loc_drive", False))
        self.state.is_running = bool(getattr(self.state, "is_running", False))
        self.state.select_goal_mode_drive = bool(getattr(self.state, "select_goal_mode_drive", False))
        self.state.kr_mode_drive = bool(getattr(self.state, "kr_mode_drive", False))
        self.state.manual_loc_drive = bool(getattr(self.state, "manual_loc_drive", False))
        self.state.trajectory_mode = bool(getattr(self.state, "trajectory_mode", False))
        # --- Таймер для режима траектории (отдельный контроллер движения) ---
        self._traj_timer = QtCore.QTimer(self)
        self._traj_timer.setInterval(100)  # 10 Гц
        self._traj_timer.timeout.connect(self._trajectory_tick)
        # Ручное управление
        self.state.manual_drive = bool(getattr(self.state, "manual_drive", False))
        self.state.manual_pwm_base = int(getattr(self.state, "manual_pwm_base", 1800) or 1800)
        self.state.manual_l_pwm = int(getattr(self.state, "manual_l_pwm", 1500))
        self.state.manual_r_pwm = int(getattr(self.state, "manual_r_pwm", 1500))

    
        # Кусочный маршрут через несколько целей
        if not hasattr(self.state, "piece_goals_px") or self.state.piece_goals_px is None:
            self.state.piece_goals_px = []

        # --- Подключения ---
        if self.btnStartStop:
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)
            self._refresh_startstop_caption()

        # --- Кнопка режима "Траектория" ---
        self.btnTrajectory: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnTrajectory")
        if self.btnTrajectory:
            self.btnTrajectory.setCheckable(True)
            self.btnTrajectory.toggled.connect(self._on_trajectory_toggled)
            # синхронизация с состоянием
            self.btnTrajectory.setChecked(bool(getattr(self.state, "trajectory_mode", False)))

        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_back_clicked)

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
            self.btnRobotDrive.toggled.connect(self._on_robot_mode_toggled)
            self._refresh_robot_caption()

        if self.btnVideoMode:
            self.btnVideoMode.setChecked(False)
            self.btnVideoMode.toggled.connect(self._on_video_toggled)

        # Кнопка «Мануальное управление» (открывает диалог)
        if self.btnManualDialog:
            self.btnManualDialog.clicked.connect(self._on_manual_window_clicked)

        # Кнопки зума карты
        if self.btnZoomIn:
            self.btnZoomIn.clicked.connect(self._on_zoom_in)
        if self.btnZoomOut:
            self.btnZoomOut.clicked.connect(self._on_zoom_out)

        # Управление целевой скоростью
        if self.btnSpeedMinusDrive:
            self.btnSpeedMinusDrive.clicked.connect(lambda: self._change_speed(-50))
            try:
                update_drive_panel(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] panel pre-update error:", e, flush=True)
        if self.btnSpeedPlusDrive:
            self.btnSpeedPlusDrive.clicked.connect(lambda: self._change_speed(+50))
            try:
                update_drive_panel(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] panel pre-update error:", e, flush=True)
        # Клики по карте
        if self.mapViewDrive is not None:
            self.mapViewDrive.setMouseTracking(True)
            self.mapViewDrive.viewport().installEventFilter(self)

        # Таймер прогресса (маршрут)
        self._progress_timer = QtCore.QTimer(self)
        self._progress_timer.setInterval(200)
        self._progress_timer.timeout.connect(self._on_progress_tick)
        if self.state.is_running:
            self._progress_timer.start()

        # Переключение страниц (IDLE/DRIVE)
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        if stack:
            stack.currentChanged.connect(self._on_stack_changed)

        self._update_controls()

        # Камера — универсальный подход
        self._ensure_camera_open()
        self._ensure_camera_timer()
        
    def _wrap_angle(a: float) -> float:
        """Нормализуем угол в радианах в диапазон [-pi, +pi]."""
        import math
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
    # ---------------------------------------------------------------------
    # Режим движения по траектории (btnTrajectory)
    # ---------------------------------------------------------------------
    
    def _on_trajectory_toggled(self, checked: bool):
        """
        Тоггл режима движения по траектории (кнопка btnTrajectory).

        Логика:
          - нельзя включать, если включён ручной режим;
          - нельзя включать, если нет маршрута (route_pts_px/route_pts_m);
          - при включении:
              * включаем trajectory_mode и is_running
              * выставляем базовую скорость (pwm_base_us)
              * запускаем apply_start_defaults / compute_controls_on_route
              * запускаем анимацию маршрута и оба таймера:
                    - _progress_timer (как в обычном Пуск)
                    - __traj_timer (углы, дельты, motors_set)
              * обновляем панель (HUD)
          - при выключении:
              * глушим таймеры
              * шлём нейтраль в моторы
              * останавливаем анимацию
              * is_running = False, trajectory_mode = False
        """
        st = self.state

        # ---- запрет при ручном режиме ----
        if getattr(st, "manual_drive", False):
            if checked:
                QtWidgets.QToolTip.showText(
                    QtGui.QCursor.pos(),
                    "Сначала выключите ручное управление (закройте окно мануала)",
                )
            btn = getattr(self, "btnTrajectory", None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
            st.trajectory_mode = False
            return

        # ---- включение траектории ----
        if checked:
            # Нельзя запускать, если нет траектории
            route_px = getattr(st, "route_pts_px", None) or []
            route_m  = getattr(st, "route_pts_m", None) or []
            if len(route_px) < 2 or len(route_m) < 2:
                QtWidgets.QToolTip.showText(
                    QtGui.QCursor.pos(),
                    "Нет траектории: задайте маршрут (Ровер+Цель) перед запуском.",
                )
                btn = getattr(self, "btnTrajectory", None)
                if btn is not None:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                st.trajectory_mode = False
                return

            st.trajectory_mode = True
            st.is_running = True
            print("[TRAJ] ON: движение по траектории", flush=True)

            # базовая скорость / PWM
            base = int(getattr(st, "pwm_base_us", 1750) or 1750)
            base = max(1750, min(2000, base))
            st.pwm_base_us = base
            st.l_pwm = base
            st.r_pwm = base

            # 1) стартовые значения (метрики маршрута и пр.)
            try:
                apply_start_defaults(st)
            except Exception as e:
                print("[TRAJ] apply_start_defaults error:", e, flush=True)

            # 2) пересчёт управляющих точек по уже готовому маршруту
            try:
                compute_controls_on_route(st, eps_px=2.0)
            except Exception as e:
                print("[TRAJ] compute_controls_on_route error:", e, flush=True)

            # 3) анимация маршрута (та же, что при обычном ПУСК)
            try:
                start_route_animation(self.ui, st)
            except Exception as e:
                print("[TRAJ] start_route_animation error:", e, flush=True)

            # 4) таймеры:
            #    - общий прогресс (флаги, jdist, HUD)
            if self._progress_timer and not self._progress_timer.isActive():
                self._progress_timer.start()
            #    - спец-логика траектории: угол, дельты, motors_set
            if self._traj_timer and not self._traj_timer.isActive():
                self._traj_timer.start()

            # 5) шлём базовый PWM + текущий B
            try:
                b = int(getattr(st, "b_pwm", 1500) or 1500)
                motors_set(st, base, base, b)
                print(f"[TRAJ] start motors L={base} R={base} B={b}", flush=True)
            except Exception as e:
                print("[TRAJ] motors_set start error:", e, flush=True)

            # 6) обновить HUD
            try:
                update_drive_panel(self.ui, st)
            except Exception as e:
                print("[TRAJ] panel update error:", e, flush=True)

        # ---- выключение траектории ----
        else:
            print("[TRAJ] OFF: остановка движения по траектории", flush=True)
            st.trajectory_mode = False
            st.is_running = False

            # стоп таймеров
            try:
                if self._traj_timer and self._traj_timer.isActive():
                    self._traj_timer.stop()
            except Exception:
                pass

            try:
                if self._progress_timer and self._progress_timer.isActive():
                    self._progress_timer.stop()
            except Exception:
                pass

            # нейтраль в моторы
            try:
                motors_set(st, 1500, 1500, getattr(st, "b_pwm", 1500) or 1500)
                print("[TRAJ] STOP → нейтраль L=1500 R=1500", flush=True)
            except Exception as e:
                print("[TRAJ] motors_set neutral error:", e, flush=True)

            # остановить анимацию маршрута (прогресс можно оставить)
            try:
                stop_route_animation(st, keep_progress=True)
            except Exception as e:
                print("[TRAJ] stop_route_animation error:", e, flush=True)

            # обновить HUD
            try:
                update_drive_panel(self.ui, st)
            except Exception as e:
                print("[TRAJ] panel post-stop update error:", e, flush=True)

    def _trajectory_tick(self):
        """
        Движение по пользовательской траектории.

        - работаем только если включён trajectory_mode;
        - ищем ближайшую точку маршрута;
        - берём локальный сегмент [seg_i -> seg_i+1];
        - считаем расстояние до вершины в МЕТРАХ;
        - если дальше 0.5 м — едем прямо (L=R);
        - если ближе 0.5 м — крутим по угловой ошибке (desired_yaw vs robot_heading_rad).
        """
        st = self.state
        if not getattr(st, "trajectory_mode", False):
            return

        pts = getattr(st, "route_pts_px", None)
        rp  = getattr(st, "robot_px", None)
        if not pts or len(pts) < 2 or not rp:
            return

        import math

        rx, ry = float(rp[0]), float(rp[1])

        # ---------- 1. ближайшая точка маршрута ----------
        min_i = 0
        min_d2 = 1e18
        for i, (x, y) in enumerate(pts):
            dx = x - rx
            dy = y - ry
            d2 = dx*dx + dy*dy
            if d2 < min_d2:
                min_d2 = d2
                min_i = i

        n = len(pts)
        if n < 2:
            return

        # сегмент [seg_i -> seg_i+1] (чтобы не улететь в min_i+1 за границу)
        seg_i = min(min_i, n - 2)
        vx1, vy1 = pts[seg_i]
        vx2, vy2 = pts[seg_i + 1]

        # ---------- 2. расстояние до вершины в МЕТРАХ ----------
        d1_px = math.hypot(vx1 - rx, vy1 - ry)
        d2_px = math.hypot(vx2 - rx, vy2 - ry)
        dist_to_vertex_px = min(d1_px, d2_px)

        mpx_x = float(getattr(st, "m_per_px_x", 0.8342405111938622) or 0.8342405111938622)
        mpx_y = float(getattr(st, "m_per_px_y", 1.2604320351524956) or 1.2604320351524956)
        mpp = 0.5 * (mpx_x + mpx_y)  # усреднённый м/px

        dist_to_vertex_m = dist_to_vertex_px * mpp
        TURN_RADIUS_M = 0.5  # за сколько метров до вершины начинаем крутить

        yaw = float(getattr(st, "robot_heading_rad", 0.0) or 0.0)

        # ---------- 3. если ещё далеко от вершины — едем прямо ----------
        if dist_to_vertex_m > TURN_RADIUS_M:
            if hasattr(st, "_traj_turning") and st._traj_turning:
                print(
                    f"[TRAJ] END TURN (вышли из зоны вершины)  "
                    f"dist={dist_to_vertex_m:.2f} м",
                    flush=True,
                )
                st._traj_turning = False

            base = int(getattr(st, "pwm_base_us", 1700) or 1700)
            base = max(1500, min(2000, base))
            L = base
            R = base
            try:
                motors_set(st, L, R, getattr(st, "b_pwm", 1500))
            except Exception as e:
                print("[TRAJ] motors_set straight ERROR:", e, flush=True)
            return

        # ---------- 4. мы в зоне поворота (≤ 0.5 м) ----------
        # локальное направление сегмента
        seg_dx = vx2 - vx1
        seg_dy = vy2 - vy1
        if abs(seg_dx) < 1e-6 and abs(seg_dy) < 1e-6:
            # сегмент выродился в точку — нечего рулить
            return

        desired_yaw = math.atan2(seg_dy, seg_dx)

        # ошибка курса
        err = desired_yaw - yaw
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi

        err_deg = math.degrees(err)

        TURN_LOG_THRESHOLD = 3.0  # только реальные повороты логируем

        if not hasattr(st, "_traj_turning"):
            st._traj_turning = False

        turning_now = abs(err_deg) >= TURN_LOG_THRESHOLD

        # ---------- ЛОГИ ПОВОРОТА ----------
        if turning_now:
            if not st._traj_turning:
                print(
                    f"[TRAJ] START TURN  err={err_deg:+.1f}°  "
                    f"dist={dist_to_vertex_m:.2f} м  "
                    f"vertex≈({(vx1+vx2)/2:.1f},{(vy1+vy2)/2:.1f})",
                    flush=True,
                )
            else:
                print(f"[TRAJ] turning  err={err_deg:+.1f}°", flush=True)
        else:
            if st._traj_turning:
                print(f"[TRAJ] END TURN  err={err_deg:+.1f}°", flush=True)

        st._traj_turning = turning_now

        # ---------- 5.е моторами ----------
        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = max(1500, min(2000, base))

        if not turning_now:
            delta = 0
        else:
            KP = 3.0  # усилие поворота; можно усилить/ослабить
            delta = int(KP * err_deg)

        MAX_DELTA = 250
        delta = max(-MAX_DELTA, min(MAX_DELTA, delta))

        # сглаживаем дельту, чтобы не дёргало
        if not hasattr(st, "_traj_pwm_smooth"):
            st._traj_pwm_smooth = 0.0
        st._traj_pwm_smooth = 0.5 * st._traj_pwm_smooth + 0.5 * float(delta)
        d = int(st._traj_pwm_smooth)

        L = base + d
        R = base - d
        L = max(1000, min(2000, L))
        R = max(1000, min(2000, R))

        try:
            motors_set(st, L, R, getattr(st, "b_pwm", 1500))
            if turning_now:
                print(f"[TRAJ] motors L={L} R={R} Δ={d:+d}", flush=True)
        except Exception as e:
            print("[TRAJ] motors_set turn ERROR:", e, flush=True) 

    # -------------------- ЛОГИ ДОРОГИ --------------------
    def _log_road_state(self, road_state: str, rf: float, sf: float):
        import json

        rec = {
            "ts": time.time(),
            "state": road_state,
            "rf": float(rf),
            "sf": float(sf),
        }
        self._road_log.append(rec)

        try:
            log_dir = os.path.join(HERE, "logs")
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, "roadseg.log")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception as e:
            print("[SEG] road log write error:", e, flush=True)

    def _ema(self, prev, new, alpha=0.4):
        if prev is None:
            return float(new)
        return float(alpha * new + (1.0 - alpha) * prev)
    # в обработчике нового скана переднего лидара:

    def on_front_lidar_points(self, pts_m):
        """
        pts_m: список [(x_m, y_m), ...] в МЕТРАХ в системе робота
        """
        # твоя логика безопасности / STOP
        self.state._lidar_front_last_pts = pts_m
        # ...
        # а теперь попытка "прищёлкнуть" позицию по лидарам
        snap_robot_by_lidar(
            self.state,
            self._lid_matcher,
            pts_m,
            min_points=30,
            min_score=0.95,
            max_route_diff_m=5.0,
        )
 
    # ---------------- ИНДИКАТОР «ДОРОГА» ----------------
    def _update_road_indicator(self):
        try:
            from status import set_indicator
        except Exception:
            return

        indRoad = self.ui.findChild(QtWidgets.QLabel, "indBattery")
        if not indRoad:
            return

        st = self.state
        rs = getattr(st, "road_state", "no_road")

        prev = getattr(self, "_road_state_prev_for_events", None)
        if prev is None:
            self._road_state_prev_for_events = rs
        elif rs != prev:
            if prev in ("no_road", "unknown", "far_road") and rs == "on_road":
                print(f"[ROAD] дорога НАЙДЕНА (prev={prev} -> on_road)", flush=True)
            elif prev == "on_road" and rs in ("no_road", "unknown", "far_road"):
                print(f"[ROAD] дорога ПОТЕРЯНА (prev=on_road -> {rs})", flush=True)
            self._road_state_prev_for_events = rs

        if rs == "on_road":
            set_indicator(indRoad, "ok")
        else:
            set_indicator(indRoad, "bad")

    
    def _road_follow_control(
        self,
        road_bin: np.ndarray,
        *,
        # =========================
        # базовое следование
        # =========================
        max_delta_pwm: int = 260,          # амплитуда поворота (PWM)
        steer_tau_s: float = 0.18,         # сглаживание steer (сек)
        min_total: float = 30.0,           # минимум “дороги” для расчёта центра масс

        # =========================
        # turn behaviour (bias от графа/перекрёстка)
        # =========================
        turn_light_target: float = 0.30,
        turn_mid_target: float = 0.45,
        turn_hard_target: float = 0.65,
        turn_light_deg: float = 30.0,
        turn_hard_deg: float = 110.0,

        # окно фаз (метры до перекрёстка)
        approach_m: float = 9.0,           # “начинаем смещаться”
        enter_turn_m: float = 2.5,         # “начинаем резкий поворот”

        # =========================
        # gating / clear
        # =========================
        cam_recent_s: float = 0.6,         # “камера видела перекрёсток недавно”
        pass_clear_m: float = 1.0,         # считаем перекрёсток пройденным после +N метров вдоль маршрута

        # =========================
        # вертикальные веса (НОРМА)
        # =========================
        top_skip_frac: float = 0.15,       # всегда выкидываем верх (небо/фон)
        bottom_focus_frac: float = 0.25,   # нижняя “полоса удержания”
        bottom_focus_weight: float = 0.70, # доля веса нижней полосы

        # =========================
        # вертикальные веса (ПОВОРОТ)
        # =========================
        turn_upper_from_bottom_lo: float = 0.50,
        turn_upper_from_bottom_hi: float = 0.85,
        turn_upper_weight: float = 0.70,

        # =========================
        # доп. защита от съезда
        # =========================
        edge_guard: bool = True,
        edge_guard_strength: float = 0.25,

        # =========================
        # STRAIGHT @ junction: усиливаем центр по X
        # (только когда граф говорит straight и камера видит перекрёсток)
        # =========================
        straight_center_frac: float = 0.60,     # ширина центральной зоны (доля кадра)
        straight_center_gain: float = 0.60,     # +60% веса центру  => множитель (1 + gain)
        straight_sides_cut: float = 0.60,       # -60% веса бокам   => множитель (1 - cut)
    ):
        """
        Road-follow:
        - базово держим центр масс дороги (по взвешенному по вертикали кадру)
        - при приближении к junction (граф) + подтверждении камеры -> добавляем bias (смещение цели)
        - сглаживаем steer low-pass (tau)
        - если в повороте "дороги впереди нет" (узкий FOV) -> держим прошлый steer
        - чистим fturn и его поля после прохождения junction по route_done_m
        - логи фаз: APPROACH (~approach_m), TURN (~enter_turn_m), PASSED — по одному разу
        """
        st = self.state

        # ---------- условия работы ----------
        if not getattr(st, "is_running", False):
            return
        if getattr(st, "manual_drive", False):
            return
        if getattr(st, "road_state", "no_road") != "on_road":
            return
        if road_bin is None or road_bin.size == 0:
            return

        H, W = road_bin.shape[:2]
        if H < 10 or W < 10:
            return

        # ------------------------------------------------------------
        # dt (оцениваем частоту тиков)
        # ------------------------------------------------------------
        tnow = time.monotonic()
        last_t = float(getattr(st, "_rf_last_ts", 0.0) or 0.0)
        if last_t > 0.0:
            dt = float(max(0.001, min(0.25, tnow - last_t)))
        else:
            dt = 0.05
        st._rf_last_ts = tnow
        st._rf_dt = dt

        # ------------------------------------------------------------
        # 1) Взвешенный центр масс дороги (по вертикали)
        # ------------------------------------------------------------
        top_skip = int(H * float(top_skip_frac))
        top_skip = max(0, min(H - 1, top_skip))

        img = road_bin[top_skip:, :]
        H2 = img.shape[0]
        if H2 <= 2:
            return

        # нижняя полоса удержания
        bf = float(bottom_focus_frac)
        bf = max(0.05, min(0.60, bf))
        bottom_h = int(H2 * bf)
        bottom_h = max(1, min(H2, bottom_h))

        bw = float(bottom_focus_weight)
        bw = max(0.50, min(0.95, bw))

        w_bottom = bw / float(bottom_h)
        w_rest = (1.0 - bw) / float(max(1, H2 - bottom_h))

        y_weights = np.ones((H2,), dtype=np.float32) * w_rest
        y_weights[H2 - bottom_h:] = w_bottom

        # ------------------------------------------------------------
        # 2) Базовый центр масс (по вертикально-взвешенной маске)
        # ------------------------------------------------------------
        img_f = img.astype(np.float32)
        col_scores = (img_f * y_weights[:, None]).sum(axis=0)

        total = float(col_scores.sum())
        if total < float(min_total):
            return

        xs = np.arange(W, dtype=np.float32)

        # ------------------------------------------------------------
        # 3) cam_recent (камера недавно видела перекрёсток)
        # ------------------------------------------------------------
        cam_ts = float(getattr(st, "_cam_junc_last_seen_ts", 0.0) or 0.0)
        cam_recent = (cam_ts > 0.0) and ((tnow - cam_ts) <= float(cam_recent_s))

        # ------------------------------------------------------------
        # 4) граф-хинт: turn + jdist + turn_deg
        # ------------------------------------------------------------
        jdist = getattr(st, "jdist", None)
        turn = getattr(st, "next_turn_dir", None)
        adeg = float(getattr(st, "turn_deg", 0.0) or 0.0)   # signed (+left, -right)
        abs_deg = float(abs(adeg))

        in_approach = (jdist is not None and float(jdist) <= float(approach_m))
        in_turnzone = (jdist is not None and float(jdist) <= float(enter_turn_m))

        allow_turn_bias = bool(in_approach and cam_recent and turn in ("left", "right", "straight"))

        # ------------------------------------------------------------
        # 4.1) STRAIGHT @ junction: усиливаем центр по X (гейтится allow_turn_bias)
        #     центр шириной straight_center_frac, центру +60%, бокам -60%
        # ------------------------------------------------------------
        if allow_turn_bias and turn == "straight":
            cfrac = float(straight_center_frac)
            cfrac = max(0.10, min(0.90, cfrac))

            wC = int(W * cfrac)
            wC = max(1, min(W, wC))
            wSide = (W - wC) // 2
            wL = wSide
            wR = W - (wL + wC)
            if wR < 1:
                wR = 1
                wC = max(1, W - wL - wR)

            gain = float(straight_center_gain)
            cut = float(straight_sides_cut)
            gain = max(0.0, min(2.0, gain))
            cut = max(0.0, min(0.95, cut))

            w = np.ones(W, dtype=np.float32)
            w[:wL] = (1.0 - cut)          # бокам -60% => 0.4
            w[wL:wL + wC] = (1.0 + gain)  # центру +60% => 1.6
            w[wL + wC:] = (1.0 - cut)

            col_scores = col_scores * w

            total2 = float(col_scores.sum())
            if total2 >= float(min_total):
                total = total2  # использовать усиленный total
            # если вдруг стало хуже (мало дороги), оставим старый total/col_scores (но col_scores уже умножен)
            # — это ок, т.к. при нехватке total мы всё равно вернёмся выше.

        x_center = float((xs * col_scores).sum() / total)
        offset = (x_center - (W / 2.0)) / (W / 2.0)
        offset = float(max(-1.0, min(1.0, offset)))

        # ------------------------------------------------------------
        # 5) Цель смещения (bias) от графа
        # ------------------------------------------------------------
        def target_for(turn_dir: str | None, abs_deg_val: float) -> float:
            if turn_dir == "left":
                if abs_deg_val < float(turn_light_deg):  return -float(turn_light_target)
                if abs_deg_val < float(turn_hard_deg):   return -float(turn_mid_target)
                return -float(turn_hard_target)
            if turn_dir == "right":
                if abs_deg_val < float(turn_light_deg):  return +float(turn_light_target)
                if abs_deg_val < float(turn_hard_deg):   return +float(turn_mid_target)
                return +float(turn_hard_target)
            return 0.0

        bias_target = 0.0
        if allow_turn_bias:
            bias_target = target_for(turn, abs_deg)

        # ------------------------------------------------------------
        # 6) В turn-zone: смотрим “верхнюю” полосу (узкий FOV), смешиваем центры
        # ------------------------------------------------------------
        no_road_ahead = False
        if allow_turn_bias and in_turnzone:
            lo = float(turn_upper_from_bottom_lo)
            hi = float(turn_upper_from_bottom_hi)
            lo = max(0.10, min(0.95, lo))
            hi = max(lo + 0.01, min(0.99, hi))

            y0 = int(H2 * (1.0 - hi))
            y1 = int(H2 * (1.0 - lo))
            y0 = max(0, min(H2 - 1, y0))
            y1 = max(y0 + 1, min(H2, y1))

            upper = img_f[y0:y1, :]
            if upper.size > 0:
                col_u = upper.sum(axis=0)
                tot_u = float(col_u.sum())
                if tot_u < float(min_total) * 0.35:
                    no_road_ahead = True
                else:
                    x_u = float((xs * col_u).sum() / tot_u)
                    tuw = float(turn_upper_weight)
                    tuw = max(0.0, min(0.95, tuw))
                    x_center2 = (1.0 - tuw) * x_center + tuw * x_u
                    offset2 = (x_center2 - (W / 2.0)) / (W / 2.0)
                    offset2 = float(max(-1.0, min(1.0, offset2)))
                    offset = offset2

        # ------------------------------------------------------------
        # 7) Итоговый offset: база + bias (подтягивание к цели)
        # ------------------------------------------------------------
        if allow_turn_bias:
            jd = float(jdist) if jdist is not None else 999.0
            alpha = 1.0 - max(0.0, min(1.0, jd / float(approach_m)))
            if in_turnzone:
                alpha = min(1.0, alpha + 0.35)
            offset = float((1.0 - alpha) * offset + alpha * float(bias_target))
            offset = float(max(-1.0, min(1.0, offset)))

        # ------------------------------------------------------------
        # 8) steer (low-pass). Если “стена” в повороте — держим прошлый steer.
        # ------------------------------------------------------------
        prev = float(getattr(st, "_rf_steer", 0.0) or 0.0)

        if allow_turn_bias and in_turnzone and no_road_ahead:
            steer = prev
        else:
            tau = float(steer_tau_s)
            tau = max(1e-4, tau)
            a = 1.0 - float(np.exp(-dt / tau))
            steer = prev + a * (offset - prev)

        steer = float(max(-1.0, min(1.0, steer)))
        st._rf_steer = steer

        # ------------------------------------------------------------
        # 9) edge_guard (не съезжай) — по нижней полосе
        # ------------------------------------------------------------
        if edge_guard:
            band = img_f[H2 - bottom_h:, :]
            if band.size > 0:
                col_b = band.sum(axis=0)
                tot_b = float(col_b.sum())
                if tot_b > float(min_total) * 0.5:
                    third = max(1, W // 3)
                    left_mass = float(col_b[:third].sum() / tot_b)
                    right_mass = float(col_b[W - third:].sum() / tot_b)

                    push = 0.0
                    if left_mass > 0.55:
                        push += float(edge_guard_strength) * (left_mass - 0.55)
                    if right_mass > 0.55:
                        push -= float(edge_guard_strength) * (right_mass - 0.55)

                    push = float(max(-0.25, min(0.25, push)))
                    steer2 = float(max(-1.0, min(1.0, steer + push)))
                    steer = steer2
                    st._rf_steer = steer

        # ------------------------------------------------------------
        # 10) Моторы
        # ------------------------------------------------------------
        def clamp(x, lo, hi):
            return lo if x < lo else hi if x > hi else x

        delta = int(clamp(steer * float(max_delta_pwm), -max_delta_pwm, max_delta_pwm))

        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = int(clamp(base, 1500, 2000))

        l = int(clamp(base + delta, 1000, 2000))
        r = int(clamp(base - delta, 1000, 2000))

        motors_set(st, l, r, None)

        # ------------------------------------------------------------
        # 11) Логи фаз (без спама)
        # ------------------------------------------------------------
        if allow_turn_bias and in_approach:
            if getattr(st, "_turn_phase", None) != "approach":
                st._turn_phase = "approach"
                print(f"[TURN][APPROACH] j_id={getattr(st, '_active_turn_j_id', None)} "
                    f"jdist={float(jdist):.1f}m turn={turn} deg={adeg:.1f}", flush=True)

        if allow_turn_bias and in_turnzone:
            if getattr(st, "_turn_phase", None) != "turn":
                st._turn_phase = "turn"
                print(f"[TURN][ENTER]    j_id={getattr(st, '_active_turn_j_id', None)} "
                    f"jdist={float(jdist):.2f}m turn={turn} deg={adeg:.1f}", flush=True)

        # ------------------------------------------------------------
        # 12) Clear if passed junction
        # ------------------------------------------------------------
        active_s = getattr(st, "_active_turn_s_m", None)
        if active_s is not None:
            done_m = float(getattr(st, "route_done_m", 0.0) or 0.0)
            if done_m >= float(active_s) + float(pass_clear_m):
                if getattr(st, "_turn_phase", None) is not None:
                    print(f"[TURN][PASSED]   j_id={getattr(st, '_active_turn_j_id', None)} done={done_m:.1f}m", flush=True)

                st.fturn = None
                st._fturn_ts = None
                st.fturn_l = 0.0
                st.fturn_c = 0.0
                st.fturn_r = 0.0

                st._active_turn_j_id = None
                st._active_turn_s_m = None

                st._turn_phase = None
                st._cam_junc_last_seen_ts = 0.0
    
    # ---------------- VIDEO / CAMERA MODE ----------------
    def _on_video_toggled(self, checked: bool):
        mode = "camera" if checked else "radar"
        self.state.video_mode = mode
        self._video_mode = mode

        btn = self.btnVideoMode
        if btn:
            btn.setText("Камера" if mode == "camera" else "Радар")

        if mode == "camera":
            # очищаем радарную сцену и показываем камеру
            if self._radar_scene:
                self._radar_scene.clear()
            self._radar_dots_group = None
        else:
            # убираем оверлей камеры и возвращаем радар
            self._remove_camera_overlay()
            self._ensure_radar_scene()
            self._restore_radar_view()

        print(f"[DRIVE] video mode -> {mode}", flush=True)

    def _ensure_camera_open(self):
        """Открываем камеру (или берём из state._cam) один раз."""
        if self._cam is not None and self._cam.isOpened():
            return True

        if getattr(self.state, "_cam", None) is not None and self.state._cam.isOpened():
            self._cam = self.state._cam
            return True

        try:
            cap = cv2.VideoCapture(self.state.cam_device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.state.cam_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.state.cam_h)
            cap.set(cv2.CAP_PROP_FPS, self.state.cam_fps)

            ok, _ = cap.read()
            if not ok:
                cap.release()
                self._cam = None
                self.state._cam = None
                self.state.camera_available = False
                print("[CAM] open failed", flush=True)
                return False

            self._cam = cap
            self.state._cam = cap
            self.state.camera_available = True
            print("[CAM] opened and ready", flush=True)
            return True
        except Exception as e:
            print("[CAM] open error:", e, flush=True)
            self._cam = None
            self.state._cam = None
            self.state.camera_available = False
            return False

    def _ensure_camera_timer(self):
        if self._cam_timer is None:
            self._cam_timer = QtCore.QTimer(self.ui)
            self._cam_timer.setInterval(33)  # ~30 FPS
            self._cam_timer.timeout.connect(self._camera_tick)
        if not self._cam_timer.isActive():
            self._cam_timer.start()

    def _camera_tick(self):
        if self._cam is None:
            return

        ok, frame = self._cam.read()
        if not ok or frame is None:
            return


        # ===== FPS МЕТРИКА (раз в 15 секунд) =====
        now = time.monotonic()
        st = self.state

        t0 = float(getattr(st, "_cam_fps_t0", 0.0) or 0.0)
        n  = int(getattr(st, "_cam_fps_n", 0) or 0)

        if t0 <= 0.0:
            st._cam_fps_t0 = now
            st._cam_fps_n = 0
        else:
            n += 1
            st._cam_fps_n = n
            dt = now - t0
            if dt >= 15.0:
                fps = n / dt
                st.cam_fps = float(fps)
                #print(f"[CAM FPS] {fps:.2f}  (n={n} dt={dt:.1f}s)", flush=True)
                st._cam_fps_t0 = now
                st._cam_fps_n = 0

        # дальше как было
        h, w = frame.shape[:2]
        # st уже есть, можно не переопределять, но можно оставить как у тебя:
        # st = self.state
        h, w = frame.shape[:2]
        st = self.state

        use_seg = (self._seg is not None) and getattr(self._seg, "ok", False)

        # -------- Вариант 1: без сегментации --------
        if not use_seg:
            st.road_state = "no_road"
            st.road_on = False
            st.road_frac = 0.0
            st.stripe_frac = 0.0
            st.road_frac_ema = None
            st.stripe_frac_ema = None
            self._mask_ema = None

            self._cam_tick_counter += 1
            prev_state = self._last_road_state
            if ("no_road" != prev_state) or (self._cam_tick_counter % 30 == 0):
                print("[CAM] road_state=no_road (seg OFF or no model)", flush=True)
            self._last_road_state = "no_road"

            self._log_road_state("no_road", 0.0, 0.0)

            if getattr(self.state, "video_mode", "radar") != "camera":
                return

            sc = self._radar_scene
            if sc is None:
                self._ensure_radar_scene()
                sc = self._radar_scene
            if sc is None or not self.radarViewDrive:
                return

            qimg = QtGui.QImage(frame.data, w, h, frame.strides[0], QtGui.QImage.Format_BGR888)
            pix = QtGui.QPixmap.fromImage(qimg)

            sc.clear()
            item = sc.addPixmap(pix)
            item.setZValue(1)
            self._cam_pixmap_item = item

            br = item.boundingRect()
            sc.setSceneRect(br)

            self.radarViewDrive.resetTransform()
            self.radarViewDrive.fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
            return

        # -------- Вариант 2: есть модель, но маска не получилась --------
        # -------- Вариант 2: есть модель, но маска не получилась --------
        try:
            mask = self._seg.infer(frame)
        except Exception as e:
            print("[CAM] seg infer error, fallback to no_road:", e, flush=True)
            mask = None

        if mask is None or mask.size == 0:
            st.road_state = "no_road"
            st.road_on = False
            st.road_frac = 0.0
            st.stripe_frac = 0.0
            st.road_frac_ema = None
            st.stripe_frac_ema = None
            self._mask_ema = None
            self._update_road_indicator()

            self._cam_tick_counter += 1
            prev_state = self._last_road_state
            if ("no_road" != prev_state) or (self._cam_tick_counter % 30 == 0):
                print("[CAM] road_state=no_road (mask is None)", flush=True)
            self._last_road_state = "no_road"

            self._log_road_state("no_road", 0.0, 0.0)

            if getattr(self.state, "video_mode", "radar") != "camera":
                return

            sc = self._radar_scene
            if sc is None:
                self._ensure_radar_scene()
                sc = self._radar_scene
            if sc is None or not self.radarViewDrive:
                return

            qimg = QtGui.QImage(frame.data, w, h, frame.strides[0], QtGui.QImage.Format_BGR888)
            pix = QtGui.QPixmap.fromImage(qimg)

            sc.clear()
            item = sc.addPixmap(pix)
            item.setZValue(1)
            self._cam_pixmap_item = item

            br = item.boundingRect()
            sc.setSceneRect(br)
            self.radarViewDrive.resetTransform()
            self.radarViewDrive.fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
            return

        # === ВСТАВКА 1: обновить хинт поворота в доль маршрута ===
        try:
            # смотрим вперёд на 15 м, мелкие изломы игнорируем (<35°)
            update_junction_turn_hint(st)        
        except Exception as e:
            print("[CAM] update_junction_turn_hint(state, announce_dist_m=15.0, eps_px=5.0, straight_deg=8.0) error:", e, flush=True)

        # === ВСТАВКА 2: попытаться снэпнуть робота к перекрёстку по камере ===
        try:
            # mask: HxW float32, как есть после infer
            maybe_snap_robot_to_junction_by_camera(st, mask)
            #print("[CAM] SNAPPED JUNC", flush=True)
        except Exception as e:
            print("[CAM] maybe_snap_robot_to_junction_by_camera error:", e, flush=True)

       # -------- Вариант 3: есть маска --------
        H, W = mask.shape[:2]

        # ===== Нижние 60% кадра (как вчера) =====
        y0 = int(H * 0.30)        # верхняя граница рабочего диапазона
        mask_crop = mask[y0:, :]  # нижние 70% маски
        Hc, Wc = mask_crop.shape

        thr = 0.51
        road_bin = (mask_crop > thr)

        # Доля дороги (как раньше)
        road_frac = float(road_bin.mean())

        # Нижняя 15% часть — дорога "под нами"
        stripe_h = max(1, int(Hc * 0.15))
        stripe_bin = road_bin[-stripe_h:, :]
        stripe_frac = float(stripe_bin.mean())

        # EMA
        st.road_frac_ema = self._ema(getattr(st, "road_frac_ema", None), road_frac)
        st.stripe_frac_ema = self._ema(getattr(st, "stripe_frac_ema", None), stripe_frac)

        rf = st.road_frac_ema
        sf = st.stripe_frac_ema

        # ---- Классификация дороги ----
        RF_NO_ROAD = 0.02
        SF_ON_ROAD = 0.08
        RF_FAR_ROAD = 0.05

        if rf < RF_NO_ROAD and sf < RF_NO_ROAD:
            road_state = "no_road"
        elif sf >= SF_ON_ROAD:
            road_state = "on_road"
        elif rf >= RF_FAR_ROAD:
            road_state = "far_road"
        else:
            road_state = "unknown"

        st.road_state = road_state
        st.road_on = (road_state == "on_road")

        # лог + индикатор
        self._cam_tick_counter += 1
        prev_state = self._last_road_state
        #if (road_state != prev_state) or (self._cam_tick_counter % 30 == 0):
           # print(f"[CAM] road_state={road_state} rf={rf:.3f} sf={sf:.3f}", flush=True)
        self._last_road_state = road_state

        # FSM автоповорота (как было), но НЕ в режиме траектории
        if not getattr(self.state, "trajectory_mode", False):
            try:
                finished = update_autoturn_fsm(self.state)
            except Exception as e:
                print("[TURN] autoturn FSM error:", e, flush=True)
                finished = False

            if finished and getattr(self.state, "turn_pending_route_start", False) and getattr(self.state, "is_running", False):
                # ... твой старый код запуска маршрута ...
                pass  # я тут не меняю

        self._log_road_state(road_state, rf, sf)
        self._update_road_indicator()

        # В РЕЖИМЕ ТРАЕКТОРИИ руление по дороге отключаем — рулит отдельный контроллер
        if not getattr(self.state, "trajectory_mode", False):
            self._road_follow_control(road_bin)

        # === 3.2. EMA по маске для зелёного оверлея ===
        if self._mask_ema is None or self._mask_ema.shape != mask.shape:
            self._mask_ema = mask.copy()
        else:
            self._mask_ema = 0.6 * self._mask_ema + 0.4 * mask

        m = (self._mask_ema * 255.0).astype("uint8")
        overlay = frame.copy()
        overlay[:, :, 1] = np.maximum(overlay[:, :, 1], m)
        blended = (0.7 * frame + 0.3 * overlay).astype("uint8")

        # === 3.4. Показ в виджете ===
        if getattr(self.state, "video_mode", "radar") != "camera":
            return

        sc = self._radar_scene
        if sc is None:
            self._ensure_radar_scene()
            sc = self._radar_scene
        if sc is None or not self.radarViewDrive:
            return

        qimg = QtGui.QImage(
            blended.data,
            w,
            h,
            blended.strides[0],
            QtGui.QImage.Format_BGR888,
        )
        pix = QtGui.QPixmap.fromImage(qimg)

        sc.clear()
        item = sc.addPixmap(pix)
        item.setZValue(1)
        self._cam_pixmap_item = item

        br = item.boundingRect()
        sc.setSceneRect(br)
        self.radarViewDrive.resetTransform()
        self.radarViewDrive.fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
        
    def _remove_camera_overlay(self):
        sc = self._radar_scene
        if sc is None:
            return
        sc.clear()
        self._cam_pixmap_item = None

    def _restore_radar_view(self):
        if not self.radarViewDrive:
            return
        if self._radar_scene is None:
            self._ensure_radar_scene()
        self._radar_scene.setSceneRect(-300, -300, 600, 600)
        self.radarViewDrive.resetTransform()
        self.radarViewDrive.fitInView(self._radar_scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _ensure_radar_scene(self):
        if not self.radarViewDrive:
            return
        sc = self.radarViewDrive.scene()
        if sc is None:
            sc = QtWidgets.QGraphicsScene(self.radarViewDrive)
            self.radarViewDrive.setScene(sc)
        self._radar_scene = sc
        self._draw_radar_grid()

    # ---------- РАДАР ----------
    def _setup_radar(self):
        # ПЕРЕДНИЙ
        if not self._radar_rx_front or not self._radar_rx_front.isRunning():
            self._radar_rx_front = LidarUdpReceiver(host="127.0.0.1", port=10000, parent=self)
            self._radar_rx_front.pointsReady.connect(self._on_radar_points_front, QtCore.Qt.QueuedConnection)
            self._radar_rx_front.start()
            print("[RADAR] FRONT listening on 127.0.0.1:10000", flush=True)

        # ЗАДНИЙ
        if not self._radar_rx_rear or not self._radar_rx_rear.isRunning():
            try:
                self._radar_rx_rear = LidarUdpReceiver(
                    host="127.0.0.1", port=10001, parent=self
                )
                self._radar_rx_rear.pointsReady.connect(
                    self._on_radar_points_rear, QtCore.Qt.QueuedConnection
                )
                self._radar_rx_rear.start()
                self.state.has_rear_lidar = True
                print("[RADAR] REAR listening on 127.0.0.1:10001", flush=True)
            except Exception as e:
                self._radar_rx_rear = None
                self.state.has_rear_lidar = False
                print("[RADAR] failed to start REAR UDP receiver:", e, flush=True)
                # если заднего лидара нет — лампа всегда зелёная
                try:
                    from status import set_indicator
                    indRear = self.ui.findChild(QtWidgets.QLabel, "indLidarRear")
                    if indRear:
                        set_indicator(indRear, "ok")
                except Exception:
                    pass


    def _stop_radar(self):
        """
        Останавливаем оба приёмника лидара (передний и задний).
        """
        for attr, handler in (
            ("_radar_rx_front", self._on_radar_points_front),
            ("_radar_rx_rear",  self._on_radar_points_rear),
        ):
            rx = getattr(self, attr, None)
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
            setattr(self, attr, None)

    def _on_radar_points(self, pts_xy_m):
        if not pts_xy_m:
            return

        self.state._lidar_last_pts = list(pts_xy_m)
        try:
            import time as _t
            self.state._lidar_last_ts = int(_t.time() * 1000)
        except Exception:
            self.state._lidar_last_ts = 0

        try:
            _lidar(self.ui, self.state)
        except Exception:
            pass

        if getattr(self.state, "video_mode", "radar") == "camera":
            # В режиме камеры радар внизу не рисуем
            return

        sc = self._radar_scene
        if sc is None:
            return

        r_def = QtCore.QRectF(-300, -300, 600, 600)
        if sc.sceneRect() != r_def:
            self._restore_radar_view()

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
            self._radar_dots_group = sc.createItemGroup(items)
            self._radar_dots_group.setZValue(5)

        item = getattr(self, "_cam_pixmap_item", None)
        if item:
            item.setVisible(False)

    def _draw_radar_grid(self):
        sc = self._radar_scene
        if sc is None:
            return
        r = sc.sceneRect()
        cx, cy = 0.0, 0.0
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
            items.append(sc.addEllipse(cx - rad, cy - rad, 2 * rad, 2 * rad, pen_grid))
            rad += step_px

        grp = sc.createItemGroup(items)
        grp.setZValue(1)

    # ---------- Завершение ----------
    def shutdown(self):
        # Сначала аккуратно останавливаем лидары
        try:
            self._stop_radar()
            print("[RADAR] stopped safely", flush=True)
        except Exception as e:
            print("[RADAR] shutdown error:", e, flush=True)

        # Дальше — как было: камера, ардуино и т.п.
        try:
            if hasattr(self.state, "_cam") and self.state._cam:
                self.state._cam.release()
                self.state._cam = None
                print("[CAMERA] released", flush=True)
        except Exception as e:
            print("[CAMERA] release error:", e, flush=True)

        try:
            from robot_cmd import close_link
            close_link()
        except Exception as e:
            print("[ARDUINO] close_link error:", e, flush=True)

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
        # Запрет ПУСК при включённом ручном режиме
        if getattr(self.state, "manual_drive", False):
            if checked:
                QtWidgets.QToolTip.showText(
                    QtGui.QCursor.pos(),
                    "Сначала выключите ручное управление (закройте окно мануала)",
                )
            if self.btnStartStop:
                self.btnStartStop.blockSignals(True)
                self.btnStartStop.setChecked(False)
                self.btnStartStop.blockSignals(False)
            self.state.is_running = False
            self._refresh_startstop_caption()
            self._update_controls()
            return

        self.state.is_running = bool(checked)
        self._refresh_startstop_caption()
        self._update_controls()

        from robot_cmd import update_drive_panel

        try:
            update_drive_panel(self.ui, self.state)
        except Exception as e:
            print("[DRIVE] panel pre-update error:", e, flush=True)

        if self.state.is_running:
            # Автоповорот перед стартом маршрута
            try:
                from robot_cmd import autoturn_on_route_start
                can_start = autoturn_on_route_start(self.state, pwm=200, eps_px=3.0)
            except Exception as e:
                print("[TURN] autoturn_on_route_start error:", e, flush=True)
                can_start = True

            if not can_start:
                return

            # 1) стартовые значения
            try:
                apply_start_defaults(self.state)
            except Exception as e:
                print("[DRIVE] apply_start_defaults error:", e, flush=True)

            # 2) пересчитать КТ / метрики
            try:
                compute_controls_on_route(self.state, eps_px=2.0)
            except Exception as e:
                print("[DRIVE] compute_controls_on_route error:", e, flush=True)

            # 3) анимация / прогресс
            try:
                start_route_animation(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] start_route_animation error:", e, flush=True)
            if self._progress_timer and not self._progress_timer.isActive():
                self._progress_timer.start()

            # 4) панель после старта
            try:
                update_drive_panel(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] panel post-start update error:", e, flush=True)

            QtCore.QTimer.singleShot(0, lambda: update_drive_panel(self.ui, self.state))

            # датасет
            self.state._dataset_active = bool(self.state.dataset_mode)
            if self.state._dataset_active:
                self.state._last_capture_m = max(
                    0.0,
                    float(getattr(self.state, "route_done_m", 0.0) or 0.0)
                    - float(getattr(self.state, "capture_step_m", 3.0) or 3.0),
                )

        else:
            # STOP
            try:
                if self._progress_timer and self._progress_timer.isActive():
                    self._progress_timer.stop()
            except Exception:
                pass
            # ---- ОТПРАВЛЯЕМ НЕЙТРАЛ В МОТОРЫ ----
            try:
                motors_set(self.state, 1500, 1500, None)
                print("[DRIVE] STOP → нейтраль L=1500 R=1500", flush=True)
            except Exception as e:
                print("[DRIVE] motors_set neutral error:", e, flush=True)
            try:
                stop_route_animation(self.state, keep_progress=True)
            except Exception as e:
                print("[DRIVE] stop_route_animation error:", e, flush=True)

            self.state._dataset_active = False

            try:
                update_drive_panel(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] panel post-stop update error:", e, flush=True)

            QtCore.QTimer.singleShot(0, lambda: update_drive_panel(self.ui, self.state))

    def _on_back_clicked(self):
        """DRIVE → IDLE: стоп всего и переход на IDLE."""
        try:
            if hasattr(self, "_progress_timer") and self._progress_timer.isActive():
                self._progress_timer.stop()
        except Exception:
            pass
        try:
            from graphics import stop_route_animation, clear_route_visual
            stop_route_animation(self.state, keep_progress=True)
            clear_route_visual(self.state, also_clear_data=False)
        except Exception:
            pass

        self.state.is_running = False
        self.state.safety_stop = False

        try:
            from status import set_indicator
            set_indicator(self.indBattery, "ok")
            set_indicator(self.indLidar, "ok")
        except Exception:
            pass

        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        pageIdle = self.ui.findChild(QtWidgets.QWidget, "pageIdle")
        if stack and pageIdle:
            stack.setCurrentWidget(pageIdle)

    # ---------------------------------------------------------------------
    # Переключатели режимов (ЦЕЛЬ / КР / РОВЕР)
    # ---------------------------------------------------------------------

    def _on_select_goal_drive_toggled(self, enabled: bool):
        self.state.select_goal_mode_drive = bool(enabled)
        if enabled:
            if self.btnKR and self.btnKR.isChecked():
                self.btnKR.setChecked(False)
                self.state.kr_mode_drive = False
            if self.btnRobotDrive and self.btnRobotDrive.isChecked():
                self.btnRobotDrive.setChecked(False)
                self.state.manual_loc_drive = False
        else:
            if self.state.is_running == False:
                # СБРОС целей и накопленного маршрута
                self.state.route_goal_px = []
                self.state.goal_px = None

                self.state.route_pts_px = []
                self.state.route_pts_m = []
                self.state.route_len_m = 0.0

                reset_route_progress(self.state)

                # перерисовка
                try:
                    redraw_route(self.state, self.ui)
                except Exception:
                    pass
                try:
                    update_drive_panel(self.ui, self.state)
                except Exception:
                    pass
                # маркеры (если нужно убрать флажок цели)
                for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                    if sc is not None:
                        try:
                            redraw_markers(self.state, sc)
                        except Exception:
                            pass

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
        self.state.manual_loc_drive = bool(enabled)
        if enabled:
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
            self.btnSelectGoalDrive.setText(
                "Цель" + (" (вкл.)" if self.state.select_goal_mode_drive else "")
            )

    def _refresh_kr_caption(self):
        if self.btnKR:
            self.btnKR.setText("КР" + (" (вкл.)" if self.state.kr_mode_drive else ""))

    def _refresh_robot_caption(self):
        if self.btnRobotDrive:
            self.btnRobotDrive.setText(
                "Ровер" + (" (вкл.)" if self.state.manual_loc_drive else "")
            )

    def _update_controls(self):
        running = bool(self.state.is_running)
        manual = bool(getattr(self.state, "manual_drive", False))

        # Кнопка "Назад"
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not running)

        # ЦЕЛЬ
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setEnabled(not running and not manual)
            if (running or manual) and self.btnSelectGoalDrive.isChecked():
                self.btnSelectGoalDrive.setChecked(False)
                self.state.select_goal_mode_drive = False
                self._refresh_select_goal_caption()

        # КР
        if self.btnKR:
            self.btnKR.setEnabled(not running and not manual)
            if (running or manual) and self.btnKR.isChecked():
                self.btnKR.setChecked(False)
                self.state.kr_mode_drive = False
                self._refresh_kr_caption()

        # РОВЕР
        if self.btnRobotDrive:
            self.btnRobotDrive.setEnabled(not running and not manual)
            if (running or manual) and self.btnRobotDrive.isChecked():
                self.btnRobotDrive.setChecked(False)
                self.state.manual_loc_drive = False
                self._refresh_robot_caption()

        # ПУСК/СТОП при ручном режиме блокируем
        if self.btnStartStop:
            self.btnStartStop.setEnabled(not manual)


    # ---------------------------------------------------------------------
    # Мануальный режим: вход/выход (используется диалогом)
    # ---------------------------------------------------------------------
    def _enter_manual_mode(self):
        """Входим в ручной режим: останавливаем маршрут, включаем manual_drive."""
        self.state.manual_drive = True

        # если маршрут шёл — останавливаем
        if self.state.is_running and self.btnStartStop:
            self.btnStartStop.blockSignals(True)
            self.btnStartStop.setChecked(False)
            self.btnStartStop.blockSignals(False)
            self.state.is_running = False
            try:
                stop_route_animation(self.state, keep_progress=True)
            except Exception:
                pass

        # ставим нейтраль
        self.state.manual_l_pwm = 1500
        self.state.manual_r_pwm = 1500

        self._update_controls()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _exit_manual_mode(self):
        """Выходим из ручного режима, но никуда не едем автоматически."""
        self.state.manual_drive = False

        self._update_controls()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _on_manual_window_clicked(self):
        """Открыть/активировать окно ручного управления."""
        if self._manual_window is None:
            self._manual_window = ManualDriveWindow(self.ui, self.state, self)

        self._manual_window.show()
        self._manual_window.raise_()
        self._manual_window.activateWindow()

    # ---------------------------------------------------------------------
    # Мануальное управление
    # ---------------------------------------------------------------------
    def _manual_cmd(self, direction: str):
        """Записываем в state желаемые PWM для ручного режима."""
        if not getattr(self.state, "manual_drive", False):
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                "Ручной режим включается при открытии окна мануального управления",
            )
            return

        base = int(getattr(self.state, "pwm_base_us", 1800) or 1800)
        base = max(1500, min(2000, base))
        delta = base - 1500  # 0..500

        if direction == "stop":
            l = r = 1500
        elif direction == "fwd":
            l = r = 1500 + delta
        elif direction == "back":
            l = r = 1500 - delta

        elif direction == "left":
            # поворот налево = левая медленнее, правая быстрее
            l = 1500 - delta
            r = 1500 + delta

        elif direction == "right":
            # поворот направо = левая быстрее, правая медленнее
            l = 1500 + delta
            r = 1500 - delta

        elif direction == "fwd_left":
            l = 1500
            r = 1500 + delta

        elif direction == "fwd_right":
            l = 1500 + delta
            r = 1500

        elif direction == "back_left":
            l = 1500 - delta
            r = 1500

        elif direction == "back_right":
            l = 1500
            r = 1500 - delta

        else:
            l = r = 1500

        # Клэмп
        l = max(1000, min(2000, l))
        r = max(1000, min(2000, r))

        self.state.manual_l_pwm = int(l)
        self.state.manual_r_pwm = int(r)

        print(f"[MANUAL] cmd={direction} L={l} R={r}", flush=True)

        # обновить HUD
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        # и реально отправить в Arduino
        try:
            motors_set(self.state, l, r, None)
        except Exception as e:
            print("[MANUAL] motors_set error:", e, flush=True)

    # ---------------------------------------------------------------------
    # Управление целевой скоростью (верхний бар)
    # ---------------------------------------------------------------------

    def _update_pwm_label(self):
        base = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
        self.lblPwm.setText(f"База PWM: {base} мкс")

    def _update_speed_label(self):
            """
            Показываем текущую скорость в м/с по базовому PWM:
            1000..1500 → -5..0 м/с
            1500..2000 → 0..5 м/с
            """
            pwm = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
            pwm = max(1000, min(2000, pwm))

            # 500 мкс → 5 м/с ⇒ 0.01 м/с на 1 мкс
            v_ms = (pwm - 1500) * 0.01

            # параллельно держим speed_mps в state
            self.state.speed_mps = v_ms

            if self.lblSpeed:
                self.lblSpeed.setText(f"Скорость: {v_ms:+.1f} м/с (PWM {pwm})")

    def _change_speed(self, delta: int):
            """
            Меняем базовый PWM (pwm_base_us) и, если маршрут запущен,
            сразу отправляем его через motors_set, чтобы обновить speed_mps.
            """
            base = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
            base = max(1500, min(2000, base + delta))
            self.state.pwm_base_us = base

            print(f"[DRIVE] pwm_base_us -> {base}", flush=True)

            # если маршрут сейчас идёт — сразу обновляем моторы
            if getattr(self.state, "is_running", False):
                try:
                    from robot_cmd import motors_set
                    motors_set(self.state, base, base, None)
                except Exception as e:
                    print("[DRIVE] motors_set error on speed change:", e, flush=True)

            # обновить панель (lblSpeed etc.)
            try:
                from robot_cmd import update_drive_panel
                update_drive_panel(self.ui, self.state)
            except Exception:
                pass
    # ---------------------------------------------------------------------
    # Зум карты
    # ---------------------------------------------------------------------
    def _on_zoom_in(self):
        if not self.mapViewDrive:
            return
        self._map_zoom *= 1.2
        self._apply_map_zoom()

    def _on_zoom_out(self):
        if not self.mapViewDrive:
            return
        self._map_zoom /= 1.2
        if self._map_zoom < 0.2:
            self._map_zoom = 0.2
        self._apply_map_zoom()

    def _apply_map_zoom(self):
        if not self.mapViewDrive:
            return
        self.mapViewDrive.resetTransform()
        self.mapViewDrive.scale(self._map_zoom, self._map_zoom)

    # ---------------------------------------------------------------------
    # Клик по карте
    # ---------------------------------------------------------------------
    def _on_drive_map_click(self, x_px: float, y_px: float):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Остановите движение")
            return
        if getattr(self.state, "manual_drive", False):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Выключите ручное управление")
            return
        if not getattr(self.state, "graph", None):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Граф карты не загружен")
            return

        # 1) Режим РОВЕР: ручная локализация
        if getattr(self.state, "manual_loc_drive", False):
            set_robot_pose_px((x_px, y_px), self.state)
            for sc in (
                getattr(self.state, "_idle_scene", None),
                getattr(self.state, "_drive_scene", None),
            ):
                if sc is not None:
                    try:
                        redraw_markers(self.state, sc)
                    except Exception:
                        pass
            if getattr(self.state, "goal_px", None):
                if build_route_from_robot_to_goal(self.state):
                    reset_route_progress(self.state)
                    recompute_route_metrics(self.state)
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

        # 2) Режим КР — флажки включения/выключения 3-го выхода (B)
        if getattr(self.state, "kr_mode_drive", False):
            st = self.state

            before_n = len(getattr(st, "control_pts_px", []) or [])

            # старая логика: привязка к графу, округление и т.п.
            set_control_item_px((x_px, y_px), st)

            after_n = len(getattr(st, "control_pts_px", []) or [])

            if after_n > before_n:
                # точка реально добавилась
                is_on = bool(getattr(st, "kr_b_next_on", True))
                kind = "b_on" if is_on else "b_off"

                # список типов флажков
                if not hasattr(st, "control_pts_kind") or st.control_pts_kind is None:
                    st.control_pts_kind = []

                # на всякий случай выровняем длину kinds с количеством точек,
                # если до этого что-то удаляли
                while len(st.control_pts_kind) < (after_n - 1):
                    st.control_pts_kind.append(None)

                st.control_pts_kind.append(kind)

                # просто запоминаем тип, НИЧЕГО не шлём в motors_set здесь
                # триггер будет только при пересечении флага (check_flag_collision_and_update)
                st.kr_b_next_on = not is_on

                print(f"[KR] added flag #{after_n} kind={kind}", flush=True)

            # перерисовка маркеров
            for sc in (
                getattr(st, "_idle_scene", None),
                getattr(st, "_drive_scene", None),
            ):
                if sc is not None:
                    try:
                        redraw_markers(st, sc)
                    except Exception:
                        pass

            if self.ui.statusBar():
                n = len(getattr(st, "control_pts_px", []) or [])
                self.ui.statusBar().showMessage(f"Добавлен флажок КР #{n}", 1200)
            return

        # 3) Цель
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                "Включите режим «Ровер», «КР» или «Цель».",
            )
            return
        # 1) Снэп + установка текущей цели
        goal_px = set_goal_px((x_px, y_px), self.state)  # если set_goal_px возвращает точку — отлично
        # если НЕ возвращает, то goal_px = self.state.goal_px

        # 2) Добавляем в список целей (route_goal_px)
        if not hasattr(self.state, "route_goal_px") or self.state.route_goal_px is None:
            self.state.route_goal_px = []
        self.state.route_goal_px.append(self.state.goal_px)  # или goal_px если он есть

        ...

        if getattr(self.state, "robot_px", None):
            ok = build_route_from_robot_to_goal(self.state)
            if ok:
                # reset_route_progress УБРАТЬ — маршрут теперь накапливается
                # reset_route_progress(self.state)

                # Можно оставить пересчёт КР (если он не чистит route_pts_px)
                try:
                    compute_controls_on_route(self.state, eps_px=2.0)
                except Exception:
                    pass

                redraw_route(self.state, self.ui)
                update_drive_panel(self.ui, self.state)
    # ---------------------------------------------------------------------
    # Таймер прогресса
    # ---------------------------------------------------------------------


    def _on_progress_tick(self):
        try:
            check_flag_collision_and_update(self.state)
        except Exception:
            pass

        # обновляем подсказку по перекрёстку (запишет state.jdist и state.next_turn_dir)
        try:
            update_junction_turn_hint(self.state)
        except Exception:
            pass

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # Переключение страниц
    # ---------------------------------------------------------------------
    
    def _on_stack_changed(self, idx: int):
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        if not stack:
            return
        page = stack.widget(idx)
        if page is self.pageDrive:
            self._setup_radar()
            try:
                from status import set_indicator
                set_indicator(self.indBattery, "ok")
                set_indicator(self.indLidar, "ok")
            except Exception:
                pass
        else:
            self._stop_radar()

    def _draw_radar_points(self, pts_xy_m):
            sc = self._radar_scene
            if sc is None:
                return

            r_def = QtCore.QRectF(-300, -300, 600, 600)
            if sc.sceneRect() != r_def:
                self._restore_radar_view()

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
                self._radar_dots_group = sc.createItemGroup(items)
                self._radar_dots_group.setZValue(5)

            item = getattr(self, "_cam_pixmap_item", None)
            if item:
                item.setVisible(False)

    def _on_radar_points_front(self, pts_xy_m):
        """
        Передний лидар:
          - сохраняем последние точки
          - проверяем стоп по переднему лида́ру
          - если было препятствие (safety_stop=True), а стало False
            и маршрут запущен → возобновляем движение через motors_set/apply_start_defaults
        """
        if not pts_xy_m:
            return

        st = self.state

        # сохраняем точки
        st._lidar_front_last_pts = list(pts_xy_m)
        st._lidar_last_pts = list(pts_xy_m)

        # запоминаем предыдущее состояние стопа
        prev_stop = bool(getattr(st, "safety_stop", False))

        # проверка стопа по лидару (может изменить st.safety_stop)
        try:
            from graphics import lidar_front_stop
            lidar_front_stop(self.ui, st)
        except Exception as e:
            print("[RADAR FRONT] lidar_front_stop error:", e, flush=True)

        cur_stop = bool(getattr(st, "safety_stop", False))

        # если было препятствие, а теперь нет — и мы в режиме ПУСК → возобновляем ход
        if prev_stop and not cur_stop and getattr(st, "is_running", False):
            try:
                base = int(getattr(st, "pwm_base_us", 1700) or 1700)
                base = max(1500, min(2000, base))
                print(f"[RADAR FRONT] obstacle cleared → resume motors at {base} us", flush=True)
                from robot_cmd import motors_set
                motors_set(st, base, base, None)
            except Exception as e:
                print("[RADAR FRONT] resume error:", e, flush=True)

        # рисуем радар, если включён режим «радар»
        if getattr(st, "video_mode", "radar") == "radar":
            self._draw_radar_points(pts_xy_m)

    def _on_radar_points_rear(self, pts_xy_m):
        if not pts_xy_m:
            return

        # Сохраняем точки заднего лидара
        self.state._lidar_rear_last_pts = list(pts_xy_m)

        # Проверка стопа по заднему
        try:
            from graphics import lidar_rear_stop
            lidar_rear_stop(self.ui, self.state)
        except Exception as e:
            print("[RADAR REAR] lidar_rear_stop error:", e, flush=True)


class ManualDriveWindow(QtWidgets.QDialog):
    """
    Отдельное окно с кнопками движения и +/− для PWM.
    Управляет state.manual_drive и полями manual_l_pwm / manual_r_pwm через DrivePage._manual_cmd().
    """
    def __init__(self, ui_parent, state, owner: "DrivePage"):
        super().__init__(ui_parent)
        self.state = state
        self.owner = owner

        self.setWindowTitle("Мануальное управление")
        self.setModal(False)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self.resize(300, 260)

        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setSpacing(8)

        # Текущий PWM
        self.lblPwm = QtWidgets.QLabel(self)
        self._update_pwm_label()
        vbox.addWidget(self.lblPwm, alignment=QtCore.Qt.AlignHCenter)

        # Кнопки движения (3x3)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)

        def add_btn(row, col, text, cmd):
            btn = QtWidgets.QPushButton(text, self)
            btn.setMinimumSize(90, 70)
            btn.clicked.connect(lambda _=None, c=cmd: self.owner._manual_cmd(c))
            grid.addWidget(btn, row, col)

        add_btn(0, 0, "↖", "fwd_left")
        add_btn(0, 1, "↑", "fwd")
        add_btn(0, 2, "↗", "fwd_right")

        add_btn(1, 0, "←", "left")
        add_btn(1, 1, "■", "stop")
        add_btn(1, 2, "→", "right")

        add_btn(2, 0, "↙", "back_left")
        add_btn(2, 1, "↓", "back")
        add_btn(2, 2, "↘", "back_right")

        vbox.addLayout(grid)

        # Строка с кнопками PWM +/−
        h_pwm = QtWidgets.QHBoxLayout()
        h_pwm.setSpacing(8)

        self.btnPwmMinus = QtWidgets.QPushButton("− PWM", self)
        self.btnPwmPlus = QtWidgets.QPushButton("+ PWM", self)
        self.btnPwmMinus.setMinimumHeight(32)
        self.btnPwmPlus.setMinimumHeight(32)

        self.btnPwmMinus.clicked.connect(lambda: self._change_pwm(-50))
        self.btnPwmPlus.clicked.connect(lambda: self._change_pwm(+50))



        h_pwm.addWidget(self.btnPwmMinus)
        h_pwm.addWidget(self.btnPwmPlus)

        vbox.addLayout(h_pwm)

        # Кнопка Назад
        self.btnBack = QtWidgets.QPushButton("Назад", self)
        self.btnBack.setMinimumHeight(36)
        self.btnBack.clicked.connect(self.close)
        vbox.addWidget(self.btnBack)

    def showEvent(self, ev: QtGui.QShowEvent):
        super().showEvent(ev)
        # Вход в мануальный режим
        self.owner._enter_manual_mode()

    def closeEvent(self, ev: QtGui.QCloseEvent):
        # Выход из мануального режима
        self.owner._exit_manual_mode()
        super().closeEvent(ev)
        

    def _change_pwm(self, delta):
        st = self.state
        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = max(1000, min(2000, base + delta))
        st.pwm_base_us = base
        self.lblPwm.setText(f"{base} µs")

    def _update_pwm_label(self):
        base = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
        self.lblPwm.setText(f"База PWM: {base} мкс")
  