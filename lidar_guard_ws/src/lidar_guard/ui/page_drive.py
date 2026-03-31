# -*- coding: utf-8 -*-
from typing import Tuple
from PyQt5 import QtWidgets, QtCore, QtGui
import os
import sys
import time

# === routing / graphics / robot_cmd как раньше ===
from routing import (
    set_goal_px,
    set_control_item_px,
    set_robot_pose_px,
    build_route_from_robot_to_goal,
    compute_controls_on_route,
    update_junction_turn_hint,
)
from graphics import (
    redraw_route,
    redraw_markers,
    start_route_animation,
    stop_route_animation,
    reset_route_progress,
    check_flag_collision_and_update,
    prepare_view,
    recompute_route_metrics,
)
from robot_cmd import (
    update_drive_panel,
    apply_start_defaults,
    motors_set,
    poll_arduino_startstop,
)

# === новые вынесенные подсистемы ===
from drive_radar import RadarController
from drive_video import VideoController
from drive_trajectory import TrajectoryController
from drive_manual import ManualController
from drive_dataset import DatasetController

Point = Tuple[float, float]


# режимы (приходят из Idle)
MODE_MARKERS = "markers"
MODE_ROAD = "road"
MODE_TRAJ = "traj"


def _qs() -> QtCore.QSettings:
    # единый namespace настроек
    return QtCore.QSettings("Rover", "PlatformGUI")


class DrivePage(QtCore.QObject):
    """
    ТОНКИЙ orchestrator:
    - UI wiring
    - переключатели режимов
    - клики по карте
    - старт/стоп маршрута
    - склейка контроллеров: radar/video/trajectory/manual/dataset
    """

    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__(ui)
        self.ui = ui
        self.state = state

        # ---- Виджеты DRIVE ----
        self.pageDrive: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageDrive")
        self.mapViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        self.radarViewDrive: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "radarViewDrive")

        # Зум карты
        self._map_zoom = 1.5
        self.btnZoomIn = ui.findChild(QtWidgets.QToolButton, "btnZoomIn")
        self.btnZoomOut = ui.findChild(QtWidgets.QToolButton, "btnZoomOut")

        # Верхний бар
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

        # Целевая скорость (Drive) — могут отсутствовать, но оставим
        self.btnSpeedMinusDrive: QtWidgets.QToolButton = ui.findChild(QtWidgets.QToolButton, "btnSpeedMinusDrive")
        self.btnSpeedPlusDrive: QtWidgets.QToolButton = ui.findChild(QtWidgets.QToolButton, "btnSpeedPlusDrive")

        # Траектория — в новой логике скрываем (режим выбирается в idle)
        self.btnTrajectory: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnTrajectory")

        # Мануальное окно
        self.btnManualDialog: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnManualDialog")

        # ---- Tuning panel (LOCK + sliders) ----
        self.btnTuneLock: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnTuneLock")

        # Row1 (always)
        self.sldPwmBase: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldPwmBase")
        self.lblPwmBaseVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblPwmBaseVal")

        self.sldLidarSolid: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldLidarSolid")
        self.lblLidarSolidVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblLidarSolidVal")

        # Row2 (always)
        self.sldSteerStrength: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldSteerStrength")
        self.lblSteerStrengthVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblSteerStrengthVal")

        self.sldSteerTau: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldSteerTau")
        self.lblSteerTauVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblSteerTauVal")

        self.sldSnapBefore: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldSnapBefore")
        self.lblSnapBeforeVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblSnapBeforeVal")

        # Row3 (road)
        self.sldRoadBias: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldRoadBias")
        self.lblRoadBiasVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblRoadBiasVal")

        self.sldSegThr: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldSegThr")
        self.lblSegThrVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblSegThrVal")

        self.sldJuncSearch: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldJuncSearch")
        self.lblJuncSearchVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblJuncSearchVal")

        # Row3 (markers) — должны быть добавлены в ui
        self.sldPoleDistDrive: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldPoleDistDrive")
        self.lblPoleDistDriveVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblPoleDistDriveVal")

        self.sldTurnDistDrive: QtWidgets.QSlider = ui.findChild(QtWidgets.QSlider, "sldTurnDistDrive")
        self.lblTurnDistDriveVal: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblTurnDistDriveVal")

        # контейнеры (в ui добавь эти widgets, можно просто QWidget с layout)
        self.boxDriveRow3Road: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "boxDriveRow3Road")
        self.boxDriveRow3Markers: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "boxDriveRow3Markers")

        # ---- init state defaults + load persisted ----
        self._load_persisted_settings()

        # state: lock flag
        if not hasattr(self.state, "tune_locked"):
            self.state.tune_locked = True  # по умолчанию залочено

        # ---- init state flags ----
        self.state.is_running = bool(getattr(self.state, "is_running", False))
        self.state.select_goal_mode_drive = bool(getattr(self.state, "select_goal_mode_drive", False))
        self.state.kr_mode_drive = bool(getattr(self.state, "kr_mode_drive", False))
        self.state.manual_loc_drive = bool(getattr(self.state, "manual_loc_drive", False))
        self.state.trajectory_mode = bool(getattr(self.state, "trajectory_mode", False))
        self.state.manual_drive = bool(getattr(self.state, "manual_drive", False))

        if not hasattr(self.state, "piece_goals_px") or self.state.piece_goals_px is None:
            self.state.piece_goals_px = []
        if not hasattr(self.state, "route_goal_px") or self.state.route_goal_px is None:
            self.state.route_goal_px = []

        # ---- mapView настройки ----
        if self.mapViewDrive:
            self.mapViewDrive.setRenderHints(
                QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
            )
            self.mapViewDrive.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.mapViewDrive.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.mapViewDrive.setMouseTracking(True)
            self.mapViewDrive.viewport().installEventFilter(self)
        # ✅ применить дефолтный зум сразу
        try:
            self._apply_map_zoom()
        except Exception:
            pass
        # ---- radarView: ОДНА сцена навсегда ----
        self._radar_scene_drive = None
        if self.radarViewDrive:
            sc = self.radarViewDrive.scene()
            if sc is None:
                sc = QtWidgets.QGraphicsScene(self.radarViewDrive)
                self.radarViewDrive.setScene(sc)
            self._radar_scene_drive = sc

            prepare_view(self.radarViewDrive)

            self.radarViewDrive.setRenderHints(
                QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
            )
            self.radarViewDrive.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.radarViewDrive.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.radarViewDrive.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
            self.radarViewDrive.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)

        # camera is auto-detected in VideoController; keep state.cam_device optional
        if not hasattr(self.state, "cam_device"):
            self.state.cam_device = None

        # ---- debounced save (QSettings) ----
        self._save_timer = QtCore.QTimer(self)
        self._save_timer.setInterval(300)
        self._save_timer.setSingleShot(True)
        self._save_timer.timeout.connect(self._save_persisted_settings)

        # ---- tuning wiring ----
        if self.btnTuneLock:
            self.btnTuneLock.setCheckable(True)
            # checked=True = UNLOCK
            self.btnTuneLock.setChecked(False)  # по UI у тебя так
            self.btnTuneLock.toggled.connect(self._on_tune_lock_toggled)

        if self.sldPwmBase:
            self.sldPwmBase.valueChanged.connect(self._on_pwm_base_changed)
        if self.sldLidarSolid:
            self.sldLidarSolid.valueChanged.connect(self._on_lidar_solid_changed)
        if self.sldRoadBias:
            self.sldRoadBias.valueChanged.connect(self._on_road_bias_changed)
        if self.sldSteerStrength:
            self.sldSteerStrength.valueChanged.connect(self._on_steer_strength_changed)
        if self.sldSteerTau:
            self.sldSteerTau.valueChanged.connect(self._on_steer_tau_changed)
        if self.sldSegThr:
            self.sldSegThr.valueChanged.connect(self._on_seg_thr_changed)
        if self.sldJuncSearch:
            self.sldJuncSearch.valueChanged.connect(self._on_junc_search_changed)
        if self.sldSnapBefore:
            self.sldSnapBefore.valueChanged.connect(self._on_pwm_bias_changed)
        if self.btnEStop:
            self.btnEStop.clicked.connect(QtWidgets.QApplication.quit)
        if self.sldPoleDistDrive:
            self.sldPoleDistDrive.valueChanged.connect(self._on_pole_dist_changed)
        if self.sldTurnDistDrive:
            self.sldTurnDistDrive.valueChanged.connect(self._on_turn_dist_changed)

        # ---- controllers ----
        # dataset: без ui (dataset не должен тащить UI)
        self.dataset = DatasetController(state=self.state)

        # radar
        self.radar = RadarController(ui=self.ui, state=self.state, radar_view=self.radarViewDrive)
        self.radar._scene = self._radar_scene_drive
        self.radar.setup()

        # video
        self.video = VideoController(
            ui=self.ui,
            state=self.state,
            radar_view=self.radarViewDrive,
            radar_scene_provider=lambda: self._radar_scene_drive,
            dataset=self.dataset,
        )

        # trajectory (оставляем контроллер, но кнопку убираем)
        self.trajectory = TrajectoryController(
            ui=self.ui,
            state=self.state,
            on_start=self._on_traj_started,
            on_stop=self._on_traj_stopped,
        )

        # manual
        self.manual = ManualController(ui=self.ui, state=self.state)

        # ---- timers ----
        self._progress_timer = QtCore.QTimer(self)
        self._progress_timer.setInterval(200)
        self._progress_timer.timeout.connect(self._on_progress_tick)
        if self.state.is_running:
            self._progress_timer.start()

        # ---- tuning log (debounced) ----
        self._tune_log_pending = {}
        self._tune_log_timer = QtCore.QTimer(self)
        self._tune_log_timer.setInterval(250)
        self._tune_log_timer.setSingleShot(True)
        self._tune_log_timer.timeout.connect(self._flush_tune_log)

        # ---- stack changed ----
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        if stack:
            stack.currentChanged.connect(self._on_stack_changed)

        # ---- wiring UI signals ----
        if self.btnStartStop:
            self.btnStartStop.setCheckable(True)
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)
            self._refresh_startstop_caption()

        # В НОВОМ FLOW: траектория выбирается на idle, кнопку убираем
        if self.btnTrajectory:
            self.btnTrajectory.setVisible(False)
            self.btnTrajectory.setEnabled(False)

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
            self.btnVideoMode.setCheckable(True)
            self.btnVideoMode.setChecked(getattr(self.state, "video_mode", "radar") == "camera")
            self.btnVideoMode.toggled.connect(self._on_video_toggled)

        if self.btnManualDialog:
            self.btnManualDialog.clicked.connect(self._on_manual_window_clicked)

        if self.btnZoomIn:
            self.btnZoomIn.clicked.connect(self._on_zoom_in)
        if self.btnZoomOut:
            self.btnZoomOut.clicked.connect(self._on_zoom_out)

        # ---- initial UI sync ----
        self._apply_mode_tuning_ui()
        self._update_controls()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        try:
            self._sync_tuning_from_state()
            self._apply_tuning_lock_ui()
        except Exception:
            pass

        # стартуем камеру-таймер
        QtCore.QTimer.singleShot(800, self.video.ensure_started)
    def _retarget_route_progress_to_robot_pose(self):
        """
        Коррекция текущего маршрута без перестроения:
        - оставляем route_pts_px как есть
        - находим ближайшую точку на полилинии маршрута к текущей позе робота
        - выставляем route_progress_idx / route_seg_off_m / route_done_m так,
        чтобы движение продолжилось с этого места
        """
        pts = list(getattr(self.state, "route_pts_px", []) or [])
        if len(pts) < 2:
            return

        rp = getattr(self.state, "robot_px", None)
        if rp is None:
            return

        rx, ry = float(rp[0]), float(rp[1])

        # scale: м/пикс (если есть длины)
        route_len_px = float(getattr(self.state, "route_len_px", 0.0) or 0.0)
        route_len_m = float(getattr(self.state, "route_len_m", 0.0) or 0.0)
        m_per_px = (route_len_m / route_len_px) if (route_len_px > 1e-9 and route_len_m > 1e-9) else 0.0

        # Кумиулятив по пикселям (если нет — построим)
        cum_px = list(getattr(self.state, "route_cum_px", None) or [])
        if len(cum_px) != len(pts):
            cum_px = [0.0]
            for i in range(len(pts) - 1):
                x0, y0 = pts[i]
                x1, y1 = pts[i + 1]
                dx, dy = float(x1) - float(x0), float(y1) - float(y0)
                cum_px.append(cum_px[-1] + (dx * dx + dy * dy) ** 0.5)
            self.state.route_cum_px = cum_px

        best_d2 = 1e30
        best_i = 0
        best_t = 0.0

        # Ищем ближайшую проекцию на любом сегменте маршрута
        for i in range(len(pts) - 1):
            x0, y0 = float(pts[i][0]), float(pts[i][1])
            x1, y1 = float(pts[i + 1][0]), float(pts[i + 1][1])
            vx, vy = x1 - x0, y1 - y0
            vv = vx * vx + vy * vy
            if vv < 1e-9:
                # вырожденный сегмент
                dx, dy = rx - x0, ry - y0
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2, best_i, best_t = d2, i, 0.0
                continue

            # проекция t на отрезок [0..1]
            t = ((rx - x0) * vx + (ry - y0) * vy) / vv
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0

            px = x0 + t * vx
            py = y0 + t * vy
            dx, dy = rx - px, ry - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2, best_i, best_t = d2, i, t

        # Теперь выставляем прогресс:
        # - idx = сегмент best_i
        # - seg_off_m = сколько метров пройдено по этому сегменту (t * seg_len_px * m_per_px)
        # - done_m = сколько метров пройдено по всему маршруту
        x0, y0 = float(pts[best_i][0]), float(pts[best_i][1])
        x1, y1 = float(pts[best_i + 1][0]), float(pts[best_i + 1][1])
        seg_len_px = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5

        done_px = float(cum_px[best_i]) + best_t * seg_len_px

        self.state.route_progress_idx = int(best_i)
        self.state.route_seg_off_m = float(done_px * m_per_px) - float(getattr(self.state, "route_cum_m", [0.0] * len(pts))[best_i]) if hasattr(self.state, "route_cum_m") and len(getattr(self.state, "route_cum_m", [])) == len(pts) else float(best_t * seg_len_px * m_per_px)
        self.state.route_done_m = float(done_px * m_per_px) if m_per_px > 0.0 else float(getattr(self.state, "route_done_m", 0.0) or 0.0)

        # На всякий — если мы перескочили ближе к концу, маршрут не считаем завершенным принудительно
        self.state.route_finished = False
    # ---------------------------------------------------------------------
    # Persistence
    # ---------------------------------------------------------------------
    def _load_persisted_settings(self):
        s = _qs()
        st = self.state

        # режим (устанавливается на idle, но подстрахуем)
        if not hasattr(st, "nav_mode") or not st.nav_mode:
            st.nav_mode = s.value("idle/nav_mode", MODE_MARKERS)

        # common drive
        st.pwm_base_us = int(s.value("drive/pwm_base_us", getattr(st, "pwm_base_us", 1700) or 1700))
        st.lidar_stop_solidness = float(
            s.value("drive/lidar_stop_solidness", getattr(st, "lidar_stop_solidness", 0.70) or 0.70)
        )
        st.max_delta_pwm = int(s.value("drive/max_delta_pwm", getattr(st, "max_delta_pwm", 120) or 120))
        st.steer_tau = float(s.value("drive/steer_tau", getattr(st, "steer_tau", 0.20) or 0.20))
        st.pwm_bias = int(s.value("drive/pwm_bias", getattr(st, "pwm_bias", 0) or 0))

        # road-only
        st.road_center_bias = float(s.value("road/road_center_bias", getattr(st, "road_center_bias", 0.06) or 0.06))
        st.seg_thr = float(s.value("road/seg_thr", getattr(st, "seg_thr", 0.35) or 0.35))
        st.junction_search_m = int(
            s.value("road/junction_search_m", getattr(st, "junction_search_m", 10) or 10)
        )
        st.search_ahead_m = float(getattr(st, "junction_search_m", 10))
        st.search_behind_m = float(getattr(st, "junction_search_m", 10))

        # markers-only
        st.marker_target_side_m = float(s.value("markers/marker_target_side_m", getattr(st, "marker_target_side_m", 2.0) or 2.0))
        st.marker_turn_trigger_m = int(s.value("markers/marker_turn_trigger_m", getattr(st, "marker_turn_trigger_m", 4) or 4))

        # lock state (опционально сохранять)
        if not hasattr(st, "tune_locked"):
            st.tune_locked = True

    def _save_persisted_settings_debounced(self):
        if self._save_timer:
            self._save_timer.start()
    def _silent_uncheck_ui_only(self, btn):
        """Только UI: отжать кнопку без сигналов. НИЧЕГО не трогаем в state."""
        if btn and btn.isChecked():
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)

    def _silent_uncheck(self, btn, state_attr: str, refresh_fn):
        """
        Для взаимного выключения режимов (когда пользователь включил другой режим).
        Тут state менять можно, потому что это осознанная смена режима.
        """
        self._silent_uncheck_ui_only(btn)
        setattr(self.state, state_attr, False)
        try:
            refresh_fn()
        except Exception:
            pass
    def _save_persisted_settings(self):
        s = _qs()
        st = self.state

        s.setValue("idle/nav_mode", getattr(st, "nav_mode", MODE_MARKERS))

        s.setValue("drive/pwm_base_us", int(getattr(st, "pwm_base_us", 1700) or 1700))
        s.setValue("drive/lidar_stop_solidness", float(getattr(st, "lidar_stop_solidness", 0.70) or 0.70))
        s.setValue("drive/max_delta_pwm", int(getattr(st, "max_delta_pwm", 120) or 120))
        s.setValue("drive/steer_tau", float(getattr(st, "steer_tau", 0.20) or 0.20))
        s.setValue("drive/pwm_bias", int(getattr(st, "pwm_bias", 0) or 0))

        s.setValue("road/road_center_bias", float(getattr(st, "road_center_bias", 0.06) or 0.06))
        s.setValue("road/seg_thr", float(getattr(st, "seg_thr", 0.35) or 0.35))
        s.setValue("road/junction_search_m", int(getattr(st, "junction_search_m", 10) or 10))
        s.setValue("markers/marker_target_side_m",
                float(getattr(st, "marker_target_side_m", 2.0) or 2.0))
        s.setValue("markers/marker_turn_trigger_m", int(getattr(st, "marker_turn_trigger_m", 4) or 4))

    # ---------------------------------------------------------------------
    # Mode-dependent tuning UI
    # ---------------------------------------------------------------------
    def _apply_mode_tuning_ui(self):
        mode = getattr(self.state, "nav_mode", MODE_MARKERS)

        markers_names = [
            "boxDriveRow3Markers",
            "lblPoleDistDriveTitle","lblPoleDistDriveMin","lblPoleDistDriveMax","lblPoleDistDriveVal","sldPoleDistDrive",
            "lblTurnDistDriveTitle","lblTurnDistDriveMin","lblTurnDistDriveMax","lblTurnDistDriveVal","sldTurnDistDrive",
        ]

        road_names = [
            "boxDriveRow3Road",
            "lblRoadBiasTitle","lblRoadBiasMin","lblRoadBiasMax","lblRoadBiasVal","sldRoadBias",
            "lblJuncSearchTitle","lblJuncSearchMin","lblJuncSearchMax","lblJuncSearchVal","sldJuncSearch",
            "lblSegThrTitle","lblSegThrMin","lblSegThrMax","lblSegThrVal","sldSegThr",
        ]

        if mode == MODE_TRAJ:
            self._set_visible_by_names(markers_names, False)
            self._set_visible_by_names(road_names, False)
            return

        self._set_visible_by_names(markers_names, mode == MODE_MARKERS)
        self._set_visible_by_names(road_names, mode == MODE_ROAD)
    def _set_visible_by_names(self, names, visible: bool):
        for n in names:
            w = self.ui.findChild(QtWidgets.QWidget, n)
            if w:
                w.setVisible(visible)
                w.setEnabled(visible)
    # DEBUG tune
    def _tune_log(self, key: str, value):
        """Debounced log: печатаем пачкой, чтобы не спамить при перетаскивании."""
        self._tune_log_pending[key] = value
        if self._tune_log_timer:
            self._tune_log_timer.start()

    def _flush_tune_log(self):
        if not self._tune_log_pending:
            return
        msg = " | ".join([f"{k}={v}" for k, v in self._tune_log_pending.items()])
        print("[TUNE]", msg, flush=True)
        self._tune_log_pending.clear()

    # ---------------------------------------------------------------------
    # SLIDERS LOCK
    # ---------------------------------------------------------------------
    def _on_tune_lock_toggled(self, checked: bool):
        """
        LOCK:
          - checked=True  -> UNLOCK (можно менять)
          - checked=False -> LOCK (нельзя менять)
        """
        self.state.tune_locked = not bool(checked)  # locked = not checked
        self._apply_tuning_lock_ui()

        if self.state.tune_locked:
            self._sync_tuning_from_state()

    def _apply_tuning_lock_ui(self):
        unlocked = (not bool(getattr(self.state, "tune_locked", True)))

        # всегда доступность зависит от lock, но видимость зависит от режима
        for w in (
            self.sldPwmBase,
            self.sldLidarSolid,
            self.sldSteerStrength,
            self.sldSteerTau,
            self.sldSnapBefore,  # pwm bias
            # road row3
            self.sldRoadBias,
            self.sldSegThr,
            self.sldJuncSearch,
            # markers row3
            self.sldPoleDistDrive,
            self.sldTurnDistDrive,
        ):
            if w:
                w.setEnabled(bool(unlocked))

    def _sync_tuning_from_state(self):
        st = self.state

        # PWM base
        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = max(1500, min(2000, base))
        if self.sldPwmBase:
            self.sldPwmBase.blockSignals(True)
            self.sldPwmBase.setValue(base)
            self.sldPwmBase.blockSignals(False)
        if self.lblPwmBaseVal:
            self.lblPwmBaseVal.setText(str(base))

        # lidar solidness 0.20..0.98
        solid = float(getattr(st, "lidar_stop_solidness", 0.70) or 0.70)
        solid = max(0.20, min(0.98, solid))
        if self.sldLidarSolid:
            self.sldLidarSolid.blockSignals(True)
            self.sldLidarSolid.setValue(int(round(solid * 100)))
            self.sldLidarSolid.blockSignals(False)
        if self.lblLidarSolidVal:
            self.lblLidarSolidVal.setText(f"{solid:.2f}")

        # steer strength 80..420
        m = int(getattr(st, "max_delta_pwm", 120) or 120)
        m = max(80, min(420, m))
        if self.sldSteerStrength:
            self.sldSteerStrength.blockSignals(True)
            self.sldSteerStrength.setValue(m)
            self.sldSteerStrength.blockSignals(False)
        if self.lblSteerStrengthVal:
            self.lblSteerStrengthVal.setText(str(m))

        # steer tau 0.05..0.60 stored in state.steer_tau
        tau = float(getattr(st, "steer_tau", 0.20) or 0.20)
        tau = max(0.05, min(0.60, tau))
        if self.sldSteerTau:
            self.sldSteerTau.blockSignals(True)
            self.sldSteerTau.setValue(int(round(tau * 100)))
            self.sldSteerTau.blockSignals(False)
        if self.lblSteerTauVal:
            self.lblSteerTauVal.setText(f"{tau:.2f}")

        # PWM bias: -10..+10 stored in state.pwm_bias
        pb = int(getattr(st, "pwm_bias", 0) or 0)
        pb = max(-10, min(10, pb))
        if self.sldSnapBefore:
            self.sldSnapBefore.blockSignals(True)
            # диапазон должен быть -10..10 в ui
            self.sldSnapBefore.setValue(pb)
            self.sldSnapBefore.blockSignals(False)
        if self.lblSnapBeforeVal:
            self.lblSnapBeforeVal.setText(str(pb))

        # road center bias -0.45..+0.45
        bias = float(getattr(st, "road_center_bias", 0.0) or 0.0)
        bias = max(-0.45, min(0.45, bias))
        if self.sldRoadBias:
            self.sldRoadBias.blockSignals(True)
            self.sldRoadBias.setValue(int(round(bias * 100)))
            self.sldRoadBias.blockSignals(False)
        if self.lblRoadBiasVal:
            self.lblRoadBiasVal.setText(f"{bias:+.2f}".replace("+", ""))

        # seg threshold 0.35..0.85
        thr = float(getattr(st, "seg_thr", 0.60) or 0.60)
        thr = max(0.35, min(0.85, thr))
        if self.sldSegThr:
            self.sldSegThr.blockSignals(True)
            self.sldSegThr.setValue(int(round(thr * 100)))
            self.sldSegThr.blockSignals(False)
        if self.lblSegThrVal:
            self.lblSegThrVal.setText(f"{thr:.2f}")

        # junction search meters 1..25
        js = int(getattr(st, "junction_search_m", 10) or 10)
        js = max(1, min(25, js))
        if self.sldJuncSearch:
            self.sldJuncSearch.blockSignals(True)
            self.sldJuncSearch.setValue(js)
            self.sldJuncSearch.blockSignals(False)
        if self.lblJuncSearchVal:
            self.lblJuncSearchVal.setText(str(js))

        # markers: pole distance 1..5
        pd = float(getattr(st, "marker_target_side_m", 2.0) or 2.0)
        pd = max(0.5, min(5.0, pd))

        if self.sldPoleDistDrive:
            self.sldPoleDistDrive.setMinimum(1)
            self.sldPoleDistDrive.setMaximum(10)   # ← диапазон
            self.sldPoleDistDrive.setSingleStep(1)
            self.sldPoleDistDrive.blockSignals(True)
            self.sldPoleDistDrive.setValue(int(round(pd * 2)))  # ← масштаб ×2
            self.sldPoleDistDrive.blockSignals(False)

        if self.lblPoleDistDriveVal:
            self.lblPoleDistDriveVal.setText(f"{pd:.1f} м")

        # markers: turn distance 1..8
        td = int(getattr(st, "marker_turn_trigger_m", 4) or 4)
        td = max(1, min(8, td))
        if self.sldTurnDistDrive:
            self.sldTurnDistDrive.blockSignals(True)
            self.sldTurnDistDrive.setValue(td)
            self.sldTurnDistDrive.blockSignals(False)
        if self.lblTurnDistDriveVal:
            self.lblTurnDistDriveVal.setText(f"{td} м")

        # lock button reflect state
        if self.btnTuneLock:
            want_checked = not bool(getattr(st, "tune_locked", True))
            self.btnTuneLock.blockSignals(True)
            self.btnTuneLock.setChecked(want_checked)
            self.btnTuneLock.blockSignals(False)

        # видимость строк под режим
        self._apply_mode_tuning_ui()

    # ---------------------------------------------------------------------
    # Slider handlers (common)
    # ---------------------------------------------------------------------
    def _on_pwm_base_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        v = int(max(1500, min(2000, int(v))))
        if self.lblPwmBaseVal:
            self.lblPwmBaseVal.setText(str(v))

        cur = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
        delta = v - cur
        if delta != 0:
            self._change_speed(delta)

        self._tune_log("pwm_base_us", v)
        self._save_persisted_settings_debounced()

    def _on_lidar_solid_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        solid = max(0.20, min(0.98, float(v) / 100.0))
        self.state.lidar_stop_solidness = solid
        if self.lblLidarSolidVal:
            self.lblLidarSolidVal.setText(f"{solid:.2f}")
        self._tune_log("lidar_stop_solidness", f"{solid:.2f}")
        self._save_persisted_settings_debounced()

    def _on_steer_strength_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        v = int(max(80, min(420, int(v))))
        self.state.max_delta_pwm = v
        if self.lblSteerStrengthVal:
            self.lblSteerStrengthVal.setText(str(v))
        self._tune_log("max_delta_pwm", v)
        self._save_persisted_settings_debounced()

    def _on_steer_tau_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        tau = max(0.05, min(0.60, float(v) / 100.0))
        self.state.steer_tau = tau
        if self.lblSteerTauVal:
            self.lblSteerTauVal.setText(f"{tau:.2f}")
        self._tune_log("steer_tau", f"{tau:.2f}")
        self._save_persisted_settings_debounced()

    def _on_pwm_bias_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        v = int(max(-10, min(10, int(v))))
        self.state.pwm_bias = v
        if self.lblSnapBeforeVal:
            self.lblSnapBeforeVal.setText(str(v))
        self._tune_log("pwm_bias", v)
        self._save_persisted_settings_debounced()

    # ---------------------------------------------------------------------
    # Slider handlers (road)
    # ---------------------------------------------------------------------
    def _on_road_bias_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        bias = max(-0.45, min(0.45, float(v) / 100.0))
        self.state.road_center_bias = bias
        if self.lblRoadBiasVal:
            self.lblRoadBiasVal.setText(f"{bias:.2f}")
        self._tune_log("road_center_bias", f"{bias:.2f}")
        self._save_persisted_settings_debounced()

    def _on_seg_thr_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        thr = max(0.35, min(0.85, float(v) / 100.0))
        self.state.seg_thr = thr
        if self.lblSegThrVal:
            self.lblSegThrVal.setText(f"{thr:.2f}")
        self._tune_log("seg_thr", f"{thr:.2f}")
        self._save_persisted_settings_debounced()

    def _on_junc_search_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        v = int(max(1, min(25, int(v))))
        self.state.junction_search_m = v
        self.state.search_ahead_m = float(v)
        self.state.search_behind_m = float(v)

        if self.lblJuncSearchVal:
            self.lblJuncSearchVal.setText(str(v))

        self._tune_log("junction_search_m", v)
        self._save_persisted_settings_debounced()

    # ---------------------------------------------------------------------
    # Slider handlers (markers)
    # ---------------------------------------------------------------------
    def _on_pole_dist_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        # v — это значение слайдера (×2)
        dist_m = float(v) * 0.5
        dist_m = max(0.5, min(5.0, dist_m))
        self.state.marker_target_side_m = dist_m
        if self.lblPoleDistDriveVal:
            self.lblPoleDistDriveVal.setText(f"{dist_m:.1f} м")
        self._tune_log("marker_target_side_m", f"{dist_m:.1f}")
        self._save_persisted_settings_debounced()


    def _on_turn_dist_changed(self, v: int):
        if bool(getattr(self.state, "tune_locked", True)):
            self._sync_tuning_from_state()
            return

        v = int(max(1, min(8, int(v))))
        self.state.marker_turn_trigger_m = v
        if self.lblTurnDistDriveVal:
            self.lblTurnDistDriveVal.setText(f"{v} м")
        self._tune_log("marker_turn_trigger_m", v)
        self._save_persisted_settings_debounced()

    # ---------------------------------------------------------------------
    # Trajectory glue (оставляем для совместимости, но UI кнопки нет)
    # ---------------------------------------------------------------------
    def _on_trajectory_toggled(self, checked: bool):
        self.trajectory.set_enabled(bool(checked))
        want = bool(checked)
        have = bool(getattr(self.state, "trajectory_mode", False))
        if self.btnTrajectory and want != have:
            self.btnTrajectory.blockSignals(True)
            self.btnTrajectory.setChecked(have)
            self.btnTrajectory.blockSignals(False)

    def _on_traj_started(self):
        if self._progress_timer and not self._progress_timer.isActive():
            self._progress_timer.start()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _on_traj_stopped(self):
        if self._progress_timer and self._progress_timer.isActive():
            self._progress_timer.stop()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # Video mode
    # ---------------------------------------------------------------------
    def _on_video_toggled(self, checked: bool):
        mode = "camera" if checked else "radar"
        self.state.video_mode = mode
        if self.btnVideoMode:
            self.btnVideoMode.setText("Камера" if mode == "camera" else "Радар")
        self.video.set_mode(mode)
        self.radar.set_mode(mode)

    # ---------------------------------------------------------------------
    # START/STOP (обычный режим маршрута)
    # ---------------------------------------------------------------------
    def _on_startstop_toggled(self, checked: bool):
        print(f"[UI] StartStop toggled -> {checked}", flush=True)
        self.video.reset_pole_snap_anchor(reason="user_start")
        # ---- запрет при ручном режиме ----
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

        # ---- режим траектории выбран на idle ----
        if getattr(self.state, "nav_mode", MODE_MARKERS) == MODE_TRAJ:
            self.state.is_running = bool(checked)
            self._refresh_startstop_caption()
            self._update_controls()

            if self.state.is_running:
                # === ТОЧНО КАК В ОСТАЛЬНЫХ РЕЖИМАХ ===
                try:
                    self._load_persisted_settings()
                    self._sync_tuning_from_state()
                    apply_start_defaults(self.state)

                    compute_controls_on_route(self.state, eps_px=2.0)
                    start_route_animation(self.ui, self.state)

                    if self._progress_timer and not self._progress_timer.isActive():
                        self._progress_timer.start()

                    # trajectory — ТОЛЬКО рулит PWM, не визуалкой
                    self.trajectory.set_enabled(True)

                except Exception as e:
                    print("[TRAJ][START] error:", e, flush=True)
            else:
                try:
                    self.trajectory.set_enabled(False)
                    stop_route_animation(self.state, keep_progress=True)
                except Exception:
                    pass

            try:
                update_drive_panel(self.ui, self.state)
            except Exception:
                pass

            return

        # ---- normal start/stop ----
        self.state.is_running = bool(checked)
        self._refresh_startstop_caption()
        self._update_controls()

        # HUD обновим
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        if self.state.is_running:
            # START
            try:
                self.dataset.on_start()
            except Exception:
                pass

            # 1) сначала загрузить сохранённые настройки
            try:
                self._load_persisted_settings()
            except Exception as e:
                print("[DRIVE] _load_persisted_settings error:", e, flush=True)

            # 2) применить на UI
            try:
                self._sync_tuning_from_state()
            except Exception as e:
                print("[DRIVE] _sync_tuning_from_state error:", e, flush=True)

            # 3) выставить стартовые PWM уже ПОСЛЕ загрузки (чтобы не переехать)
            try:
                apply_start_defaults(self.state)
            except Exception as e:
                print("[DRIVE] apply_start_defaults error:", e, flush=True)

            # ✅ restore B если до STOP было включено
            try:
                if bool(getattr(self.state, "_b_restore_valid", False)):
                    b_restore = int(getattr(self.state, "_b_restore_value", 1500) or 1500)
                    # L/R уже выставлены в apply_start_defaults -> берём текущие
                    l_now = int(getattr(self.state, "l_pwm", 1500) or 1500)
                    r_now = int(getattr(self.state, "r_pwm", 1500) or 1500)
                    motors_set(self.state, l_now, r_now, b_restore)
                    print(f"[DRIVE] START → restore B={b_restore}", flush=True)
                    self.state._b_restore_valid = False
            except Exception as e:
                print("[DRIVE] restore B error:", e, flush=True)
            # 4) подготовить контролы/анимацию
            try:
                compute_controls_on_route(self.state, eps_px=2.0)
            except Exception as e:
                print("[DRIVE] compute_controls_on_route error:", e, flush=True)

            try:
                start_route_animation(self.ui, self.state)
            except Exception as e:
                print("[DRIVE] start_route_animation error:", e, flush=True)

            if self._progress_timer and not self._progress_timer.isActive():
                self._progress_timer.start()

            try:
                update_drive_panel(self.ui, self.state)
            except Exception:
                pass

            return 

        # STOP
        try:
            self.dataset.on_stop()
        except Exception:
            pass

        try:
            if self._progress_timer and self._progress_timer.isActive():
                self._progress_timer.stop()
        except Exception:
            pass

        try:
            # ✅ если навесное было включено (B != 1500) — запомним и на STOP выключим
            b_cur = int(getattr(self.state, "b_pwm", 1500) or 1500)
            if b_cur != 1500:
                self.state._b_restore_valid = True
                self.state._b_restore_value = b_cur
                motors_set(self.state, 1500, 1500, 1500)
                print(f"[DRIVE] STOP → neutral L/R/B=1500 (saved B={b_cur})", flush=True)
            else:
                # было выключено — просто нейтраль
                self.state._b_restore_valid = False
                motors_set(self.state, 1500, 1500, 1500)
                print("[DRIVE] STOP → neutral L/R/B=1500", flush=True)

        except Exception as e:
            print("[DRIVE] motors_set neutral error:", e, flush=True)
        try:
            stop_route_animation(self.state, keep_progress=True)
        except Exception:
            pass

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass
    # ---------------------------------------------------------------------
    # Progress timer
    # ---------------------------------------------------------------------
    def _on_progress_tick(self):
        try:
            def _set_startstop_from_hw(checked: bool):
                want = bool(checked)
                if self.btnStartStop:
                    if bool(self.btnStartStop.isChecked()) == want:
                        return
                    self.btnStartStop.setChecked(want)
                else:
                    if bool(getattr(self.state, "is_running", False)) == want:
                        return
                    self._on_startstop_toggled(want)

            poll_arduino_startstop(_set_startstop_from_hw)
        except Exception:
            pass
        try:
            self.dataset.on_progress_tick()
        except Exception:
            pass

        try:
            check_flag_collision_and_update(self.state)
        except Exception:
            pass

        try:
            update_junction_turn_hint(self.state)
        except Exception:
            pass

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # DRIVE → IDLE
    # ---------------------------------------------------------------------
    def _on_back_clicked(self):
        try:
            if hasattr(self, "_progress_timer") and self._progress_timer.isActive():
                self._progress_timer.stop()
        except Exception:
            pass

        try:
            self.trajectory.set_enabled(False)
        except Exception:
            pass

        try:
            from graphics import clear_route_visual
            stop_route_animation(self.state, keep_progress=True)
            clear_route_visual(self.state, also_clear_data=False)
        except Exception:
            pass

        self.state.is_running = False
        self.state.safety_stop = False

        # ✅ глушим всё при уходе со страницы Drive
        try:
            motors_set(self.state, 1500, 1500, 1500)
            print("[DRIVE] BACK → neutral L=1500 R=1500 B=1500", flush=True)
        except Exception as e:
            print("[DRIVE] BACK motors_set neutral error:", e, flush=True)

        try:
            from status import set_indicator
            if self.indBattery:
                set_indicator(self.indBattery, "ok")
            if self.indLidar:
                set_indicator(self.indLidar, "ok")
        except Exception:
            pass

        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        pageIdle = self.ui.findChild(QtWidgets.QWidget, "pageIdle")
        if stack and pageIdle:
            stack.setCurrentWidget(pageIdle)

    # ---------------------------------------------------------------------
    # Map click
    # ---------------------------------------------------------------------
    def eventFilter(self, obj, ev):
        if self.mapViewDrive and obj is self.mapViewDrive.viewport():
            if ev.type() == QtCore.QEvent.MouseButtonPress and ev.button() == QtCore.Qt.LeftButton:
                sp = self.mapViewDrive.mapToScene(ev.pos())
                self._on_drive_map_click(float(sp.x()), float(sp.y()))
                return True
        return super().eventFilter(obj, ev)

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

        # РОВЕР: ручная локализация
        if getattr(self.state, "manual_loc_drive", False):
            set_robot_pose_px((x_px, y_px), self.state)

            # ✅ НЕ сбрасываем маршрут: это коррекция текущего маршрута.
            # Перенацеливаем прогресс по уже существующей полилинии маршрута
            # на ближайшую к новой позе точку.
            try:
                self._retarget_route_progress_to_robot_pose()
            except Exception:
                pass

            for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(self.state, sc)
                    except Exception:
                        pass

            # Маршрут как линия тот же, но можно обновить UI/панель
            try:
                update_drive_panel(self.ui, self.state)
            except Exception:
                pass
            return
        # КР: флажки B
        if getattr(self.state, "kr_mode_drive", False):
            st = self.state
            before_n = len(getattr(st, "control_pts_px", []) or [])
            set_control_item_px((x_px, y_px), st)
            after_n = len(getattr(st, "control_pts_px", []) or [])

            if after_n > before_n:
                is_on = bool(getattr(st, "kr_b_next_on", True))
                kind = "b_on" if is_on else "b_off"
                if not hasattr(st, "control_pts_kind") or st.control_pts_kind is None:
                    st.control_pts_kind = []
                while len(st.control_pts_kind) < (after_n - 1):
                    st.control_pts_kind.append(None)
                st.control_pts_kind.append(kind)
                st.kr_b_next_on = not is_on
                print(f"[KR] added flag #{after_n} kind={kind}", flush=True)

            for sc in (getattr(st, "_idle_scene", None), getattr(st, "_drive_scene", None)):
                if sc is not None:
                    try:
                        redraw_markers(st, sc)
                    except Exception:
                        pass
            if self.ui.statusBar():
                n = len(getattr(st, "control_pts_px", []) or [])
                self.ui.statusBar().showMessage(f"Добавлен флажок КР #{n}", 1200)
            return

        # Цель
        if not self.state.select_goal_mode_drive:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Включите режим «Ровер», «КР» или «Цель».")
            return

        set_goal_px((x_px, y_px), self.state)
        if not hasattr(self.state, "route_goal_px") or self.state.route_goal_px is None:
            self.state.route_goal_px = []
        self.state.route_goal_px.append(self.state.goal_px)

        if getattr(self.state, "robot_px", None):
            ok = build_route_from_robot_to_goal(self.state)
            if ok:
                try:
                    compute_controls_on_route(self.state, eps_px=2.0)
                except Exception:
                    pass
                redraw_route(self.state, self.ui)
                try:
                    update_drive_panel(self.ui, self.state)
                except Exception:
                    pass

    # ---------------------------------------------------------------------
    # Switchers (ЦЕЛЬ / КР / РОВЕР)
    # ---------------------------------------------------------------------
    def _on_select_goal_drive_toggled(self, enabled: bool):
        self.state.select_goal_mode_drive = bool(enabled)

        if enabled:
            # отжать КР и Ровер, но НЕ очищать их данные
            self._silent_uncheck(self.btnKR, "kr_mode_drive", self._refresh_kr_caption)
            self._silent_uncheck(self.btnRobotDrive, "manual_loc_drive", self._refresh_robot_caption)

            self._refresh_select_goal_caption()
            return

        # enabled=False -> это пользователь нажал "Отменить" у Цели => тут ЧИСТИМ как ты и хотел
        self.state.goal_px = None
        self.state.route_goal_px = []
        self.state.piece_goals_px = []

        try:
            from graphics import clear_route_visual
            clear_route_visual(self.state, also_clear_data=True)
        except Exception:
            pass

        self.state.route_progress_idx = 0
        self.state.route_seg_off_m = 0.0
        reset_route_progress(self.state)

        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(self.state, sc)
                except Exception:
                    pass

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        self._refresh_select_goal_caption()

    def _on_kr_toggled(self, enabled: bool):
        self.state.kr_mode_drive = bool(enabled)

        if enabled:
            # отжать Цель и Ровер, но НЕ очищать их данные
            self._silent_uncheck(self.btnSelectGoalDrive, "select_goal_mode_drive", self._refresh_select_goal_caption)
            self._silent_uncheck(self.btnRobotDrive, "manual_loc_drive", self._refresh_robot_caption)

            self._refresh_kr_caption()
            return

        # enabled=False -> пользователь нажал "Отменить" у КР => ЧИСТИМ КР
        self.state.control_pts_px = []
        self.state.control_pts_kind = []
        self.state.kr_b_next_on = True

        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(self.state, sc)
                except Exception:
                    pass

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        self._refresh_kr_caption()
    def _on_robot_mode_toggled(self, enabled: bool):
        self.state.manual_loc_drive = bool(enabled)

        if enabled:
            # отжать Цель и КР, но НЕ очищать их данные
            self._silent_uncheck(self.btnSelectGoalDrive, "select_goal_mode_drive", self._refresh_select_goal_caption)
            self._silent_uncheck(self.btnKR, "kr_mode_drive", self._refresh_kr_caption)

        self._refresh_robot_caption()

    def _refresh_startstop_caption(self):
        if self.btnStartStop:
            self.btnStartStop.setText("Стоп" if self.state.is_running else "Пуск")

    def _refresh_select_goal_caption(self):
        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setText("Отменить" if self.state.select_goal_mode_drive else "Цель")
    def _refresh_kr_caption(self):
        if self.btnKR:
            self.btnKR.setText("Отменить" if self.state.kr_mode_drive else "КР")

    def _refresh_robot_caption(self):
        if self.btnRobotDrive:
            self.btnRobotDrive.setText("Ровер" + (" (вкл.)" if self.state.manual_loc_drive else ""))

    def _update_controls(self):
        running = bool(getattr(self.state, "is_running", False))
        manual  = bool(getattr(self.state, "manual_drive", False))

        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not running)

        # режимные кнопки: во время движения/мануала - дизейблим и отжимаем ТОЛЬКО UI
        lock_modes = (running or manual)

        if self.btnSelectGoalDrive:
            self.btnSelectGoalDrive.setEnabled(not lock_modes)
            if lock_modes:
                self._silent_uncheck_ui_only(self.btnSelectGoalDrive)

        if self.btnKR:
            self.btnKR.setEnabled(not lock_modes)
            if lock_modes:
                self._silent_uncheck_ui_only(self.btnKR)

        if self.btnRobotDrive:
            self.btnRobotDrive.setEnabled(not lock_modes)
            if lock_modes:
                self._silent_uncheck_ui_only(self.btnRobotDrive)

        if self.btnStartStop:
            self.btnStartStop.setEnabled(not manual)

    # ---------------------------------------------------------------------
    # Manual mode (используется ManualController)
    # ---------------------------------------------------------------------
    def _enter_manual_mode(self):
        self.state.manual_drive = True

        if self.state.is_running and self.btnStartStop:
            self.btnStartStop.blockSignals(True)
            self.btnStartStop.setChecked(False)
            self.btnStartStop.blockSignals(False)
            self.state.is_running = False
            try:
                stop_route_animation(self.state, keep_progress=True)
            except Exception:
                pass

        self._update_controls()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _exit_manual_mode(self):
        self.state.manual_drive = False
        self._update_controls()
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def _on_manual_window_clicked(self):
        self.manual.open_window()

    # ---------------------------------------------------------------------
    # Speed
    # ---------------------------------------------------------------------
    def _change_speed(self, delta: int):
        base = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
        base = max(1500, min(2000, base + int(delta)))
        self.state.pwm_base_us = base
        print(f"[DRIVE] pwm_base_us -> {base}", flush=True)

        if getattr(self.state, "is_running", False) and not getattr(self.state, "trajectory_mode", False):
            try:
                motors_set(self.state, base, base, self.state.b_pwm)
            except Exception as e:
                print("[DRIVE] motors_set error on speed change:", e, flush=True)

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

        self._save_persisted_settings_debounced()

    # ---------------------------------------------------------------------
    # Zoom
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
    # Stack switch
    # ---------------------------------------------------------------------
    def _on_stack_changed(self, idx: int):
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        if not stack:
            return
        page = stack.widget(idx)

        if page is self.pageDrive:
            self._load_persisted_settings()   # на всякий — чтобы всегда актуально
            self._apply_mode_tuning_ui()
            self._sync_tuning_from_state()
            self._apply_tuning_lock_ui()

            self.radar.setup()
            self.video.ensure_started()

            try:
                from status import set_indicator
                if self.indBattery:
                    set_indicator(self.indBattery, "ok")
                if self.indLidar:
                    set_indicator(self.indLidar, "ok")
            except Exception:
                pass
        else:
            self.radar.stop()

    # ---------------------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------------------
    def shutdown(self):
        # сохранить настройки перед выходом
        try:
            self._save_persisted_settings()
        except Exception:
            pass

        try:
            self.trajectory.set_enabled(False)
        except Exception:
            pass

        try:
            self.radar.stop()
        except Exception:
            pass

        try:
            self.video.shutdown()
        except Exception:
            pass
        try:
            motors_set(self.state, 1500, 1500, 1500)
            print("[DRIVE] shutdown → neutral L=1500 R=1500 B=1500", flush=True)
        except Exception as e:
            print("[DRIVE] shutdown motors_set neutral error:", e, flush=True)