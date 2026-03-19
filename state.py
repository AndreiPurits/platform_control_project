# state.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets
import os
class AppState:
    def __init__(self):
        # Карта
        self.active_map_path = None
        self.graph = None
        self.points_px = None
        self.meters_per_pixel = None
        self.m_per_px_x = 0.8342405111938622
        self.m_per_px_y = 1.2604320351524956
        self.robot_px = None
        self.goal_px  = None
        self.route_goal_px = []
        self.route_pts_px = []
        self.route_pts_m = []
        self._junc_s_m = None
        self.route_len_m  = 0.0
        self.route_len_px = 0.0
        self.route_done_m = 0.0
        self.junctions_px = []
        self.drive_pwm_us = 1700
        self.jdist = None
        self.next_turn_dir = None
        self.route_contr_m = 0.0

        self.route_cum_px = []

        # --- РЕЖИМ ТРАЕКТОРИИ ---
        self.trajectory_mode: bool = False   # едем по вручную заданной траектории
        self.traj_idx: int = 0               # текущий сегмент (route_pts_px[traj_idx] -> route_pts_px[traj_idx+1])
        self.traj_seg_heading_rad: float = 0.0  # опорный угол текущего сегмента
        self.traj_wp_radius_m: float = 0.2     # радиус "достижения" точки, м

        self.speed_mps = 0.0
        self.spline_polyline: Optional[List[Tuple[float, float]]] = None

        self.robot_idx: Optional[int] = None   # белый флаг (позиция робота)
        self.goal_idx: Optional[int]  = None   # красный флаг (контрольная точка)

        self.manual_loc_mode: bool = False       # IDLE: режим «поставить флаг»
        self.select_goal_mode_idle: bool = False # IDLE: режим «выбора цели»
        self.select_goal_mode_drive: bool = False# DRIVE: режим «выбора цели»
        self.is_running: bool = False            # DRIVE: пуск/стоп
        self.loc_mode_idle: bool = False 
        self.loc_mode_drive: bool = False 

        self.b_pwm = 1500          # текущее значение 3-го выхода
        self.b_on_pwm = 2000       # "включить" (например, 2000 мкс)
        self.b_off_pwm = 1500      # "выключить" (нейтраль)
        self.kr_b_next_on = True   # следующий флажок КР будет ON, потом OFF и т.д.

        # КР: точки + тип (для цвета и логики B)
        self.control_pts_px: list[tuple[float, float]] = []
        self.control_pts_kind: list[str] = []   # "b_on" / "b_off"

        # LIDAR

        self.lidar_snap_enabled = False
        self.lidar_px_hint = None
        self.lidar_snap_step_px = 3.0
        self._lidar_last_pts = []         # последние принятые точки лидара ([(x,y),...])


        # UI-элементы сцены, чтобы убирать/перерисовывать
        # Idle
        self.idle_map_pixmap_item: QtWidgets.QGraphicsPixmapItem = None
        self.idle_robot_item: QtWidgets.QGraphicsItem = None
        self.idle_goal_item: QtWidgets.QGraphicsItem = None
        self.idle_route_item: QtWidgets.QGraphicsItem = None
        self.idle_control_items: list = []

        # Drive
        self.drive_map_pixmap_item: QtWidgets.QGraphicsPixmapItem = None
        self.drive_robot_item: QtWidgets.QGraphicsItem = None
        self.drive_goal_item: QtWidgets.QGraphicsItem = None
        self.drive_route_item: QtWidgets.QGraphicsItem = None
        self.drive_control_items: list = []

        self.dataset_mode: bool = False                 # включили кнопку «Датасет» на Idle
        self.dataset_root: str = os.path.expanduser("~/datasets")  # базовая директория
        self.map_name: str | None = None                # имя текущей карты (без .png)

        self.capture_step_m: float = 3.0                # каждые 3 метра
        self._last_capture_m: float = 0.0               # от какого пройденного метра отсчитываем
        self._dataset_active: bool = False              # идёт ли сейчас запись (пуск в датасете)

        # ---- Лидары ----
        # сырые точки (могут приходить по UDP из разных портов)
        self._lidar_front_last_pts = []
        self._lidar_rear_last_pts  = []

        # глобальный флаг STOP (как и раньше, “есть ли стоп вообще”)
        self.safety_stop       = False
        self.safety_stop_front = False
        self.safety_stop_rear  = False

        # ПЕРЕДНИЙ лидар (смотрит вперёд)
        self.lidar_front_sector_half_deg   = 15.0
        self.lidar_front_stop_distance_m   = 2.0
        self.lidar_front_ignore_radius_m   = 0.7
        self.lidar_front_stop_min_points   = 3
        self.lidar_front_mount_yaw_rad     = 3.14

        # ЗАДНИЙ лидар (смотрит назад)
        self.lidar_rear_sector_half_deg    = 25.0
        self.lidar_rear_stop_distance_m    = 1
        self.lidar_rear_ignore_radius_m    = 0.01
        self.lidar_rear_stop_min_points    = 3
        self.lidar_rear_mount_yaw_rad      = 0.0  

        self.pwm_base_us = 1750
        self.har_rear_lidar = False

        # «зелёный трек» — постоянный
        self.visited_path_px: list[tuple[float,float]] = []
        self.drive_visited_item: QtWidgets.QGraphicsPathItem | None = None
        self.capture_interval_m = 3.0
        self.camera_available = True   
        self.lidar_available  = True
        self._lidar_last_pts  = []          # сюда кладём последние точки
        self._lidar_last_ts   = 0

        # --- Camera control (открываем/закрываем в DrivePage) ---
        self.cam_enabled: bool = True     # можно быстро вырубить камеру логически
        self.cam_index: int = 0           # индекс видеоустройства (0 по умолчанию)
        self._cv2_cap = None              # handle cv2.VideoCapture

        # --- Dataset capture (унифицировано) ---
        self.dataset_step_m: float = 3.0          # шаг сохранения, м
        self.dataset_last_snap_m: float = 0.0     # на каком пройденном метре последний снимок

        # кэш папок под текущую карту (чтобы не вычислять каждый раз)
        self.dataset_photos_dir: Optional[str] = None
        self.dataset_lidar_dir: Optional[str] = None

        # камера
        self.cam_w_markers = 3840
        self.cam_h_markers = 2160
        self.cam_fps_markers = 30

        self.cam_w_road = 1280
        self.cam_h_road = 720
        self.cam_fps_road = 30
        self._cam = None                  # cv2.VideoCapture
        self._cam_timer = None            # QtCore.QTimer
        self._last_cam_frame = None       # numpy ndarray BGR
        
        # ограничения/калибровка
        self.max_speed_mps = 5       # физический максимум
        self.min_pwm = 60                 # чтобы моторы реально тронулись
        self.pwm_per_mps = 40.0           # калибровка: м/с -> ШИМ
        self.steer_kp = 1.5               # усиление по ошибке курса
        self.steer_deadband_rad = 0.05    # мёртвая зона ~3°
        self.turn_hard_err_rad = 0.7      # >40° — «жёсткий поворот» одной гусеницей
        self.turn_only_pwm = 140          # ШИМ при «жёстком повороте»
        self.turn_around_requested = False  # флаг «развернуться»
        
        self.l_pwm = 0
        self.r_pwm = 0
        self.yawrate_radps = 0.0
        
        self.road_frac = 0.0
        self.stripe_frac = 0.0
        self.road_frac_ema = None
        self.stripe_frac_ema = None
        self.road_state = "unknown"   # "no_road" | "far_road" | "on_road" | "unknown"
        self.road_on = False          # True, если мы точно «на дороге»
        
        self.manual_l_pwm = 0
        self.manual_r_pwm = 0
        self.manual_b_pwm = 0

        self.lane_center_px = None
        self.lane_offset_px = None
        self.lane_offset_ema_px = None
        self.lane_left_px = None
        self.lane_right_px = None
        self.lane_last_ok = None
        self.lane_junction_candidate = False

        self.robot_yaw_rad = 0
        self.traj_current_yaw = 0 

        self.fturn = ""
        self.fturn_l = 0.0
        self.fturn_c = 0.0
        self.fturn_r = 0.0
        self._last_snap_jxy = None
        
        self.turn_deg = 0
        self._active_turn_s_m = 0
        self._active_turn_j_id = 0

        self.main_road_fill_bgr = (0, 70, 0)   # темнее
        self.rail_mask_update_s = 2.5          # обновление якорей раз в 2.5 сек
        self.rail_mask_tau = 1.2               # сглаживание якорей по кадрам
        self.rail_mask_hist_n = 5              # медиана по 5 обновлениям
        self.rail_mask_bottom_saturated_thr = 0.98

        self.rail_inset_px = 35
        self.rail_head_soft = 0.25
        self.rail_curv_soft = 0.20
        self.rail_k_head_px = 0.012
        self.rail_k_curv = 0.00030

        self.mainline_bias_m = 6.0
        self.mainline_exec_m = 2.0
        self.mainline_bias_w = 0.25
        self.mainline_follow_w = 1.0
        self.mainline_post_turn_off_s = 1.0

        self.cam_device = "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0"    
        # или by-path (ещё стабильнее к порту):
        # "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0"        
        self.cam_open_timeout_s = 2.0
        self.cam_reopen_cooldown_s = 0.4
        # Auto-brightness target used in drive loop (same as camera_debug).
        self.cam_target_brightness = 110.0
        self.cam_target_tolerance = 15.0
        self.cam_roi_mean = 0.0
        self.cam_roi_median = 0.0
        self.force_mainline_only = True
        self.mainline_follow_w = 1.0
        self.mainline_bias_w = 0.15   # 0.15..0.25 ок

        self.turn_execute = False
        self.turn_allowed = False
        self.turn_bias_allowed = False

        self.turn_cam_L = False
        self.turn_cam_R = False
        self.turn_cam_C = False

         # --- Marker-based driving (AruCo) ---
        self.marker_mode: bool = True          # главный контур управления по знакам
        self.marker_len_m: float = 0.173   

        # last detection
        self.marker_seen: bool = False
        self.marker_id: int | None = None
        self.marker_dist_m: float | None = None
        self.marker_cx: float | None = None
        self.marker_side: str | None = None   # "L" | "R"
        self.marker_ts: float = 0.0

        # turn state machine
        self.marker_turn_active: bool = False
        self.marker_turn_dir: str | None = None  # "left" | "right"
        self.marker_turn_start_ts: float = 0.0
        self.marker_turn_timeout_s: float = 1.6   # грубо, потом откалибруем
        self.marker_drive_enabled = True

        self.marker_target_side_m = 2.0     # держим 2 м от столба
        self.marker_recent_s = 0.7          # сколько времени считаем "маркер ещё свежий"

        self.marker_kp = 0.35                # усиление руления по боковой ошибке
        self.marker_max_steer = 0.8

        # runtime:
        self.marker_last_ts = 0.0
        self.marker_active = False
        # --- ARUCO ids (final) ---
        self.aruco_id_straight_right_pole = 10  # прямо по правому столбу (столб справа)
        self.aruco_id_straight_left_pole  = 15  # прямо по левому столбу (столб слева)
        self.aruco_id_turn_right          = 20  # поворот направо
        self.aruco_id_turn_left           = 30  # поворот налево

        # marker follow side: "right"|"left" (какой столб держим)
        self.marker_follow_side: str = "right"
        self.marker_tvec = None

        self.marker_turn_trigger_m = 5.0
        self.marker_steer_slew = 2.8
        self.marker_approach_max_steer = 0.45
        self.marker_approach_kp = 0.60
        self.marker_approach_dead_m = 0.08
        self.marker_approach_alpha = 0.25
        
        self.pwm_trim_lr = 2

        self.rails_enabled: bool = False
        self.road_model_path: str = ""

        self.pole_spacing_m: int = 25 
        self.manual_drive: bool = False
        self.marker_pass_m: float = 1.2
        # minimal time between two pole pass events (seconds)
        self.marker_pass_cooldown_s: float = 0.8

        self.aruco_center_crop_enable = False