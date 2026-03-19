# state.py
# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional
from PyQt5 import QtWidgets

class AppState:
    def __init__(self):
        # контрольные точки
        self.control_pts_px: list[tuple[float, float]] = []

        # UI items для флагов
        self.idle_control_items: list[QtWidgets.QGraphicsItem] = []
        self.drive_control_items: list[QtWidgets.QGraphicsItem] = []
        
        # Карта
        self.active_map_path = None
        self.graph = None
        self.points_px = None
        self.meters_per_pixel = None

        self.robot_px = None
        self.goal_px  = None
        self.route_pts_px = []
        self.route_len_m  = 0.0
        self.route_done_m = 0.0
        # Сплайн карты (в пикселях)
        self.spline_polyline: Optional[List[Tuple[float, float]]] = None

        # Индексы на сплайне (логическое состояние — переносимо между экранами)
        self.robot_idx: Optional[int] = None   # белый флаг (позиция робота)
        self.goal_idx: Optional[int]  = None   # красный флаг (контрольная точка)

        # Режимы
        self.manual_loc_mode: bool = False       # IDLE: режим «поставить флаг»
        self.select_goal_mode_idle: bool = False # IDLE: режим «выбора цели»
        self.select_goal_mode_drive: bool = False# DRIVE: режим «выбора цели»
        self.is_running: bool = False            # DRIVE: пуск/стоп

        # UI-элементы сцены, чтобы убирать/перерисовывать
        # Idle
        self.idle_map_pixmap_item: QtWidgets.QGraphicsPixmapItem = None
        self.idle_robot_item: QtWidgets.QGraphicsItem = None
        self.idle_goal_item: QtWidgets.QGraphicsItem = None
        self.idle_route_item: QtWidgets.QGraphicsItem = None

        # Drive
        self.drive_map_pixmap_item: QtWidgets.QGraphicsPixmapItem = None
        self.drive_robot_item: QtWidgets.QGraphicsItem = None
        self.drive_goal_item: QtWidgets.QGraphicsItem = None
        self.drive_route_item: QtWidgets.QGraphicsItem = None