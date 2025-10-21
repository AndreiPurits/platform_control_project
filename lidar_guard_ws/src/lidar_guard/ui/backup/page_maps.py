# page_maps.py
# -*- coding: utf-8 -*-
import os
from PyQt5 import QtWidgets
from state import AppState
from graphics import show_map_on_views
from routing import load_graph_and_points_for

class MapsPage:
    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState, maps_dir: str = None):
        self.ui = ui
        self.state = state
        self.maps_dir = maps_dir or os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui")
        os.makedirs(self.maps_dir, exist_ok=True)

        self.pageMaps: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageMaps")
        self.btnMapsBack: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapsBack")
        self.btnChooseFromFolder: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnChooseFromFolder")
        self.lblMapsHint: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblMapsHint")

        if self.lblMapsHint:
            self.lblMapsHint.setText(f"Папка: {self.maps_dir}\nНажмите «Выбрать карту…» и укажите PNG.")

        if self.btnMapsBack:
            self.btnMapsBack.clicked.connect(self.ui.to_idle)
        if self.btnChooseFromFolder:
            self.btnChooseFromFolder.clicked.connect(self._choose_png)

        # ссылки на виды
        self.idle_view  = ui.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        self.drive_view = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")

    def _choose_png(self):
        start_dir = self.maps_dir if os.path.isdir(self.maps_dir) else os.path.expanduser(
            "/home/andrei/lidar_guard_ws/src/lidar_guard/ui"
        )
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.ui, "Выбрать карту (PNG)", start_dir, "PNG (*.png);;Все файлы (*)"
        )
        if not fname:
            return

        self.state.active_map_path = fname

        # показать карту в обоих видах
        show_map_on_views(fname, self.idle_view, self.drive_view, self.state)

        # загрузить граф/points
        load_graph_and_points_for(fname, self.state)

        # и вернуть пользователя на IDLE (где кнопки «Локализация/Цель»)
        if hasattr(self.ui, "to_idle"):
            self.ui.to_idle()