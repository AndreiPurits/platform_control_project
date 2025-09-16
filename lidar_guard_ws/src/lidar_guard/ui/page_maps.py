# -*- coding: utf-8 -*-
import os
from PyQt5 import QtWidgets, QtCore, QtGui
from graphics import show_map_on_views
from routing import load_map_spline_for
from state import AppState

class MapsPage:
    """Экран MAPS: минималистичный вариант — кнопка 'Выбрать карту…' и подсказка."""
    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState, maps_dir: str = None):
        self.ui = ui
        self.state = state

        # Папка с картами — можно передать снаружи, иначе дефолт: ~/Downloads/maps_repo
        self.maps_dir = maps_dir or os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui")
        os.makedirs(self.maps_dir, exist_ok=True)

        # Виджеты из .ui
        self.pageMaps: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageMaps")
        self.btnMapsBack: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapsBack")
        self.btnChooseFromFolder: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnChooseFromFolder")
        self.lblMapsHint: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblMapsHint")

        # Подсказка с путём
        if self.lblMapsHint:
            self.lblMapsHint.setText(f"Папка: {self.maps_dir}\nНажмите «Выбрать карту…» и укажите PNG.")

        # Кнопки
        if self.btnMapsBack:
            self.btnMapsBack.clicked.connect(self.ui.to_idle)
        if self.btnChooseFromFolder:
            self.btnChooseFromFolder.clicked.connect(self._choose_png)

    def _choose_png(self):
      
        start_dir = self.maps_dir if os.path.isdir(self.maps_dir) else os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui")
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.ui,
            "Выбрать карту (PNG)",
            start_dir,
            "PNG (*.png);;Все файлы (*)"
        )
        if not fname:
            return

        # Запомнить и подгрузить сплайн (если есть .json)
        self.state.active_map_path = fname
        load_map_spline_for(fname, self.state)  # <-- важно: передаём state

        # Показать карту в Idle/Drive
        idle_view = self.ui.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        drive_view = self.ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        show_map_on_views(fname, idle_view, drive_view, self.state)

        # Статус-бар
        if hasattr(self.ui, "statusBar"):
            base = os.path.basename(fname)
            has_json = os.path.isfile(os.path.splitext(fname)[0] + ".json")
            msg = f"Карта выбрана: {base}" + (" (сплайн загружен)" if has_json else " (.json не найден)")
            try:
                self.ui.statusBar().showMessage(msg, 4000)
            except Exception:
                pass

        # Назад в IDLE
        self.ui.to_idle()
