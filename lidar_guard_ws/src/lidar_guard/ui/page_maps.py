import os
from PyQt5 import QtWidgets, QtGui, QtCore

from state import AppState
from graphics import show_map_on_views, attach_click_router, update_hud_text
from routing import load_graph_for

# -*- coding: utf-8 -*-
"""
== API INDEX ================================================================
CLASS: MapsPage(ui, state, maps_dir=None)
- _choose_png() -> None
    Диалог выбора PNG; грузит карту, пытается загрузить рядом *_graph.json,
    показывает карту в обоих видах и вешает обработчик «два клика → маршрут».
- _after_route() -> None
    Хук после успешного построения маршрута (зарезервировано под расширения).

ПОВЕДЕНИЕ:
- При выборе PNG:
    * show_map_on_views(...) кладёт картинку и HUD в обе сцены (idle/drive).
    * load_graph_for(...) ищет и грузит базовый граф рядом (base + "_graph.json"),
      пишет его в state.graph и связанные метаданные.
    * attach_click_router(...) на каждый view ставит обработчик, который принимает два клика
      (старт, цель), строит маршрут через routing.build_route_snap_pixels и вызывает graphics.redraw_route.
- HUD и status bar сообщают: загружен ли граф, построен ли маршрут и его длину.
============================================================================
"""

import os
from PyQt5 import QtWidgets

# Графика: поднимаем картинку, HUD и обработчики кликов
from graphics import show_map_on_views, attach_click_router, update_hud_text
# Роутинг: загрузка графа (из *_graph.json)
from routing import load_graph_for


class MapsPage:
    """
    Страница MAPS:
      - выбор PNG из папки,
      - автозагрузка *_graph.json (если есть),
      - показ карты в обоих видах,
      - «два клика → маршрут» через attach_click_router (по графу).
    """

    def __init__(self, ui: QtWidgets.QMainWindow, state: AppState, maps_dir: str = None):
        self.ui = ui
        self.state = state

        # Где искать карты (по умолчанию — твой каталог)
        self.maps_dir = maps_dir or os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui")
        os.makedirs(self.maps_dir, exist_ok=True)

        # Виджеты из .ui
        self.pageMaps: QtWidgets.QWidget = ui.findChild(QtWidgets.QWidget, "pageMaps")
        self.btnMapsBack: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnMapsBack")
        self.btnChooseFromFolder: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnChooseFromFolder")
        self.lblMapsHint: QtWidgets.QLabel = ui.findChild(QtWidgets.QLabel, "lblMapsHint")

        # Два вида карты
        self.idle_view: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        self.drive_view: QtWidgets.QGraphicsView = ui.findChild(QtWidgets.QGraphicsView, "mapViewDrive")

        # Подсказка
        if self.lblMapsHint:
            self.lblMapsHint.setText(f"Папка: {self.maps_dir}\nНажмите «Выбрать карту…» и укажите PNG.")

        # Кнопки
        if self.btnMapsBack:
            self.btnMapsBack.clicked.connect(self.ui.to_idle)
        if self.btnChooseFromFolder:
            self.btnChooseFromFolder.clicked.connect(self._choose_png)

        # держатели фильтров (чтобы GC не забрал обработчики)
        self._idle_click_router = None
        self._drive_click_router = None

    # ------------------------------------------------------------------
    # Выбор PNG, подгрузка графа, показ в двух видах + навешивание кликов
    # ------------------------------------------------------------------
    def _choose_png(self):
        start_dir = self.maps_dir if os.path.isdir(self.maps_dir) else os.path.expanduser(
            "/home/andrei/lidar_guard_ws/src/lidar_guard/ui"
        )
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.ui,
            "Выбрать карту (PNG)",
            start_dir,
            "PNG (*.png);;Все файлы (*)"
        )
        if not fname:
            return

        # запоминаем активную карту
        self.state.active_map_path = fname

        # пробуем загрузить рядом граф base+"_graph.json"
        load_graph_for(fname, self.state)   # безопасно внутри: сам проверит наличие

        # показываем карту в обоих видах (и HUD)
        show_map_on_views(fname, self.idle_view, self.drive_view, self.state)

        # HUD: сброс текста
        for sc in (getattr(self.state, "_idle_scene", None), getattr(self.state, "_drive_scene", None)):
            if sc is not None:
                try:
                    update_hud_text(sc, "Маршрут не задан")
                except Exception:
                    pass

        # «два клика → маршрут» — на оба вида
        self._idle_click_router = attach_click_router(self.idle_view, self.state, self.ui, on_after_route=self._after_route)
        self._drive_click_router = attach_click_router(self.drive_view, self.state, self.ui, on_after_route=self._after_route)

        # статус-бар: отчёт
        base = os.path.basename(fname)
        has_graph = getattr(self.state, "graph", None) is not None
        msg = f"Карта выбрана: {base}" + (" (граф загружен)" if has_graph else " (граф не найден)")
        try:
            self.ui.statusBar().showMessage(msg, 4000)
        except Exception:
            pass

        # назад в IDLE
        if hasattr(self.ui, "to_idle"):
            self.ui.to_idle()

    # ------------------------------------------------------------------
    # Хук после построения маршрута (сейчас опционально)
    # ------------------------------------------------------------------
    def _after_route(self):
        """
        Вызывается attach_click_router-ом после успешной маршрутизации.
        Здесь можно дописать логику (лог/телеметрия/сохранение последнего пути).
        Пока ничего не делаем — есть HUD/статус-бар из graphics.redraw_route().
        """
        pass