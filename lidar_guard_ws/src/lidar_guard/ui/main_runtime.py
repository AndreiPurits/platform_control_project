# -*- coding: utf-8 -*-  # main_runtime.py
import os
import signal
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic

from state import AppState
from page_idle import IdlePage
from page_drive import DrivePage
from routing import refresh_all_overlays

import logging
logging.disable(logging.NOTSET)


UI_PATH = os.path.join(os.path.dirname(__file__), "main_window.ui")


class Main(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        if not os.path.isfile(UI_PATH):
            raise FileNotFoundError(f"UI not found: {UI_PATH}")

        uic.loadUi(UI_PATH, self)
        self.setWindowTitle("Platform GUI")

        # глобальное состояние
        self.state = AppState()

        # базовые ссылки на ключевые виджеты/страницы
        self.stack: QtWidgets.QStackedWidget = self.findChild(QtWidgets.QStackedWidget, "stackRoot")
        self.pageIdle: QtWidgets.QWidget = self.findChild(QtWidgets.QWidget, "pageIdle")
        self.pageDrive: QtWidgets.QWidget = self.findChild(QtWidgets.QWidget, "pageDrive")

        if not self.stack or not self.pageIdle or not self.pageDrive:
            raise RuntimeError("Страницы stackRoot / pageIdle / pageDrive не найдены в UI.")

        # инициализируем контроллеры страниц
        self.idle = IdlePage(self, self.state)
        self.drive = DrivePage(self, self.state)

        # элементы DRIVE top bar, которые обрабатываются на уровне Main
        self.btnStartStop: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnEStop: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnEStop")

        # старт/стоп
        if self.btnStartStop:
            self.btnStartStop.setCheckable(True)
            self.btnStartStop.setChecked(False)
            self.btnStartStop.setText("Пуск")
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)

        # выбор карты с DRIVE возможен только при стопе
        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_drive_pick_map)

        # Кнопка "Выбор карты" на IDLE вызывает тот же пикер
        btn_choose_idle = self.findChild(QtWidgets.QPushButton, "btnChooseMap")
        if btn_choose_idle:
            btn_choose_idle.clicked.connect(self.pick_map_and_load)

        # E-STOP (заглушка)
        if self.btnEStop:
            self.btnEStop.clicked.connect(lambda: QtWidgets.QMessageBox.information(
                self, "E-STOP", "Аварийный стоп (заглушка)")
            )

        # стартуем в IDLE
        self.to_idle()
        if self.statusBar():
            self.statusBar().showMessage("Готово", 2000)

    # ----------- навигация между экранами -----------
    def to_idle(self):
        self.stack.setCurrentWidget(self.pageIdle)
        # перерисуем оверлеи (если уже выбраны флаг/цель)
        refresh_all_overlays(
            self.state,
            self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
            self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"),
        )

    def to_drive(self):
        self.stack.setCurrentWidget(self.pageDrive)
        # перерисуем оверлеи поверх карты DRIVE
        refresh_all_overlays(
            self.state,
            self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
            self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"),
        )

    # ----------- верхний бар DRIVE -----------
    def _on_startstop_toggled(self, checked: bool):
        self.state.is_running = bool(checked)
        if self.btnStartStop:
            self.btnStartStop.setText("Стоп" if checked else "Пуск")

        # выбор карты разрешён только при стопе
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not self.state.is_running)

        # подсказка в статус-баре
        if self.statusBar():
            self.statusBar().showMessage("START" if checked else "STOP", 1500)

    def _on_drive_pick_map(self):
        if self.state.is_running:
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Сначала Стоп → затем карта.")
            return
        self.pick_map_and_load()
    # --- НОВОЕ: утилиты диагностики и безопасного сброса ---


    def _hard_stop_animation(self):
        """Полный стоп анимации: остановить таймер и обнулить ссылку, чтобы не тикал по удалённым QGraphicsItem."""
        s = self.state
        try:
            t = getattr(s, "_route_timer", None)
            if t and t.isActive():
                t.stop()
        except Exception:
            pass
        s._route_timer = None  # ВАЖНО: обнуляем, чтобы нигде не тыркнуло

    def _reset_all_route_and_markers(self):
        """Чистим путь, цель, прогресс и перерисовываем сцены; флаги НЕ трогаем (они контролируются отдельно)."""
        from graphics import clear_route_visual, redraw_markers
        s = self.state
        # убрать красный маршрут и данные
        try:
            clear_route_visual(s, also_clear_data=True)
        except Exception:
            pass
        # цель
        s.goal_px = None
        # прогресс
        s.route_progress_idx = 0
        s.route_seg_off_m = 0.0
        s.route_done_m = 0.0
        s.route_finished = False
        # перерисовать сцены
        for sc in (getattr(s, "_idle_scene", None), getattr(s, "_drive_scene", None)):
            if sc is not None:
                try:
                    redraw_markers(s, sc)
                except Exception:
                    pass

    def _reset_everything_on_new_map(self):
        """Полный консистентный сброс перед показом новой карты."""
        self._hard_stop_animation()
        # Сбрасываем ВСЁ, что могло накопиться, включая флаги (ты просил «обнулиться вообще всё» при выборе карты)
        s = self.state
        s.robot_px = None
        s.goal_px = None
        s.route_pts_px = []
        s.route_pts_m  = []
        s.route_len_m  = 0.0
        s.route_done_m = 0.0
        s.route_progress_idx = 0
        s.route_seg_off_m = 0.0
        s.route_finished = False
        s.control_pts_px = []   # ← флаги тоже обнуляем при выборе НОВОЙ карты
    # жёстко убить маршрутный таймер перед загрузкой новой карты
        try:
            if hasattr(s, "_route_timer") and s._route_timer:
                if s._route_timer.isActive():
                    s._route_timer.stop()
                s._route_timer.deleteLater()
        except Exception:
            pass
        s._route_timer = None
        # Сцены очистятся внутри show_map_on_views, но перерисуем безопасно пустыми данными
        for name in ("idle_route_item","drive_route_item","idle_robot_item","drive_robot_item","idle_goal_item","drive_goal_item"):
            if hasattr(s, name):
                setattr(s, name, None)
        
    def pick_map_and_load(self):
        # Полный сброс перед выбором карты
        self._reset_everything_on_new_map()

        # НЕНАТИВНЫЙ диалог (устойчивее в среде ROS/Wayland)
        start_dir = os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/maps_repo")
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog  # NEW: избегаем крашей
        try:
            fname, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Выбрать карту (PNG)", start_dir,
                "PNG (*.png);;Все файлы (*)", options=options
            )
        except Exception as e:
            print("[MAP] QFileDialog error:", e, flush=True)
            return

        if not fname:
            print("[MAP] отмена выбора", flush=True)
            return

        self.state.active_map_path = fname

        # показать карту в обеих сценах
        from graphics import show_map_on_views
        idle_view  = self.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        drive_view = self.findChild(QtWidgets.QGraphicsView, "mapViewDrive")
        try:
            show_map_on_views(fname, idle_view, drive_view, self.state)
        except Exception as e:
            print("[MAP] show_map_on_views error:", e, flush=True)

        # загрузить граф/points
        from routing import load_graph_and_points_for, refresh_all_overlays
        ok = False
        try:
            ok = load_graph_and_points_for(fname, self.state)
        except Exception as e:
            print("[MAP] load_graph_and_points_for error:", e, flush=True)

        # перерисовать оверлеи
        try:
            refresh_all_overlays(self.state, idle_view, drive_view)
        except Exception as e:
            print("[MAP] refresh_all_overlays error:", e, flush=True)

        # кнопка «Пуск» → гарантированно в «Пуск»
        try:
            self.state.is_running = False
            btn = self.findChild(QtWidgets.QPushButton, "btnStartStop")
            if btn:
                btn.setChecked(False)
                btn.setText("Пуск")
        except Exception:
            pass

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    # небольшая косметика отрисовки на HiDPI
    app.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app.setApplicationName("Platform GUI")

    w = Main()
    w.resize(1280, 800)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()