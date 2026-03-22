# -*- coding: utf-8 -*-  # main_runtime.py
import os
import signal
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic

from state import AppState
from page_idle import IdlePage
from page_drive import DrivePage
from routing import refresh_all_overlays
from graphics import show_map_on_views
import logging
logging.disable(logging.NOTSET)

import faulthandler, sys
faulthandler.enable(all_threads=True)
sys.stderr.write("[FH] faulthandler enabled\n")
sys.stderr.flush()
UI_PATH = os.path.join(os.path.dirname(__file__), "main_window.ui")


class Main(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        if not os.path.isfile(UI_PATH):
            raise FileNotFoundError(f"UI not found: {UI_PATH}")

        uic.loadUi(UI_PATH, self)
        self.setWindowTitle("Platform GUI")
        # На всякий случай снимаем жёсткий дизайнерский размер (если .ui когда-то снова зафиксируют)
        self.setMinimumSize(320, 240)
        self.setMaximumSize(16777215, 16777215)
        # Выход: Ctrl+Q, Ctrl+W
        QtWidgets.QShortcut(QtGui.QKeySequence.Quit, self, activated=self.close)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+W"), self, activated=self.close)
        self.state = AppState()

        # базовые ссылки на ключевые виджеты/страницы
        self.stack: QtWidgets.QStackedWidget = self.findChild(QtWidgets.QStackedWidget, "stackRoot")
        self.pageIdle: QtWidgets.QWidget = self.findChild(QtWidgets.QWidget, "pageIdle")
        self.pageDrive: QtWidgets.QWidget = self.findChild(QtWidgets.QWidget, "pageDrive")

        if not self.stack or not self.pageIdle or not self.pageDrive:
            raise RuntimeError("Страницы stackRoot / pageIdle / pageDrive не найдены в UI.")

        # --- ВАЖНО: вложить реальные UI страниц в контейнеры main_window.ui ---
        # Здесь предполагается, что у тебя есть page_idle.ui и page_drive.ui рядом с main_window.ui
        BASE_DIR = os.path.dirname(__file__)
        IDLE_UI = os.path.join(BASE_DIR, "page_idle.ui")
        DRIVE_UI = os.path.join(BASE_DIR, "page_drive.ui")

        idle_layout: QtWidgets.QVBoxLayout = self.findChild(QtWidgets.QVBoxLayout, "pageIdleLayout")
        drive_layout: QtWidgets.QVBoxLayout = self.findChild(QtWidgets.QVBoxLayout, "pageDriveLayout")
        if idle_layout is None or drive_layout is None:
            raise RuntimeError("pageIdleLayout / pageDriveLayout не найдены в main_window.ui")

        # очистить контейнеры (на всякий случай)
        while idle_layout.count():
            it = idle_layout.takeAt(0)
            w = it.widget()
            if w:
                w.setParent(None)

        while drive_layout.count():
            it = drive_layout.takeAt(0)
            w = it.widget()
            if w:
                w.setParent(None)

        # загрузить виджеты страниц и вставить
        self.pageIdleWidget = uic.loadUi(IDLE_UI)
        self.pageDriveWidget = uic.loadUi(DRIVE_UI)
        idle_layout.addWidget(self.pageIdleWidget)
        drive_layout.addWidget(self.pageDriveWidget)
        # инициализируем контроллеры страниц
        self.idle = IdlePage(self, self.state)
        self.drive = DrivePage(self, self.state)

        # ✅ автозагрузка последней карты
        s = QtCore.QSettings("Rover", "PlatformGUI")
        last = s.value("maps/last_path", "")
        if last and os.path.isfile(last):
            try:
                self._load_map_file(last)   # helper, который ты уже добавлял/добавишь ниже
            except Exception as e:
                print("[MAIN] autoload last map failed:", e, flush=True)

        # стартуем в IDLE
        self.to_idle()
        if self.statusBar():
            self.statusBar().showMessage("Готово", 2000)
            
    def closeEvent(self, ev):  
        try:
            if hasattr(self, "drive") and self.drive:
                self.drive.shutdown()
        except Exception as e:
            print("[MAIN] drive.shutdown error:", e, flush=True)
        try:
            from robot_cmd import close_link
            close_link()
        except Exception:
            pass
        super().closeEvent(ev)
    # ----------- навигация между экранами -----------
    def to_idle(self):
        self.stack.setCurrentWidget(self.pageIdle)
        # перерисуем оверлеи (если уже выбраны флаг/цель)
        refresh_all_overlays(
            self.state,
            self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
            self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"),
        )
        
    def _on_close_clicked(self):
        """Полное завершение приложения (включая потоки камеры и лидара)."""
        try:
            # безопасно остановить DrivePage
            if hasattr(self, "drive"):
                try:
                    self.drive.shutdown()
                except Exception:
                    pass
            QtWidgets.QApplication.quit()
        except Exception as e:
            print("[CLOSE] error:", e, flush=True)
            os._exit(0)  # жёсткий выход на крайний случай

    def to_drive(self):
        # подстраховка: если idle не установил режим — пусть будет markers
        if not getattr(self.state, "nav_mode", None):
            self.state.nav_mode = "markers"

        self.stack.setCurrentWidget(self.pageDrive)

        refresh_all_overlays(
            self.state,
            self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
            self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"),
        )

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
        start_dir = os.path.expanduser("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/maps_repo")
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog

        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Выбрать карту (PNG)", start_dir,
            "PNG (*.png);;Все файлы (*)", options=options
        )
        if not fname:
            return

        self._load_map_file(fname)
        
    def _load_map_file(self, fname: str):
        if not fname or not os.path.isfile(fname):
            return

        self._reset_everything_on_new_map()

        self.state.active_map_path = fname
        base = os.path.basename(fname)
        name, _ = os.path.splitext(base)
        self.state.map_name = name

        s = QtCore.QSettings("Rover", "PlatformGUI")
        s.setValue("maps/last_path", fname)
        s.setValue("maps/last_name", name)

        photos_dir = os.path.join(self.state.dataset_root, "photos", name)
        lidar_dir  = os.path.join(self.state.dataset_root, "lidar",  name)
        os.makedirs(photos_dir, exist_ok=True)
        os.makedirs(lidar_dir, exist_ok=True)

        self.state.visited_path_px = []

        idle_view  = self.findChild(QtWidgets.QGraphicsView, "mapViewIdle")
        drive_view = self.findChild(QtWidgets.QGraphicsView, "mapViewDrive")

        show_map_on_views(fname, idle_view, drive_view, self.state)

        from routing import load_graph_and_points_for, refresh_all_overlays
        load_graph_and_points_for(fname, self.state)
        refresh_all_overlays(self.state, idle_view, drive_view)

        if hasattr(self, "drive"):
            self.drive._load_persisted_settings()
            self.drive._sync_tuning_from_state()
            self.drive._apply_mode_tuning_ui()
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    except Exception:
        pass
    app = QtWidgets.QApplication(sys.argv)
    app.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app.setApplicationName("Platform GUI")

    w = Main()
    app.aboutToQuit.connect(w.drive.shutdown)
    # Рабочая область экрана (любое разрешение), не фиксированный 1920×1280 из Designer
    w.showMaximized()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()