# -*- coding: utf-8 -*-
import os, sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
from state import AppState
from page_idle import IdlePage
from page_drive import DrivePage
from page_maps import MapsPage
from graphics import refresh_all_overlays

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
        self.pageIdle  = self.findChild(QtWidgets.QWidget, "pageIdle")
        self.pageDrive = self.findChild(QtWidgets.QWidget, "pageDrive")
        self.pageMaps  = self.findChild(QtWidgets.QWidget, "pageMaps")

        if not self.stack or not self.pageIdle or not self.pageDrive or not self.pageMaps:
            raise RuntimeError("Страницы stackRoot / pageIdle / pageDrive / pageMaps не найдены в UI.")

        # инициализируем контроллеры страниц
        self.idle  = IdlePage(self, self.state)
        self.drive = DrivePage(self, self.state)
        self.maps  = MapsPage(self, self.state)

        # элементы DRIVE top bar, которые обрабатываются на уровне Main
        self.btnStartStop: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnStartStop")
        self.btnMapPicker: QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnMapPicker")
        self.btnEStop:     QtWidgets.QPushButton = self.findChild(QtWidgets.QPushButton, "btnEStop")

        # старт/стоп
        if self.btnStartStop:
            self.btnStartStop.setCheckable(True)
            self.btnStartStop.setChecked(False)
            self.btnStartStop.setText("Пуск")
            self.btnStartStop.toggled.connect(self._on_startstop_toggled)

        # выбор карты с DRIVE возможен только при стопе
        if self.btnMapPicker:
            self.btnMapPicker.clicked.connect(self._on_drive_pick_map)

        # E-STOP (заглушка)
        if self.btnEStop:
            self.btnEStop.clicked.connect(lambda: QtWidgets.QMessageBox.information(
                self, "E-STOP", "Аварийный стоп (заглушка)"))

        # стартуем в IDLE
        self.to_idle()
        if self.statusBar():
            self.statusBar().showMessage("Готово", 2000)

    # ----------- навигация между экранами -----------
    def to_idle(self):
        self.stack.setCurrentWidget(self.pageIdle)
        # перерисуем оверлеи (если уже выбраны флаг/цель)
        refresh_all_overlays(self.state,
                             self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
                             self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"))

    def to_drive(self):
        self.stack.setCurrentWidget(self.pageDrive)
        # перерисуем оверлеи поверх карты DRIVE
        refresh_all_overlays(self.state,
                             self.findChild(QtWidgets.QGraphicsView, "mapViewIdle"),
                             self.findChild(QtWidgets.QGraphicsView, "mapViewDrive"))

    def to_maps(self):
        self.stack.setCurrentWidget(self.pageMaps)
        if self.statusBar():
            self.statusBar().showMessage("Выберите PNG в каталоге карт, рядом .json — опционально", 3000)

    # ----------- верхний бар DRIVE -----------
    def _on_startstop_toggled(self, checked: bool):
        self.state.is_running = bool(checked)
        if self.btnStartStop:
            self.btnStartStop.setText("Стоп" if checked else "Пуск")
        # ВАЖНО: НЕ возвращаемся в IDLE при стопе — остаёмся на DRIVE
        # обновим доступность кнопок на DRIVE (например, выбор карты разрешён только при стопе)
        if self.btnMapPicker:
            self.btnMapPicker.setEnabled(not self.state.is_running)

        # подсказка в статус-баре
        if self.statusBar():
            self.statusBar().showMessage("START" if checked else "STOP", 1500)

    def _on_drive_pick_map(self):
        if self.state.is_running:
            QtWidgets.QMessageBox.information(self, "Движение активно",
                "Сначала остановите движение (Стоп), затем выбирайте карту.")
            return
        self.to_maps()

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
