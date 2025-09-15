# -*- coding: utf-8 -*-
from PyQt5.QtCore import Qt, QObject, QEvent

class MapClickFilter(QObject):
    """Фильтр для кликов по viewport() QGraphicsView."""
    def __init__(self, on_click):
        super().__init__()
        self.on_click = on_click  # callable(x_px: float, y_px: float)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
            pos = event.pos()
            self.on_click(float(pos.x()), float(pos.y()))
            return True
        return False