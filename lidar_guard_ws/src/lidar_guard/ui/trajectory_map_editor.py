#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import json
import math

from PyQt5 import QtWidgets, QtGui, QtCore


class TrajectoryCanvas(QtWidgets.QWidget):
    """
    Центральное поле, где пользователь кликает точки траектории.
    Логика:
      - ЛКМ: добавить точку
      - ПКМ: отменить последнюю точку
      - Esc (обрабатывается в окне) -> выключить режим рисования
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []          # список [(x,y), ...] в пикселях
        self.hover_pos = None     # (x,y) для превью линии
        self.drawing_enabled = True

        self.setMouseTracking(True)
        self.setMinimumSize(640, 480)
        self.setAutoFillBackground(True)

        pal = self.palette()
        pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#ffffff"))
        self.setPalette(pal)

    # --- служебные методы ---

    def set_drawing_enabled(self, enabled: bool):
        self.drawing_enabled = bool(enabled)
        state = "ON" if self.drawing_enabled else "OFF"
        print(f"[TRAJ] drawing mode: {state}", flush=True)
        self.update()

    def is_drawing_enabled(self) -> bool:
        return self.drawing_enabled

    # --- события мыши / курсора ---

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        # Если рисование выключено (Esc), игнорируем клики
        if not self.drawing_enabled:
            event.ignore()
            return

        if event.button() == QtCore.Qt.LeftButton:
            pos = event.pos()
            self._add_point(float(pos.x()), float(pos.y()))
        elif event.button() == QtCore.Qt.RightButton:
            # Отмена последней точки
            if self.points:
                removed = self.points.pop()
                print(f"[TRAJ] undo point: {removed}", flush=True)
                self.update()

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        if not self.drawing_enabled:
            self.hover_pos = None
            self.update()
            return

        pos = event.pos()
        self.hover_pos = (float(pos.x()), float(pos.y()))
        self.update()

    # --- рисование ---

    def paintEvent(self, event: QtGui.QPaintEvent):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)

        # фон
        qp.fillRect(self.rect(), QtGui.QColor("#ffffff"))

        # сетка (немного визуальной привязки)
        qp.setPen(QtGui.QPen(QtGui.QColor("#eeeeee"), 1, QtCore.Qt.DotLine))
        w = self.width()
        h = self.height()
        step = 50
        for x in range(0, w, step):
            qp.drawLine(x, 0, x, h)
        for y in range(0, h, step):
            qp.drawLine(0, y, w, y)

        # сама траектория
        if self.points:
            # линии
            qp.setPen(QtGui.QPen(QtGui.QColor("#e53935"), 3))
            for (x1, y1), (x2, y2) in zip(self.points, self.points[1:]):
                qp.drawLine(int(x1), int(y1), int(x2), int(y2))

            # точки
            qp.setBrush(QtGui.QBrush(QtGui.QColor("#1e88e5")))
            qp.setPen(QtGui.QPen(QtGui.QColor("#0d47a1"), 1))
            r = 4
            for (x, y) in self.points:
                qp.drawEllipse(QtCore.QPoint(int(x), int(y)), r, r)

        # превью к следующей точке (если включено рисование)
        if (
            self.drawing_enabled
            and self.hover_pos is not None
            and len(self.points) >= 1
        ):
            x_last, y_last = self.points[-1]
            xh, yh = self.hover_pos
            qp.setPen(QtGui.QPen(QtGui.QColor("#9e9e9e"), 1, QtCore.Qt.DashLine))
            qp.drawLine(int(x_last), int(y_last), int(xh), int(yh))

        qp.end()

    # --- логика добавления точки ---

    def _add_point(self, x: float, y: float):
        self.points.append((x, y))
        self.update()

        # считаем расстояние от предыдущей точки (если она была)
        if len(self.points) >= 2:
            x1, y1 = self.points[-2]
            x2, y2 = self.points[-1]
            dx = x2 - x1
            dy = y2 - y1
            dist_px = math.hypot(dx, dy)

            # узнаём масштаб у окна
            mpp = 1.0
            parent = self.parent()
            # parent должен быть TrajectoryMapEditor
            if isinstance(parent, TrajectoryMapEditor):
                mpp = parent.current_m_per_px()

            dist_m = dist_px * mpp
            print(f"[TRAJ] segment: {dist_m:.3f} м", flush=True)


class TrajectoryMapEditor(QtWidgets.QMainWindow):
    """
    Окно редактора траекторий:
      - рисуем ломаную
      - выбираем масштаб м/px (ОЧЕНЬ МЕЛКИЙ диапазон)
      - сохраняем PNG + два JSON-а:
          <name>.png
          <name>_points_pixels.json
          <name>_graph.json
    Esc -> выключает режим рисования (можно только смотреть / сохранять).
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trajectory Map Editor")
        self.resize(800, 600)

        # === центральное полотно ===
        self.canvas = TrajectoryCanvas(self)
        self.setCentralWidget(self.canvas)
        self.canvas.setFocusPolicy(QtCore.Qt.StrongFocus)

        # === нижняя панель ===
        bottom = QtWidgets.QWidget(self)
        bottom_layout = QtWidgets.QHBoxLayout(bottom)
        bottom_layout.setContentsMargins(8, 4, 8, 4)
        bottom_layout.setSpacing(10)

        # Масштаб
        bottom_layout.addWidget(QtWidgets.QLabel("Масштаб (м/px):"))

        self.scale_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # Диапазон в 100 раз меньше, чем был:
        # 1..500 -> 0.00001 .. 0.00500 м/px
        self.scale_slider.setMinimum(1)
        self.scale_slider.setMaximum(500)
        self.scale_slider.setValue(50)  # 0.00050 м/px по умолчанию
        self.scale_slider.valueChanged.connect(self._on_scale_changed)
        bottom_layout.addWidget(self.scale_slider)

        self.scale_label = QtWidgets.QLabel("0.00050")
        bottom_layout.addWidget(self.scale_label)

        bottom_layout.addSpacing(20)

        # Кнопки
        self.btn_undo = QtWidgets.QPushButton("Отменить точку")
        self.btn_undo.clicked.connect(self._on_undo)
        bottom_layout.addWidget(self.btn_undo)

        self.btn_clear = QtWidgets.QPushButton("Очистить")
        self.btn_clear.clicked.connect(self._on_clear)
        bottom_layout.addWidget(self.btn_clear)

        bottom_layout.addSpacing(20)

        self.btn_save = QtWidgets.QPushButton("Сохранить")
        self.btn_save.clicked.connect(self._on_save)
        bottom_layout.addWidget(self.btn_save)

        self.btn_exit = QtWidgets.QPushButton("Выход")
        self.btn_exit.clicked.connect(self.close)
        bottom_layout.addWidget(self.btn_exit)

        bottom_layout.addStretch(1)

        dock = QtWidgets.QDockWidget(self)
        dock.setTitleBarWidget(QtWidgets.QWidget())  # без заголовка
        dock.setWidget(bottom)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)

        # Инициализируем текст для текущего масштаба
        self._on_scale_changed(self.scale_slider.value())

    # === масштаб ===

    def current_m_per_px(self) -> float:
        """
        Возвращает текущий масштаб в м/px.
        Диапазон:
          slider = 1..500
          mpp    = slider / 100000.0 => 0.00001 .. 0.00500 м/px
        """
        v = self.scale_slider.value()
        return float(v) / 10000.0

    def _on_scale_changed(self, value: int):
        mpp = self.current_m_per_px()
        self.scale_label.setText(f"{mpp:.5f} м/px")

        # Пересчитываем общую длину текущей траектории в метрах
        pts = self.canvas.points
        if len(pts) >= 2:
            total_px = 0.0
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                total_px += math.hypot(x2 - x1, y2 - y1)
            total_m = total_px * mpp
            print(f"[TRAJ] total length: {total_m:.3f} м", flush=True)

    # === обработка клавиатуры (Esc) ===

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        if event.key() == QtCore.Qt.Key_Escape:
            # Esc выключает режим рисования
            self.canvas.set_drawing_enabled(not self.canvas.is_drawing_enabled())
            print(f"[TRAJ] ESC -> drawing {'ON' if self.canvas.is_drawing_enabled() else 'OFF'}", flush=True)
        else:
            super().keyPressEvent(event)

    # === кнопки ===

    def _on_undo(self):
        if self.canvas.points:
            removed = self.canvas.points.pop()
            print(f"[TRAJ] undo from button: {removed}", flush=True)
            self.canvas.update()

    def _on_clear(self):
        self.canvas.points.clear()
        self.canvas.update()
        print("[TRAJ] cleared all points", flush=True)

    def _on_save(self):
        pts = self.canvas.points
        if len(pts) < 2:
            QtWidgets.QMessageBox.warning(
                self,
                "Нет данных",
                "Нарисуйте хотя бы два пункта траектории, чтобы сохранить.",
            )
            return

        base_dir = os.getcwd()
        maps_dir = os.path.join(base_dir, "maps_repo")
        os.makedirs(maps_dir, exist_ok=True)
        # Предлагаем имя файла для PNG
        dlg = QtWidgets.QFileDialog(self, "Сохранить карту траектории", maps_dir)
        dlg.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
        dlg.setNameFilter("PNG images (*.png)")
        dlg.setDefaultSuffix("png")
        if not dlg.exec_():
            return

        png_name = os.path.basename(png_path)
        png_path = os.path.join(maps_dir, png_name)
        base, _ = os.path.splitext(png_path)        

        # --- 1) Сохраняем PNG и получаем координаты в СИСТЕМЕ PNG ---
        pts_img = self._save_png(png_path, pts)

        # --- 2) Сохраняем points_pixels JSON уже в координатах PNG ---
        points_json = base + "_points_pixels.json"
        self._save_points_json(points_json, pts_img)

        # --- 3) Сохраняем graph JSON в координатах PNG ---
        graph_json = base + "_graph.json"
        self._save_graph_json(graph_json, pts_img)

        print(f"[TRAJ] saved:\n  {png_path}\n  {points_json}\n  {graph_json}", flush=True)

    # === сохранение артефактов ===

    def _save_png(self, path: str, pts):
        """
        Рисуем PNG и возвращаем список точек в координатах картинки
        (с учётом сдвига bbox+padding). Линии между точками — ЧЁРНЫЕ.
        """
        if not pts:
            return []

        # bounding box траектории в координатах canvas
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        pad = 20
        w = int(max_x - min_x + 2 * pad)
        h = int(max_y - min_y + 2 * pad)
        if w < 100:
            w = 100
        if h < 100:
            h = 100

        # координаты точек в системе PNG (0..w, 0..h)
        pts_img = []
        for (x, y) in pts:
            xi = float(x - min_x + pad)
            yi = float(y - min_y + pad)
            pts_img.append((xi, yi))

        img = QtGui.QImage(w, h, QtGui.QImage.Format_RGB32)
        img.fill(QtGui.QColor("#ffffff"))
        qp = QtGui.QPainter(img)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)

        # сетка
        qp.setPen(QtGui.QPen(QtGui.QColor("#eeeeee"), 1, QtCore.Qt.DotLine))
        step = 50
        for x in range(0, w, step):
            qp.drawLine(x, 0, x, h)
        for y in range(0, h, step):
            qp.drawLine(0, y, w, y)

        # --- траектория чёрным цветом ---
        qp.setPen(QtGui.QPen(QtGui.QColor("#000000"), 3))
        for (x1, y1), (x2, y2) in zip(pts_img, pts_img[1:]):
            qp.drawLine(int(x1), int(y1), int(x2), int(y2))

        # точки — синие, как раньше
        qp.setBrush(QtGui.QBrush(QtGui.QColor("#1e88e5")))
        qp.setPen(QtGui.QPen(QtGui.QColor("#0d47a1"), 1))
        r = 4
        for (x, y) in pts_img:
            qp.drawEllipse(QtCore.QPoint(int(x), int(y)), r, r)

        qp.end()
        img.save(path, "PNG")

        return pts_img


    def _save_points_json(self, path: str, pts_img):
        """
        Сохраняем список точек в координатах PNG.
        """
        data = [[float(x), float(y)] for (x, y) in pts_img]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def _get_transform_params(self, pts):
        """Возвращает параметры для масштабирования и центрирования точек на холсте 600x450."""
        if not pts:
            return {"scale": 1.0, "tx": 0, "ty": 0, "width": 600, "height": 450}

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        bbox_width = max_x - min_x
        bbox_height = max_y - min_y

        # Размеры целевого холста
        canvas_width = 600
        canvas_height = 450

        # Отступы (чтобы точки не прилипали к краю)
        padding = 20

        # Доступное пространство для bounding box
        avail_width = canvas_width - 2 * padding
        avail_height = canvas_height - 2 * padding

        # Масштаб по ширине и высоте
        scale_x = avail_width / bbox_width if bbox_width > 0 else 1.0
        scale_y = avail_height / bbox_height if bbox_height > 0 else 1.0

        # Берём минимальный масштаб, чтобы всё влезло
        scale = min(scale_x, scale_y)

        # Центрируем
        scaled_bbox_width = bbox_width * scale
        scaled_bbox_height = bbox_height * scale

        tx = (canvas_width - scaled_bbox_width) / 2 - min_x * scale
        ty = (canvas_height - scaled_bbox_height) / 2 - min_y * scale

        return {
            "scale": scale,
            "tx": tx,
            "ty": ty,
            "width": canvas_width,
            "height": canvas_height,
        }

    def _save_graph_json(self, path: str, pts):
        """
        Простейший graph:
          - nodes.px = список вершин
          - nodes.m  = те же точки в метрах
          - edges: один edge от 0 до N-1 с poly_px / poly_m
          - + пишем масштаб m_per_px_x / m_per_px_y в корень графа
        """
        if len(pts) < 2:
            graph = {
                "nodes": {"px": [], "m": []},
                "edges": [],
                "m_per_px_x": self.current_m_per_px(),
                "m_per_px_y": self.current_m_per_px(),
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(graph, f, ensure_ascii=False, indent=2)
            return

        mpp = self.current_m_per_px()

        nodes_px = [[float(x), float(y)] for (x, y) in pts]
        nodes_m  = [[float(x) * mpp, float(y) * mpp] for (x, y) in pts]

        edge = {
            "u": 0,
            "v": len(nodes_px) - 1,
            "poly_px": nodes_px,
            "poly_m": nodes_m,
        }

        graph = {
            "nodes": {
                "px": nodes_px,
                "m":  nodes_m,
            },
            "edges": [edge],
            "m_per_px_x": mpp,
            "m_per_px_y": mpp,
        }

        with open(path, "w", encoding="utf-8") as f:
            json.dump(graph, f, ensure_ascii=False, indent=2)

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = TrajectoryMapEditor()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()