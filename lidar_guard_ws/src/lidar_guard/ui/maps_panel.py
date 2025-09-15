# -*- coding: utf-8 -*-
import os, math, shutil
from PyQt5 import QtCore, QtGui, QtWidgets
from .routing import load_map_spline_for
from .graphics import show_map_on_views

EXTS = (".png", ".jpg", ".jpeg", ".svg", ".bmp")

def ensure_maps_dir(maps_dir: str):
    os.makedirs(maps_dir, exist_ok=True)
    try:
        files = [f for f in os.listdir(maps_dir) if f.lower().endswith(EXTS)]
    except Exception:
        files = []
    if files:
        return

    # Сгенерировать test_map.png + test_map.json
    W, H = 1200, 800
    img = QtGui.QImage(W, H, QtGui.QImage.Format_ARGB32)
    img.fill(QtGui.QColor(250, 250, 250))
    p = QtGui.QPainter(img)
    grid = QtGui.QPen(QtGui.QColor(230,230,230)); grid.setWidth(1)
    p.setPen(grid)
    for x in range(0, W, 100): p.drawLine(x, 0, x, H)
    for y in range(0, H, 100): p.drawLine(0, y, W, y)
    p.setPen(QtGui.QPen(QtGui.QColor(180,180,180), 2)); p.drawRect(0, 0, W-1, H-1)
    cx, cy = W//2, H//2
    r_outer = min(W, H)//3
    r_inner = r_outer - 50
    p.setBrush(QtGui.QBrush(QtGui.QColor(220,220,220))); p.setPen(QtCore.Qt.NoPen)
    p.drawEllipse(QtCore.QRectF(cx-r_outer, cy-r_outer, 2*r_outer, 2*r_outer))
    p.setBrush(QtGui.QBrush(QtGui.QColor(250,250,250)))
    p.drawEllipse(QtCore.QRectF(cx-r_inner, cy-r_inner, 2*r_inner, 2*r_inner))
    p.end()
    png_path = os.path.join(maps_dir, "test_map.png")
    img.save(png_path, "PNG")

    r_mid = (r_outer + r_inner) / 2.0
    polyline = []
    for ang in range(0, 360, 2):
        a = math.radians(ang)
        x = cx + r_mid * math.cos(a)
        y = cy + r_mid * math.sin(a)
        polyline.append([x, y])
    json_path = os.path.join(maps_dir, "test_map.json")
    with open(json_path, "w", encoding="utf-8") as f:
        import json; json.dump({"polyline": polyline}, f, ensure_ascii=False, indent=2)

def populate_maps_list(listWidget: QtWidgets.QListWidget, maps_dir: str, active_map_path: str):
    if not listWidget:
        return
    ensure_maps_dir(maps_dir)
    listWidget.clear()
    print(f"[MAPS] scan dir: {maps_dir}")
    if not os.path.isdir(maps_dir):
        listWidget.addItem("Папка с картами не найдена. Создайте ~/maps_repo.")
        return
    files = [f for f in os.listdir(maps_dir) if f.lower().endswith(EXTS)]
    files.sort()
    if not files:
        listWidget.addItem("Карт не найдено. Нажмите 'Импорт карты…' или 'Открыть папку'.")
        return
    for fname in files:
        full = os.path.join(maps_dir, fname)
        icon = QtGui.QIcon()
        pm = QtGui.QPixmap(full)
        if not pm.isNull():
            icon = QtGui.QIcon(pm.scaled(listWidget.iconSize(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        it = QtWidgets.QListWidgetItem(icon, fname)
        it.setData(QtCore.Qt.UserRole, full)
        listWidget.addItem(it)
    if active_map_path:
        for i in range(listWidget.count()):
            it = listWidget.item(i)
            if it.data(QtCore.Qt.UserRole) == active_map_path:
                listWidget.setCurrentRow(i)
                break

def import_map_via_dialog(parent: QtWidgets.QWidget, maps_dir: str) -> str:
    exts = "Изображения (*.png *.jpg *.jpeg *.bmp *.svg);;Все файлы (*)"
    fname, _ = QtWidgets.QFileDialog.getOpenFileName(parent, "Импорт карты", os.path.expanduser("~"), exts)
    if not fname:
        return ""
    base = os.path.basename(fname)
    dst = os.path.join(maps_dir, base)
    if os.path.exists(dst):
        ret = QtWidgets.QMessageBox.question(parent, "Замена файла",
                    f"Файл {base} уже существует в maps_repo.\nЗаменить?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ret != QtWidgets.QMessageBox.Yes:
            return ""
    try:
        shutil.copy2(fname, dst)
    except Exception as e:
        QtWidgets.QMessageBox.warning(parent, "Ошибка импорта", f"Не удалось скопировать файл:\n{e}")
        return ""
    return dst

def pick_map(path: str, ui, state):
    state.active_map_path = path
    print(f"[MAPS] выбран файл карты: {path}")
    load_map_spline_for(path, state)
    show_map_on_views(path, ui.mapViewIdle, ui.mapViewDrive, state)
    ui.to_idle()
    if hasattr(ui, "statusBar"):
        try:
            ui.statusBar().showMessage(f"Карта выбрана: {os.path.basename(path)}", 3000)
        except Exception:
            pass
        