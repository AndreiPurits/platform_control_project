# robot_cmd.py
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

# -------- скорость / статусы (глобальные для state) -----------------

def set_speed(state, mps: float):
    state.speed_mps = float(mps)

def get_speed(state) -> float:
    return float(getattr(state, "speed_mps", 0.0) or 0.0)

def set_guard_state(state, value: str):
    state.guard_state = str(value)

def set_safety_stop(ui: QtWidgets.QMainWindow, enabled: bool):
    w = ui.findChild(QtWidgets.QLabel, "lblSafetyStop")
    if not w: return
    if enabled:
        w.setStyleSheet("color:#ffffff; background:#d64545; padding:2px 6px; border-radius:4px;")
        w.setText("ПРЕПЯТСТВИЕ")
    else:
        w.setStyleSheet("")
        w.setText("")

# -------- панель DRIVE (скорость/текст маршрута/статус) ---------------

from routing import route_caption_text

def update_drive_panel(ui: QtWidgets.QMainWindow, state) -> None:
    def set_lbl(name: str, text: str):
        w = ui.findChild(QtWidgets.QLabel, name)
        if w: w.setText(text)
    set_lbl("lblMinDist", route_caption_text(state))
    set_lbl("lblSpeed",   f"{get_speed(state):.1f} м/с")
    set_lbl("lblGuardState", str(getattr(state, "guard_state", "OFF")))
    # safety stop — пока пусто (в будущем дергаем set_safety_stop)
    set_lbl("lblSafetyStop", "")