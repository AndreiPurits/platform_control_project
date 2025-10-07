# robot_cmd.py
# -*- coding: utf-8 -*-
from typing import Optional
from PyQt5 import QtWidgets
from routing import route_caption_text

def update_drive_panel(ui: QtWidgets.QMainWindow, state) -> None:
    """
    Обновляет:
      - lblMinDist    ← route_caption_text(state)
      - lblSpeed      ← заглушка "0.0 м/с" (потом подключите одометрию)
      - lblGuardState ← state.guard_state (или "OFF")
      - lblSafetyStop ← заглушка (потом подвяжем к лидару)
    """
    def set_lbl(name: str, text: str):
        w = ui.findChild(QtWidgets.QLabel, name)
        if w: w.setText(text)

    set_lbl("lblMinDist", route_caption_text(state))
    set_lbl("lblSpeed",   f"{getattr(state,'speed_mps',0.0):.1f} м/с")
    set_lbl("lblGuardState", str(getattr(state, "guard_state", "OFF")))
    # safety stop — пока выключен, просто пусто
    set_lbl("lblSafetyStop", "")

def set_speed(state, mps: float):
    state.speed_mps = float(mps)

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