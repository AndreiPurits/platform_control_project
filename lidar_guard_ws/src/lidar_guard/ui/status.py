# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

def set_indicator(widget: QtWidgets.QLabel, state: str):
    """state: 'ok'|'warn'|'bad'|'off'."""
    if not widget: return
    colors = {"ok":"#38b000","warn":"#e0a800","bad":"#d64545","off":"#bdbdbd"}
    c = colors.get(state, "#bdbdbd")
    widget.setFixedSize(20,20)
    widget.setStyleSheet(f"background:{c}; border-radius:10px; border:1px solid #777;")

def set_stop_banner(frame: QtWidgets.QFrame, label: QtWidgets.QLabel, reason: str|None):
    if not frame or not label: return
    if not reason:
        frame.hide(); return
    palette = {
        "Guard STOP":   ("#ffeeba", "#856404"),
        "E-STOP":       ("#f5c6cb", "#721c24"),
        "Operator STOP":("#cce5ff", "#004085"),
        "Battery":      ("#d1ecf1", "#0c5460"),
        "Fault":        ("#f8d7da", "#721c24"),
    }
    bg, fg = palette.get(reason, ("#e2e3e5", "#383d41"))
    frame.setStyleSheet(f"background:{bg}; border:1px solid #aaa; border-radius:6px; padding:6px;")
    label.setStyleSheet(f"color:{fg}; font-weight:600;")
    label.setText(reason)
    frame.show()