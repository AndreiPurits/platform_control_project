# -*- coding: utf-8 -*- status.py

from PyQt5 import QtWidgets

def set_indicator(widget: QtWidgets.QLabel, state: str):
    """state: 'ok'|'warn'|'bad'|'off'."""
    if not widget: return
    colors = {"ok":"#38b000","warn":"#e0a800","bad":"#d64545","off":"#bdbdbd"}
    c = colors.get(state, "#bdbdbd")
    widget.setFixedSize(20,20)
    widget.setStyleSheet(f"background:{c}; border-radius:10px; border:1px solid #777;")
