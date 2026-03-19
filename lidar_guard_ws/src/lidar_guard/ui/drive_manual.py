# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtWidgets, QtGui
from robot_cmd import motors_set, update_drive_panel
from graphics import stop_route_animation


class ManualDriveWindow(QtWidgets.QDialog):
    def __init__(self, ui_parent, state, owner):
        super().__init__(ui_parent)
        self.state = state
        self.owner = owner

        self.setWindowTitle("Мануальное управление")
        self.setModal(False)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self.resize(300, 260)

        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setSpacing(8)

        self.lblPwm = QtWidgets.QLabel(self)
        vbox.addWidget(self.lblPwm, alignment=QtCore.Qt.AlignHCenter)
        self._update_pwm_label()

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)

        def add_btn(row, col, text, cmd):
            btn = QtWidgets.QPushButton(text, self)
            btn.setMinimumSize(160, 120)
            btn.clicked.connect(lambda _=None, c=cmd: self.owner.manual_cmd(c))
            grid.addWidget(btn, row, col)

        add_btn(0, 0, "↖", "fwd_left")
        add_btn(0, 1, "↑", "fwd")
        add_btn(0, 2, "↗", "fwd_right")
        add_btn(1, 0, "←", "left")
        add_btn(1, 1, "■", "stop")
        add_btn(1, 2, "→", "right")
        add_btn(2, 0, "↙", "back_left")
        add_btn(2, 1, "↓", "back")
        add_btn(2, 2, "↘", "back_right")

        vbox.addLayout(grid)

        BTN_W, BTN_H = 160, 120  # как в add_btn()

        h_pwm = QtWidgets.QHBoxLayout()

        self.btnPwmMinus = QtWidgets.QPushButton("− PWM", self)
        self.btnPwmPlus  = QtWidgets.QPushButton("+ PWM", self)

        for b in (self.btnPwmMinus, self.btnPwmPlus):
            b.setMinimumSize(BTN_W, BTN_H)
            b.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.btnPwmMinus.clicked.connect(lambda: self._change_pwm(-50))
        self.btnPwmPlus.clicked.connect(lambda: self._change_pwm(+50))

        h_pwm.addWidget(self.btnPwmMinus)
        h_pwm.addWidget(self.btnPwmPlus)
        vbox.addLayout(h_pwm)

        self.btnBack = QtWidgets.QPushButton("Назад", self)
        self.btnBack.setMinimumSize(BTN_W * 2, BTN_H)  # чтобы по ширине было как две PWM-кнопки
        self.btnBack.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.btnBack.clicked.connect(self.close)
        vbox.addWidget(self.btnBack)
        self.btnBack.clicked.connect(self.close)
        vbox.addWidget(self.btnBack)

    def showEvent(self, ev: QtGui.QShowEvent):
        super().showEvent(ev)
        self.owner.enter_manual_mode()

    def closeEvent(self, ev: QtGui.QCloseEvent):
        self.owner.exit_manual_mode()
        super().closeEvent(ev)

    def _change_pwm(self, delta):
        st = self.state
        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = max(1000, min(2000, base + delta))
        st.pwm_base_us = base
        self._update_pwm_label()

    def _update_pwm_label(self):
        base = int(getattr(self.state, "pwm_base_us", 1700) or 1700)
        self.lblPwm.setText(f"База PWM: {base} мкс")


class ManualController(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__(ui)
        self.ui = ui
        self.state = state

        self.btnManualDialog: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnManualDialog")
        self._manual_window = None

        if self.btnManualDialog:
            self.btnManualDialog.clicked.connect(self.open_window)

    def open_window(self):
        if self._manual_window is None:
            self._manual_window = ManualDriveWindow(self.ui, self.state, self)
        self._manual_window.show()
        self._manual_window.raise_()
        self._manual_window.activateWindow()

    def enter_manual_mode(self):
        self.state.manual_drive = True

        # остановить маршрут
        if getattr(self.state, "is_running", False):
            self.state.is_running = False
            try:
                stop_route_animation(self.state, keep_progress=True)
            except Exception:
                pass

        self.state.manual_l_pwm = 1500
        self.state.manual_r_pwm = 1500

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def exit_manual_mode(self):
        self.state.manual_drive = False
        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass

    def manual_cmd(self, direction: str):
        if not getattr(self.state, "manual_drive", False):
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), "Откройте окно мануала")
            return

        base = int(getattr(self.state, "pwm_base_us", 1800) or 1800)
        base = max(1500, min(2000, base))
        delta = base - 1500

        if direction == "stop":
            l = r = 1500
        elif direction == "fwd":
            l = r = 1500 + delta
        elif direction == "back":
            l = r = 1500 - delta
        elif direction == "left":
            l = 1500 - delta
            r = 1500 + delta
        elif direction == "right":
            l = 1500 + delta
            r = 1500 - delta
        elif direction == "fwd_left":
            l = 1500
            r = 1500 + delta
        elif direction == "fwd_right":
            l = 1500 + delta
            r = 1500
        elif direction == "back_left":
            l = 1500 - delta
            r = 1500
        elif direction == "back_right":
            l = 1500
            r = 1500 - delta
        else:
            l = r = 1500

        l = max(1000, min(2000, int(l)))
        r = max(1000, min(2000, int(r)))

        self.state.manual_l_pwm = l
        self.state.manual_r_pwm = r

        try:
            motors_set(self.state, l, r, None)
        except Exception as e:
            print("[MANUAL] motors_set error:", e, flush=True)

        try:
            update_drive_panel(self.ui, self.state)
        except Exception:
            pass