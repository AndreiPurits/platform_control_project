# -*- coding: utf-8 -*-
import math
from PyQt5 import QtCore, QtWidgets, QtGui

from robot_cmd import motors_set, update_drive_panel
from graphics import stop_route_animation


# fallback: если у вас они импортируются из общего модуля — можно убрать
MODE_MARKERS = "markers"
MODE_ROAD    = "road"
MODE_TRAJ    = "traj"


class TrajectoryController(QtCore.QObject):
    def __init__(self, ui: QtWidgets.QMainWindow, state, on_start=None, on_stop=None):
        super().__init__(ui)
        self.ui = ui
        self.state = state
        self._on_start_cb = on_start
        self._on_stop_cb = on_stop

        self.btnTrajectory: QtWidgets.QPushButton = ui.findChild(QtWidgets.QPushButton, "btnTrajectory")

        self._traj_timer = QtCore.QTimer(self)
        self._traj_timer.setInterval(50)
        self._traj_timer.timeout.connect(self._trajectory_tick)

        if self.btnTrajectory:
            self.btnTrajectory.setCheckable(True)
            self.btnTrajectory.setChecked(self._is_traj_mode())
            self.btnTrajectory.toggled.connect(self._on_btn_toggled)

    # --- helpers ---
    def _is_traj_mode(self) -> bool:
        return (getattr(self.state, "nav_mode", MODE_MARKERS) == MODE_TRAJ)

    def _set_traj_mode(self, enabled: bool) -> None:
        st = self.state
        if enabled:
            st.nav_mode = MODE_TRAJ
            # для обратной совместимости (если где-то ещё проверяют)
            st.trajectory_mode = True
        else:
            # при выключении траектории НЕ навязываю какой режим дальше
            # обычно логично вернуться в MODE_ROAD или MODE_MARKERS — но это решает UI/page_drive
            st.trajectory_mode = False

    # ---- API expected by page_drive.py ----
    def set_enabled(self, enabled: bool):
        enabled = bool(enabled)
        self._apply(enabled)

        if self.btnTrajectory:
            self.btnTrajectory.blockSignals(True)
            self.btnTrajectory.setChecked(enabled)
            self.btnTrajectory.blockSignals(False)

    # ---- internal ----
    def _on_btn_toggled(self, checked: bool):
        self._apply(bool(checked))

    def _apply(self, enabled: bool):
        st = self.state

        if enabled:
            # режим траектории
            self._set_traj_mode(True)

            # траектория ведёт (но manual всегда имеет приоритет на моторы)
            st.is_running = True

            if not self._traj_timer.isActive():
                self._traj_timer.start()

            # стартовый мотор-команд сразу (только если НЕ manual)
            if not bool(getattr(st, "manual_drive", False)):
                try:
                    base = int(getattr(st, "pwm_base_us", 1700) or 1700)
                    base = max(1500, min(2000, base))
                    motors_set(st, base, base, getattr(st, "b_pwm", 1500) or 1500)
                except Exception:
                    pass

            if callable(self._on_start_cb):
                try:
                    self._on_start_cb()
                except Exception:
                    pass

            print("[TRAJ] ON", flush=True)
            return

        # ---- OFF ----
        # выключаем режим траектории (nav_mode пусть переключает UI)
        self._set_traj_mode(False)

        if self._traj_timer.isActive():
            self._traj_timer.stop()

        try:
            motors_set(st, 1500, 1500, getattr(st, "b_pwm", 1500) or 1500)
        except Exception:
            pass

        try:
            stop_route_animation(st, keep_progress=True)
        except Exception:
            pass

        if callable(self._on_stop_cb):
            try:
                self._on_stop_cb()
            except Exception:
                pass

        try:
            update_drive_panel(self.ui, st)
        except Exception:
            pass

        print("[TRAJ] OFF", flush=True)

    def _trajectory_tick(self):
        st = self.state

        # =========================================================
        # MODE GATE (ТОЛЬКО ОБЕРТКА):
        # Тикаем/рулим ТОЛЬКО если nav_mode == MODE_TRAJ
        # =========================================================
        if getattr(st, "nav_mode", MODE_MARKERS) != MODE_TRAJ:
            return

        # manual должен быть доступен всегда -> траектория не рулит моторами
        if bool(getattr(st, "manual_drive", False)):
            return

        # если вдруг кто-то выключил running
        if not bool(getattr(st, "is_running", False)):
            return

        pts = getattr(st, "route_pts_px", None)
        rp = getattr(st, "robot_px", None)
        if not pts or len(pts) < 2 or not rp:
            return

        rx, ry = float(rp[0]), float(rp[1])

        # nearest
        min_i, min_d2 = 0, 1e18
        for i, (x, y) in enumerate(pts):
            dx, dy = x - rx, y - ry
            d2 = dx * dx + dy * dy
            if d2 < min_d2:
                min_d2 = d2
                min_i = i

        n = len(pts)
        seg_i = min(min_i, n - 2)
        vx1, vy1 = pts[seg_i]
        vx2, vy2 = pts[seg_i + 1]

        d1_px = math.hypot(vx1 - rx, vy1 - ry)
        d2_px = math.hypot(vx2 - rx, vy2 - ry)
        dist_to_vertex_px = min(d1_px, d2_px)

        mpx_x = float(getattr(st, "m_per_px_x", 0.8342405111938622) or 0.8342405111938622)
        mpx_y = float(getattr(st, "m_per_px_y", 1.2604320351524956) or 1.2604320351524956)
        mpp = 0.5 * (mpx_x + mpx_y)

        dist_m = dist_to_vertex_px * mpp
        TURN_RADIUS_M = 0.5

        yaw = float(getattr(st, "robot_heading_rad", 0.0) or 0.0)

        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = max(1500, min(2000, base))

        if dist_m > TURN_RADIUS_M:
            st._traj_turning = False
            try:
                motors_set(st, base, base, getattr(st, "b_pwm", 1500) or 1500)
            except Exception:
                pass
            return

        seg_dx = vx2 - vx1
        seg_dy = vy2 - vy1
        if abs(seg_dx) < 1e-6 and abs(seg_dy) < 1e-6:
            return

        desired = math.atan2(seg_dy, seg_dx)
        err = desired - yaw
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi

        err_deg = math.degrees(err)
        turning_now = abs(err_deg) >= 3.0
        st._traj_turning = turning_now

        KP = 3.0
        delta = int(KP * err_deg) if turning_now else 0
        delta = max(-250, min(250, delta))

        l = max(1000, min(2000, base + delta))
        r = max(1000, min(2000, base - delta))

        try:
            motors_set(st, int(l), int(r), getattr(st, "b_pwm", 1500) or 1500)
        except Exception:
            pass