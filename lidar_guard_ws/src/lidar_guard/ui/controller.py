# -*- coding: utf-8 -*-  # controller.py
import math

class DiffController:
    def __init__(self, kp_lat=0.8, kh_head=0.6, k_curv=180.0, v_pwm=120, smooth=0.6):
        self.kp_lat = kp_lat
        self.kh_head = kh_head
        self.k_curv = k_curv
        self.v_pwm = v_pwm
        self.smooth = smooth
        self._L = 0
        self._R = 0

    def step(self, e_lat_px, e_head_rad):
        # простая линейка: curvature ~ Kp*lat + Kh*head
        curv = self.kp_lat * (e_lat_px / 200.0) + self.kh_head * e_head_rad
        delta = int(self.k_curv * curv)
        L = max(0, min(255, self.v_pwm - delta))
        R = max(0, min(255, self.v_pwm + delta))
        # сглаживание
        Ls = int(self.smooth * self._L + (1 - self.smooth) * L)
        Rs = int(self.smooth * self._R + (1 - self.smooth) * R)
        self._L, self._R = Ls, Rs
        return Ls, Rs