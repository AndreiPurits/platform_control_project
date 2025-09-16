import numpy as np

class Detector:
    def __init__(self, stop_distance_m=1.5, caution_distance_m=2.5, fov_guard_deg=120.0, hysteresis_m=0.2):
        self.stop_d=float(stop_distance_m)
        self.caut_d=float(caution_distance_m)
        self.fov=float(np.deg2rad(fov_guard_deg))/2.0
        self.h=float(hysteresis_m)
        self.state="CLEAR"
        self.last_d=float("inf")
    def evaluate(self, angles, ranges):
        angles=np.asarray(angles); ranges=np.asarray(ranges)
        m=np.isfinite(ranges) & (np.abs(angles)<=self.fov)
        d=np.min(ranges[m]) if np.any(m) else float("inf")
        thr_stop=self.stop_d if self.state!="STOP" else self.stop_d+self.h
        thr_caut=self.caut_d if self.state!="CAUTION" else self.caut_d+self.h
        if d<=thr_stop: self.state="STOP"
        elif d<=thr_caut: self.state="CAUTION"
        else: self.state="CLEAR"
        self.last_d=d
        return self.state,d
