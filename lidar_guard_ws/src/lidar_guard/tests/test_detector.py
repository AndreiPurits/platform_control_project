from lidar_guard.detector import Detector
import numpy as np

def test_hysteresis():
    d = Detector(stop_distance_m=2.0, caution_distance_m=3.0, fov_guard_deg=120.0, hysteresis_m=0.2)
    ang = np.linspace(-np.pi/2, np.pi/2, 181)
    rng = np.full_like(ang, 5.0, dtype=float)
    st, dist = d.evaluate(ang, rng)
    assert st == "CLEAR"
    rng[:] = 1.9
    st, dist = d.evaluate(ang, rng)
    assert st == "STOP"
    rng[:] = 2.05
    st, dist = d.evaluate(ang, rng)
    assert st == "STOP"
    rng[:] = 2.25
    st, dist = d.evaluate(ang, rng)
    assert st in ("STOP","CAUTION")
