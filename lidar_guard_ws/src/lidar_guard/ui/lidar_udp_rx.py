# -*- coding: utf-8 -*-
import json, socket, threading
from typing import List, Tuple
from PyQt5 import QtCore
from math import cos, sin

__all__ = ["LidarUdpReceiver"]

class LidarUdpReceiver(QtCore.QThread):
    pointsReady = QtCore.pyqtSignal(list)  # list[(x,y)]

    def __init__(self, host='127.0.0.1', port=10000, parent=None, timeout_sec=0.5):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._timeout = timeout_sec

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        sock.settimeout(self._timeout)
        while not self._stop.is_set():
            try:
                payload, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                msg = json.loads(payload.decode('utf-8'))
            except Exception:
                continue

            # формат 1: LaserScan-подобный JSON
            if all(k in msg for k in ("angle_min", "angle_increment", "ranges")):
                angle = float(msg.get("angle_min", 0.0))
                inc   = float(msg.get("angle_increment", 0.0))
                rmin  = float(msg.get("range_min", 0.0))
                rmax  = float(msg.get("range_max", 20.0))
                pts: List[Tuple[float,float]] = []
                for r in msg.get("ranges", []):
                    if r is None:
                        angle += inc
                        continue
                    rr = float(r)
                    if rr <= 0.0 or rr < rmin or rr > rmax:
                        angle += inc
                        continue
                    pts.append((rr * cos(angle), rr * sin(angle)))
                    angle += inc
                if pts:
                    self.pointsReady.emit(pts)
                continue

            # формат 2: прямой список точек {"pts":[[x,y],...]}
            if "pts" in msg:
                pts = []
                for p in msg["pts"]:
                    if isinstance(p, (list, tuple)) and len(p) >= 2:
                        try:
                            pts.append((float(p[0]), float(p[1])))
                        except Exception:
                            pass
                if pts:
                    self.pointsReady.emit(pts)

        sock.close()

    def stop(self):
        self._stop.set()
