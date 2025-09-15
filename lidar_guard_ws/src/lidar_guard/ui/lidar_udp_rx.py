# -*- coding: utf-8 -*-
import json, socket, threading
from typing import List, Tuple
from PyQt5 import QtCore

class LidarUdpReceiver(QtCore.QThread):
    pointsReady = QtCore.pyqtSignal(list)

    def __init__(self, host='127.0.0.1', port=9999, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._stop = threading.Event()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        sock.settimeout(0.5)
        print(f"[LIDAR-RX] listening UDP {self.host}:{self.port}")
        while not self._stop.is_set():
            try:
                payload, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                msg = json.loads(payload.decode('utf-8'))
                angle = msg.get("angle_min", 0.0)
                inc   = msg.get("angle_increment", 0.0)
                rmin  = msg.get("range_min", 0.0)
                rmax  = msg.get("range_max", 15.0)
                ranges= msg.get("ranges", [])
                pts: List[Tuple[float,float]] = []
                for r in ranges:
                    if r is None or r <= 0.0 or r < rmin or r > rmax:
                        angle += inc
                        continue
                    # обычная математика (рад): cos/sin из math
                    from math import cos, sin
                    x = r * cos(angle)
                    y = r * sin(angle)
                    pts.append((x, y))
                    angle += inc
                print(f"[LIDAR-RX] pkt -> {len(pts)} pts")
                self.pointsReady.emit(pts)
            except Exception as e:
                print("[LIDAR-RX] decode err:", e)
                continue
        sock.close()

    def stop(self):
        self._stop.set()
