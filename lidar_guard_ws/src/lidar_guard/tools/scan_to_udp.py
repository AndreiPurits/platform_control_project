#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, json, math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HOST, PORT = '127.0.0.1', 9999  # порт твоего GUI

class Scan2UDP(Node):
    def __init__(self):
        super().__init__('scan2udp')
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dst = (HOST, PORT)
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, m: LaserScan):
        ranges = [float(x) if math.isfinite(x) else None for x in m.ranges]
        pkt = {
            "angle_min": float(m.angle_min),
            "angle_increment": float(m.angle_increment),
            "range_min": float(m.range_min),
            "range_max": float(m.range_max),
            "ranges": ranges
        }
        self.s.sendto(json.dumps(pkt, separators=(',',':')).encode(), self.dst)

def main():
    rclpy.init()
    n = Scan2UDP()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
