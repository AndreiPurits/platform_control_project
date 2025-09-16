#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, json, math, argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class ScanToUdp(Node):
    def __init__(self, host, port, topic):
        super().__init__('scan_to_udp')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dst = (host, port)
        self.sub = self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg: LaserScan):
        ranges = [float(r) if math.isfinite(r) else None for r in msg.ranges]
        payload = {
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": ranges
        }
        self.sock.sendto(json.dumps(payload, separators=(',', ':')).encode('utf-8'), self.dst)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out_host', default='127.0.0.1')
    ap.add_argument('--out_port', type=int, default=10000)
    ap.add_argument('--topic', default='/scan')
    args = ap.parse_args()
    rclpy.init()
    node = ScanToUdp(args.out_host, args.out_port, args.topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()