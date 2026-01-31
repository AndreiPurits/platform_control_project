#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge LaserScan -> UDP (JSON) для GUI.

Пример запуска:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 ros2_to_udp_bridge.py --topic /scan --port 10000

Для второго лидара:
  python3 ros2_to_udp_bridge.py --topic /rear/scan --port 10001
"""

import argparse
import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanToUdp(Node):
    def __init__(self, host: str, port: int, topic: str, sample_step: int = 1):
        super().__init__('scan_to_udp')

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst = (host, port)
        self._step = max(1, int(sample_step))

        self._sub = self.create_subscription(
            LaserScan,
            topic,
            self._on_scan,
            10,
        )

        self.get_logger().info(
            f"ScanToUdp: topic={topic} -> UDP {host}:{port}, step={self._step}"
        )
        self._n = 0
        self._t0 = self.get_clock().now()

    def _on_scan(self, msg: LaserScan):
        try:
            src = msg.ranges
            if src is None:
                return

            step = self._step
            payload_ranges = []
            # ВАЖНО: не слайсом, а по индексу — безопаснее для необычных типов
            for i in range(0, len(src), step):
                r = src[i]
                try:
                    rr = float(r)
                    if math.isfinite(rr):
                        payload_ranges.append(rr)
                    else:
                        payload_ranges.append(None)
                except Exception:
                    payload_ranges.append(None)

            payload = {
                "angle_min": float(msg.angle_min),
                "angle_increment": float(msg.angle_increment) * step,
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "ranges": payload_ranges,
            }

            data = json.dumps(payload).encode("utf-8")
            self._sock.sendto(data, self._dst)

        except Exception as e:
            # не даём ноде умереть
            self.get_logger().warn(f"ScanToUdp _on_scan error: {e}")
            return

def main():
    parser = argparse.ArgumentParser(
        description="Bridge LaserScan to UDP JSON for GUI"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="UDP host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=10000,
        help="UDP port (default: 10000)",
    )
    parser.add_argument(
        "--topic",
        default="/scan",
        help="LaserScan topic name (default: /scan)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="sample every N-th ray (default: 1)",
    )

    args = parser.parse_args()

    rclpy.init()
    node = ScanToUdp(
        host=args.host,
        port=args.port,
        topic=args.topic,
        sample_step=args.step,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()