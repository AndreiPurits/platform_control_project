#!/usr/bin/env python3
import json, socket, rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanToUdp(Node):
    def __init__(self, host='127.0.0.1', port=10000, topic='/scan', sample_step=1):
        super().__init__('scan_to_udp')
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dst = (host, port)
        self._step = max(1, int(sample_step))
        self._sub = self.create_subscription(LaserScan, topic, self._on_scan, 10)

    def _on_scan(self, msg: LaserScan):
        # Можно слать «LaserScan-подобный» JSON — твой LidarUdpReceiver это понимает
        ranges = msg.ranges[::self._step]
        payload = {
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment) * self._step,
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": [float(r) if math.isfinite(r) else None for r in ranges],
        }
        data = json.dumps(payload).encode('utf-8')
        try:
            self._sock.sendto(data, self._dst)
        except Exception as e:
            self.get_logger().warn(f"UDP send error: {e}")

def main():
    rclpy.init()
    node = ScanToUdp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()