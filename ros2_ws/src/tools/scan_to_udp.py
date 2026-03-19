# -*- coding: utf-8 -*-
import json, socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanToUDP(Node):
    def __init__(self, host='127.0.0.1', port=9999, topic='/scan'):
        super().__init__('scan_to_udp')
        self.pub_addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(LaserScan, topic, self.on_scan, 10)
        self.get_logger().info(f'Forwarding {topic} to UDP {host}:{port}')

    def on_scan(self, msg: LaserScan):
        # Сжимаем: шлём каждые N-й луч (чтобы не трамбовать UDP)
        step = max(1, len(msg.ranges)//720)  # не более ~720 лучей
        data = {
            "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment * step,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": [ (None if (r!=r or r<=0.0) else float(r)) for r in msg.ranges[::step] ],
        }
        try:
            payload = json.dumps(data, separators=(',',':')).encode('utf-8')
            self.sock.sendto(payload, self.pub_addr)
        except Exception as e:
            self.get_logger().warn(f'UDP send failed: {e}')

def main():
    rclpy.init()
    node = ScanToUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
