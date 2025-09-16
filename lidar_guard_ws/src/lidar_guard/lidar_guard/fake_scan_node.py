import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class FakeScanNode(Node):
    def __init__(self):
        super().__init__('fake_scan_node')

        self.declare_parameter('angle_min_deg', -135.0)
        self.declare_parameter('angle_max_deg', 135.0)
        self.declare_parameter('angle_step_deg', 0.5)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 30.0)
        self.declare_parameter('noise_sigma_m', 0.01)
        self.declare_parameter('rate_hz', 10.0)

        double_array = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('obstacles_dist_m', [2.0], descriptor=double_array)
        self.declare_parameter('obstacles_angle_deg', [0.0], descriptor=double_array)

        old = self.get_parameters_by_prefix('')
        if 'obstacles' in old:
            try:
                items = old['obstacles'].value
                dists, angs = [], []
                if isinstance(items, list):
                    for it in items:
                        if isinstance(it, dict):
                            dists.append(float(it.get('distance_m', 2.0)))
                            angs.append(float(it.get('angle_deg', 0.0)))
                self.set_parameters([
                    Parameter('obstacles_dist_m', value=dists),
                    Parameter('obstacles_angle_deg', value=angs),
                ])
                self.get_logger().warn("Converted legacy 'obstacles' to obstacles_dist_m/obstacles_angle_deg")
            except Exception as e:
                self.get_logger().error(f"Failed to convert legacy 'obstacles': {e}")

        self.angle_min = math.radians(float(self.get_parameter('angle_min_deg').value))
        self.angle_max = math.radians(float(self.get_parameter('angle_max_deg').value))
        step = math.radians(float(self.get_parameter('angle_step_deg').value))
        self.angles = np.arange(self.angle_min, self.angle_max + 1e-9, step)
        self.rmin = float(self.get_parameter('range_min_m').value)
        self.rmax = float(self.get_parameter('range_max_m').value)
        self.noise = float(self.get_parameter('noise_sigma_m').value)
        self.rate = float(self.get_parameter('rate_hz').value)

        dists = list(self.get_parameter('obstacles_dist_m').value or [])
        angs  = list(self.get_parameter('obstacles_angle_deg').value or [])
        n = min(len(dists), len(angs))
        self.obstacles = [(float(dists[i]), math.radians(float(angs[i]))) for i in range(n)]

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        period = max(1e-3, 1.0 / max(1.0, self.rate))
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(f"fake_scan_node started with {len(self.obstacles)} obstacles")

    def tick(self):
        r = np.full_like(self.angles, self.rmax, dtype=float)
        for dist, ang in self.obstacles:
            i = int(np.argmin(np.abs(self.angles - ang)))
            k = 3
            s = max(0, i - k)
            e = min(len(r), i + k + 1)
            r[s:e] = dist
        r = np.clip(r + np.random.normal(0.0, self.noise, size=r.shape), self.rmin, self.rmax)

        msg = LaserScan()
        msg.header.frame_id = 'lidar'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angles[1] - self.angles[0]) if len(self.angles) > 1 else 0.0
        msg.range_min = self.rmin
        msg.range_max = self.rmax
        msg.ranges = [float(x) for x in r]
        self.pub.publish(msg)
def main():
    rclpy.init()
    node = FakeScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
