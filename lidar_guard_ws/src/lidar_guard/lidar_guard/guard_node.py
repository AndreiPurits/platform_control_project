import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Float32
import numpy as np
from .detector import Detector

class GuardNode(Node):
    def __init__(self):
        super().__init__('guard_node')
        self.declare_parameter('stop_distance_m', 1.5)
        self.declare_parameter('caution_distance_m', 2.5)
        self.declare_parameter('fov_guard_deg', 120.0)
        self.declare_parameter('hysteresis_m', 0.2)
        self.declare_parameter('print_debug', False)
        self.detector=Detector(
            stop_distance_m=self.get_parameter('stop_distance_m').value,
            caution_distance_m=self.get_parameter('caution_distance_m').value,
            fov_guard_deg=self.get_parameter('fov_guard_deg').value,
            hysteresis_m=self.get_parameter('hysteresis_m').value
        )
        self.print_debug=bool(self.get_parameter('print_debug').value)
        self.sub=self.create_subscription(LaserScan,'/scan',self.cb,qos_profile_sensor_data)
        self.pub_stop=self.create_publisher(Bool,'/safety_stop',10)
        self.pub_state=self.create_publisher(String,'/guard/state',10)
        self.pub_min=self.create_publisher(Float32,'/guard/min_distance',10)
    def cb(self,msg:LaserScan):
        n=len(msg.ranges)
        if n==0:
            return
        angles=np.linspace(msg.angle_min,msg.angle_max,n,endpoint=True)
        ranges=np.array(msg.ranges,dtype=float)
        st,d=self.detector.evaluate(angles,ranges)
        self.pub_stop.publish(Bool(data=st=='STOP'))
        self.pub_state.publish(String(data=st))
        self.pub_min.publish(Float32(data=float(d if np.isfinite(d) else np.inf)))
        if self.print_debug:
            self.get_logger().info(f'state={st} d={d:.3f}')
def main():
    rclpy.init()
    node=GuardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
