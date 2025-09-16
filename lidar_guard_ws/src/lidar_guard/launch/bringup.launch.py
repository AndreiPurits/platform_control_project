import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Параметры лидара
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='460800')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')

    # 1) Драйвер RPLIDAR (ROS2)
    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        parameters=[{'serial_port': lidar_port, 'serial_baudrate': lidar_baud}],
        output='screen'
    )

    # 2) Инлайн-мост /scan -> UDP:9999 (LaserScan JSON для твоего LidarUdpReceiver)
    inline_bridge = (
        "import socket,json,math,rclpy;"
        "from rclpy.node import Node;"
        "from rclpy.qos import qos_profile_sensor_data;"
        "from sensor_msgs.msg import LaserScan;"
        "HOST,PORT='127.0.0.1',9999;"
        "class N(Node):"
        "  def __init__(self):"
        "    super().__init__('scan2udp');"
        "    self.s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);"
        "    self.dst=(HOST,PORT);"
        "    self.create_subscription(LaserScan,'/scan',self.cb,qos_profile_sensor_data);"
        "  def cb(self,m):"
        "    ranges=[float(x) if math.isfinite(x) else None for x in m.ranges];"
        "    pkt={'angle_min':float(m.angle_min),'angle_increment':float(m.angle_increment),"
        "         'range_min':float(m.range_min),'range_max':float(m.range_max),'ranges':ranges};"
        "    self.s.sendto(json.dumps(pkt,separators=(',',':')).encode(),self.dst);"
        "rclpy.init(); n=N();"
        "try: rclpy.spin(n)"
        "finally: n.destroy_node(); rclpy.shutdown()"
    )
    scan_to_udp = ExecuteProcess(cmd=['python3', '-c', inline_bridge], output='screen')

    # 3) Твой GUI (запуск напрямую из src)
    gui_path = '/home/andrei/lidar_guard_ws/src/lidar_guard/ui/main_runtime.py'
    gui = ExecuteProcess(cmd=['python3', gui_path], output='screen')

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        sllidar,       # лидар
        scan_to_udp,   # мост /scan -> UDP:9999
        gui,           # интерфейс
    ])
