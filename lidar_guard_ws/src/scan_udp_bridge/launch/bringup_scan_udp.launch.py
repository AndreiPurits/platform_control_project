from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sllidar_share = get_package_share_directory('sllidar_ros2')
    c1_launch = os.path.join(sllidar_share, 'launch', 'view_sllidar_c1.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(c1_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '460800'
            }.items()
        ),
        Node(
            package='scan_udp_bridge',
            executable='scan_to_udp',
            name='scan_to_udp',
            arguments=['--out_host','127.0.0.1','--out_port','10000','--topic','/scan'],
            output='screen'
        )
    ])