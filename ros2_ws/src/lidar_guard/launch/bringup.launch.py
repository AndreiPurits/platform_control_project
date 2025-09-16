import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # параметры лидара
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='460800')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')

    # 1) драйвер RPLIDAR (ROS2)
    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        parameters=[{'serial_port': lidar_port, 'serial_baudrate': lidar_baud}],
        output='screen'
    )

    # 2) мост /scan -> UDP:9999 (отдельный .py-файл, тот же что руками работал)
    bridge_path = '/home/andrei/lidar_guard_ws/src/lidar_guard/tools/scan_to_udp.py'
    scan_to_udp = ExecuteProcess(
        cmd=['python3', bridge_path],
        output='screen',
        respawn=True,  # если упадёт — перезапустится
    )

    # 3) GUI из исходников
    gui_path = '/home/andrei/lidar_guard_ws/src/lidar_guard/ui/main_runtime.py'
    gui = ExecuteProcess(
        cmd=['python3', gui_path],
        output='screen'
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        sllidar,
        scan_to_udp,
        gui,
    ])
