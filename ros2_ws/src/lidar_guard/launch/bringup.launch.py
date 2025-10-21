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

    # 1) Драйвер LIDAR (ROS2)
    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        parameters=[{'serial_port': lidar_port, 'serial_baudrate': lidar_baud}],
        output='screen'
    )

    # 2) Твой мост /scan -> UDP:10000
    # Если в скрипте другой путь/порт — поправь ниже строку path_br и/или сам скрипт.
    path_br = '/home/andrei/ros2_ws/src/lidar_guard/ui/ros2_to_udp_bridge.py'
    scan_to_udp = ExecuteProcess(
        cmd=['python3', path_br],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # 3) GUI
    gui_path = '/home/andrei/ros2_ws/src/lidar_guard/ui/main_runtime.py'
    gui = ExecuteProcess(
        cmd=['python3', gui_path],
        output='screen',
        respawn=False
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        sllidar,       # лидар
        scan_to_udp,   # мост /scan -> UDP:10000
        gui,           # интерфейс
    ])