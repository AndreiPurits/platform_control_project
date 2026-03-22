import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Порт по умолчанию: LIDAR_SERIAL_PORT из окружения или старый by-id CP2102.
    # Если 80008004 на всех baud — часто открыт не тот USB (тот же CP2102 у Arduino и т.д.): задай порт явно.
    _fallback_by_id = (
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
        "8200a3a9df73ef11b5d7c68c8fcc3fa0-if00-port0"
    )
    default_port = os.environ.get("LIDAR_SERIAL_PORT", _fallback_by_id)

    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value=default_port)
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='115200')
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

    # 2) Мост /scan -> UDP (скрипты из share после colcon install)
    pkg_share = get_package_share_directory('lidar_guard')
    path_br = os.path.join(pkg_share, 'ui', 'ros2_to_udp_bridge.py')
    scan_to_udp = ExecuteProcess(
        cmd=['python3', path_br],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # 3) GUI
    gui_path = os.path.join(pkg_share, 'ui', 'main_runtime.py')
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
