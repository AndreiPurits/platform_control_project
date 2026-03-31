import os
import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Порт по умолчанию: из окружения или авто-детект.
    # Важно: by-id у CP210x меняется между устройствами, поэтому жёсткий путь ломается.
    env_port = os.environ.get("LIDAR_SERIAL_PORT", "").strip()

    def _pick_default_lidar_port() -> str:
        # 1) стабильный симлинк (если настроен udev)
        if os.path.exists("/dev/rplidar"):
            return "/dev/rplidar"

        # 2) первый CP210x по by-id (самый частый вариант для RPLidar)
        for p in sorted(glob.glob("/dev/serial/by-id/*")):
            name = os.path.basename(p).lower()
            if "silicon_labs" in name or "cp210" in name:
                return p

        # 3) fallback на ttyUSB0 (если нет by-id)
        usb = sorted(glob.glob("/dev/ttyUSB*"))
        if usb:
            return usb[0]

        # 4) последний шанс — стандартный путь (пусть будет видно в логах)
        return "/dev/ttyUSB0"

    default_port = env_port if env_port else _pick_default_lidar_port()

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
