import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('lidar_guard')
    ui_main = os.path.join(share_dir, 'ui', 'main_runtime.py')

    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='460800')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')

    params_file = os.path.join(share_dir, 'config', 'sllidar_params.yaml')

    gui = ExecuteProcess(
        cmd=['python3', ui_main],
        output='screen'
    )

    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        parameters=[params_file, {'serial_port': lidar_port, 'serial_baudrate': lidar_baud}],
        output='screen'
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        sllidar,
        gui,
    ])
