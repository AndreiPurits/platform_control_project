from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # По умолчанию берём GUI из lidar_guard_ws/src/lidar_guard/ui
    default_gui = os.path.expanduser('~/lidar_guard_ws/src/lidar_guard/ui/main_runtime.py')

    gui_path_arg = DeclareLaunchArgument(
        'gui_path',
        default_value=default_gui,
        description='Absolute path to main_runtime.py (GUI)'
    )
    # Если нужен твой qt5_env — переопредели python_bin при запуске
    python_bin_arg = DeclareLaunchArgument(
        'python_bin',
        default_value='/usr/bin/python3',
        description='Python interpreter to run GUI'
    )

    gui_path  = LaunchConfiguration('gui_path')
    python_bin = LaunchConfiguration('python_bin')

    # 1) Драйвер лидара (проверь порт!)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',     # см. ls -l /dev/serial/by-id/
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # 2) GUI (PyQt5) из твоего lidar_guard_ws
    gui_proc = ExecuteProcess(
        cmd=[python_bin, gui_path],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        gui_path_arg,
        python_bin_arg,
        lidar_node,
        gui_proc
    ])
