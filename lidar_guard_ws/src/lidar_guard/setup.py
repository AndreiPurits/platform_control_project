from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'lidar_guard'

def files(pattern):
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        # индекс пакетов
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),

        # конфиги (убрал расхождение в имени; положим оба, если есть)
        ('share/' + package_name + '/config', files('config/*.yaml')),

        # launch-файлы (добавил bringup и bridge)
        ('share/' + package_name + '/launch', files('launch/*.launch.py')),

        # ui (если есть .ui/.png/.json для графа рядом с картой — положим их как ресурсы)
        ('share/' + package_name + '/ui', files('ui/*.ui') + files('ui/*.png') + files('ui/*.json')),

        # README
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@example.com',
    description='Safety guard for lidar with STOP output.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # твои узлы:
            'guard_node = lidar_guard.guard_node:main',
            'fake_scan_node = lidar_guard.fake_scan_node:main',
            'gui_node = lidar_guard.gui_node:qt_main',

            # UDP мост (ROS2 LaserScan -> UDP JSON), чтобы не городить ExecuteProcess с python3 путём:
            # реализуй функцию main() в lidar_guard/ros2_to_udp_bridge.py
            'ros2_to_udp_bridge = lidar_guard.ros2_to_udp_bridge:main',

            # (опционально) UDP ресивер как отдельный утилитный нод (если захочешь тестить вне GUI):
            # реализуй main() в lidar_guard/udp_view_node.py (читает UDP и рисует PyQt окно с точками)
            # 'udp_view_node = lidar_guard.udp_view_node:main',
        ],
    },
)