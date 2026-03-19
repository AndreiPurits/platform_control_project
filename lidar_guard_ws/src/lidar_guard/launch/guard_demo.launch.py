from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_guard')
    cfg_path = os.path.join(pkg_share, 'config', 'lidar.yaml')
    with open(cfg_path,'r') as f:
        cfg = yaml.safe_load(f)
    lidar_params = cfg.get('lidar', {})
    det_params = cfg.get('detector', {})
    nodes = [
        Node(package='lidar_guard', executable='fake_scan_node', name='fake_scan', parameters=[lidar_params]),
        Node(package='lidar_guard', executable='guard_node', name='guard', parameters=[det_params]),
    ]
    return LaunchDescription(nodes)
