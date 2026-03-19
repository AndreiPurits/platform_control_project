from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml, math

def normalize_params(lidar_params: dict):
    # если в конфиге вдруг есть старое поле 'obstacles' в виде списка диктов — конвертируем
    if isinstance(lidar_params.get('obstacles'), list):
        dists, angs = [], []
        for item in lidar_params['obstacles']:
            if isinstance(item, dict):
                dists.append(float(item.get('distance_m', 2.0)))
                angs.append(float(item.get('angle_deg', 0.0)))
        lidar_params.pop('obstacles', None)
        lidar_params['obstacles_dist_m'] = dists
        lidar_params['obstacles_angle_deg'] = angs
    return lidar_params

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_guard')
    cfg_path = os.path.join(pkg_share, 'config', 'lidar.yaml')
    with open(cfg_path,'r') as f:
        cfg = yaml.safe_load(f)
    lidar_params = normalize_params(cfg.get('lidar', {}))
    det_params = cfg.get('detector', {})
    return LaunchDescription([
        Node(package='lidar_guard', executable='fake_scan_node', name='fake_scan', parameters=[lidar_params]),
        Node(package='lidar_guard', executable='guard_node', name='guard', parameters=[det_params]),
        Node(package='lidar_guard', executable='gui_node', name='lidar_gui'),
    ])
