from setuptools import setup
import os
from glob import glob

package_name = 'lidar_guard'
share_dir = os.path.join('share', package_name)

data_files = [
    (os.path.join(share_dir, 'launch'), ['launch/bringup.launch.py']),
    (os.path.join(share_dir, 'config'), ['config/sllidar_params.yaml']),
    (os.path.join(share_dir, 'ui'), [
        'ui/main_runtime.py',
        'ui/state.py',
        'ui/page_drive.py',
        'ui/graphics.py',
        'ui/widgets.py',
        'ui/page_idle.py',
        'ui/page_maps.py',
        'ui/maps_panel.py',
        'ui/routing.py',
        'ui/status.py',
        'ui/main_window.ui'
    ]),
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [f'resource/{package_name}']),
    (share_dir, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@example.com',
    description='UI + lidar bring-up',
    license='MIT',
)
