from setuptools import setup
import os

package_name = 'lidar_guard'
share_dir = os.path.join('share', package_name)


def _collect_ui_share_files():
    """Install all top-level .py and .ui under ui/ (single tree: lidar_guard_ws/.../ui via symlink)."""
    ui_dir = 'ui'
    if not os.path.isdir(ui_dir):
        return []
    out = []
    for name in sorted(os.listdir(ui_dir)):
        path = os.path.join(ui_dir, name)
        if os.path.isfile(path) and name.endswith(('.py', '.ui')):
            out.append(path)
    return out


def _collect_idle_images():
    """idle_bg / mode_* PNG для page_idle._load_idle_images (корень .gitignore *.png не мешает pip install)."""
    d = os.path.join('ui', 'idle_images')
    if not os.path.isdir(d):
        return []
    return [
        os.path.join(d, name)
        for name in sorted(os.listdir(d))
        if name.endswith('.png') and os.path.isfile(os.path.join(d, name))
    ]


data_files = [
    (os.path.join(share_dir, 'launch'), ['launch/bringup.launch.py']),
    (os.path.join(share_dir, 'config'), ['config/sllidar_params.yaml']),
    (os.path.join(share_dir, 'ui'), _collect_ui_share_files()),
    (os.path.join(share_dir, 'ui', 'idle_images'), _collect_idle_images()),
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
