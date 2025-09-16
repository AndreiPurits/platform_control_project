from setuptools import setup

package_name = 'lidar_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/lidar.yaml']),
        ('share/' + package_name + '/launch', ['launch/guard_demo.launch.py','launch/guard_gui_demo.launch.py']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools','numpy','PyYAML'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@example.com',
    description='Safety guard for lidar with STOP output.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guard_node = lidar_guard.guard_node:main',
            'fake_scan_node = lidar_guard.fake_scan_node:main',
            'gui_node = lidar_guard.gui_node:qt_main',
        ],
    },
)
