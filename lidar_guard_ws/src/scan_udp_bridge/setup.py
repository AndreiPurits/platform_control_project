from setuptools import setup

package_name = 'scan_udp_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_scan_udp.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@example.com',
    description='ROS2 /scan → UDP bridge and UDP converters',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_to_udp = scan_udp_bridge.scan_to_udp:main',
            'udp_bridge = scan_udp_bridge.udp_bridge:main',
        ],
    },
)