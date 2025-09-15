from setuptools import setup

package_name = 'lidar_guard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Unified bringup: sllidar_ros2 + GUI',
    license='MIT',
    entry_points={
        # консольных скриптов не объявляем — GUI стартуем через ExecuteProcess
    },
)
