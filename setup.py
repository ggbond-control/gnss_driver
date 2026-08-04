from glob import glob
from setuptools import find_packages, setup

package_name = 'g60_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='g60_driver maintainers',
    maintainer_email='maintainer@example.com',
    description='ROS 2 Jazzy driver and local trajectory publisher for G60 GNSS receivers.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'g60_serial = g60_driver.serial_node:main',
            'g60_udp = g60_driver.udp_node:main',
            'g60_tcp = g60_driver.tcp_node:main',
            'g60_nmea_topic = g60_driver.topic_node:main',
            'g60_trajectory = g60_driver.trajectory_node:main',
            'g60_gps_odom_alignment = g60_driver.gps_odom_alignment:main',
        ],
    },
)
