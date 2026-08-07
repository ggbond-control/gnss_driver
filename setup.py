from glob import glob
from setuptools import find_packages, setup

package_name = 'g60_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'template.ovjsn']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='g60_driver maintainers',
    maintainer_email='maintainer@example.com',
    description='ROS 2 Jazzy G60 GNSS driver with GPS/odometry alignment and OVJSN export.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'g60_serial = g60_driver.serial_node:main',
            'g60_gps_odom_alignment = g60_driver.gps_odom_alignment:main',
            'g60_fix_to_polyline = g60_driver.fix_to_polyline:main',
            'g60_transform_convert = g60_driver.transform_convert:main',
            'g60_transform = g60_driver.transform_node:main',
            'g60_transform_adjust = g60_driver.transform_adjust:main',
        ],
    },
)
