from glob import glob
from setuptools import find_packages, setup

package_name = 'gnss_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'template.ovjsn']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/devices', glob('config/devices/*.yaml')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/data', ['data/.gitkeep']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='gnss_driver maintainers',
    maintainer_email='maintainer@example.com',
    description='Extensible ROS 2 Jazzy GNSS adapters, alignment, transforms and navigation bridge.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'g60_driver = gnss_driver.nodes.g60_node:main',
            'g90_driver = gnss_driver.nodes.g90_node:main',
            'd1m_bridge = gnss_driver.nodes.d1m_bridge_node:main',
            'gnss_alignment = gnss_driver.gps_odom_alignment:main',
            'gnss_trajectory = gnss_driver.fix_to_polyline:main',
            'gnss_transform = gnss_driver.transform_node:main',
            'gnss_transform_convert = gnss_driver.transform_convert:main',
            'gnss_transform_adjust = gnss_driver.transform_adjust:main',
            # Compatibility aliases
            'gnss_serial = gnss_driver.nodes.g60_node:main',
            'gnss_g90_serial = gnss_driver.nodes.g90_node:main',
            'gnss_rtk_bridge = gnss_driver.nodes.d1m_bridge_node:main',
        ],
    },
)
