from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gardentech',
    maintainer_email='user@example.com',
    description='EtherCAT 기반 갠트리 로봇 모터 제어 ROS2 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_control.motor_driver_node:main',
        ],
    },
)
