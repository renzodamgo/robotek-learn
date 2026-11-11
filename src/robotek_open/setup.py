from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robotek_open'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/car_picam.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renzo Damian',
    maintainer_email='renzo.damian.go@gmail.com',
    description='Open Challenge strategy for the Robotek WRO 2026 car: LiDAR, gyro and camera only.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'open_v5 = robotek_open.v5.node:main',
        ],
    },
)
