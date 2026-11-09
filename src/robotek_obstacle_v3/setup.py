from setuptools import find_packages, setup

package_name = 'robotek_obstacle_v3'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/car_picam.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renzo Damian',
    maintainer_email='renzo.damian.go@gmail.com',
    description='Obstacle v3 de WRO-Strategies como solución de referencia del curso (LiDAR, giroscopio y cámara).',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'drive = robotek_obstacle_v3.drive:main',
        ],
    },
)
