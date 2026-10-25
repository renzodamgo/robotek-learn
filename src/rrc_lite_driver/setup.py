from setuptools import setup

package_name = 'rrc_lite_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.rrc_lite_driver_sdk'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
        ('share/' + package_name + '/config', ['config/controller_params.yaml', 'config/servo_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renzo Damian',
    maintainer_email='renzo.damian.go@gmail.com',
    description='Driver ROS 2 de la placa HiWonder RRC Lite (IMU, boton, bateria, LED, servo de direccion).',
    license='Hiwonder (con permiso); cambios del equipo Robotek: MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrc_driver = rrc_lite_driver.rrc_driver:main',
        ],
    },
)
