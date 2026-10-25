
from setuptools import find_packages, setup

package_name = 'robotek_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/real_car_v1.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renzo Damian',
    maintainer_email='renzo.damian.go@gmail.com',
    description='Simulation-only evaluation: contact recorder, actuator and scan models, rule judges, the Open and Obstacle labs and the Obstacle field. A strategy must not depend on it.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'contact_recorder = robotek_evaluation.contact_recorder:main',
            'obstacle_recorder = robotek_evaluation.obstacle_recorder:main',
            'sim_actuator_model = robotek_evaluation.sim_actuator_model:main',
            'sim_scan_model = robotek_evaluation.sim_scan_model:main',
        ],
    },
)
