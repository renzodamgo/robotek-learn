from glob import glob

from setuptools import find_packages, setup

package_name = 'robotek_course'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renzo Damian',
    maintainer_email='renzo.damian.go@gmail.com',
    description='Helpers for the Open Challenge course: round launch, recorder, score, mini simulator, strategy base.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'round_recorder = robotek_course.round_recorder:main',
            'score = robotek_course.score:main',
            'keyboard_drive = robotek_course.keyboard_drive:main',
        ],
    },
)
