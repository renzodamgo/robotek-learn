"""The Open challenge on the real car: exactly car.launch.py with strategy:=my_open (same arguments, same nodes).

    ros2 launch robotek_course open_challenge.launch.py

Waits for Start: press the button of the RRC Lite board once. Press it again to stop. The first time, with the
car on blocks (wheels in the air). If the package my_open does not exist yet, it says how to get it
(bash tools/get-reference.sh) and exits with code 1. Use strategy:=<your package> for another strategy.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    path = os.path.join(get_package_share_directory('robotek_course'), 'launch', 'car.launch.py')
    spec = importlib.util.spec_from_file_location('robotek_car_launch', path)
    car = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(car)
    return car.generate_launch_description()
