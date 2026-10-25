"""The simulator of the course: Gazebo, the field, the car as measured, its sensors and (optionally) RViz.

    ros2 launch robotek_course sim.launch.py                    # Gazebo window
    ros2 launch robotek_course sim.launch.py gui:=false rviz:=true
    ros2 launch robotek_course sim.launch.py direction:=cw      # the car starts facing the other way
    ros2 launch robotek_course sim.launch.py actuator_params:=steer_delay=0.18,steer_rate=0.6   # a slower servo (step 12)

It does not drive the car and does not record anything: ``round.launch.py`` adds your node and the referee.
The car is the real one: servo and motor with their delays (``real_car_v1.yaml``), a LiDAR that loses 4.3 % of
its rays, a camera at 6 fps and a gyroscope with bias and noise.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def actuator_overrides(text):
    """``name=value,name=value`` -> parameters of ``sim_actuator_model`` that replace the ones of real_car_v1.yaml.

    Values are numbers; a list is written with slashes (``speed_table_real=0.1/0.2/0.3/0.4/0.5``); ``speed_scale=0.85``
    multiplies the real speed table (a low battery, a stronger motor). Nothing given: the car as measured.
    """
    import yaml
    table = yaml.safe_load(open(os.path.join(get_package_share_directory('robotek_evaluation'), 'config',
                                              'real_car_v1.yaml')))['sim_actuator_model']['ros__parameters']
    out = {}
    for item in filter(None, (part.strip() for part in text.split(','))):
        name, _, value = item.partition('=')
        if name == 'speed_scale':
            out['speed_table_real'] = [float(x) * float(value) for x in table['speed_table_real']]
        elif '/' in value:
            out[name] = [float(x) for x in value.split('/')]
        else:
            out[name] = float(value)
    return out


def _nodes(context):
    share = get_package_share_directory
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)
    if not spawn_yaw:
        spawn_yaw = str(math.pi if LaunchConfiguration('direction').perform(context) == 'cw' else 0.0)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('robotek_gazebo'), 'launch', 'robotek_sim.launch.py')),
        launch_arguments={
            'field': 'open', 'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
            'spawn_x': LaunchConfiguration('spawn_x').perform(context), 'spawn_y': '-1.0', 'spawn_yaw': spawn_yaw,
            'ackermann_velocity_limit': '1.0', 'ackermann_steer_p_gain': '20.0',
            'ackermann_acceleration': '30.0',      # the actuator model limits the accelerations, not the plugin
            'sensor_noise': 'true', 'real_sensors': 'true',
        }.items())
    world = LaunchConfiguration('world').perform(context) or os.path.join(
        share('robotek_gazebo'), 'worlds', 'robotek_open_field.sdf')
    real_car = os.path.join(share('robotek_evaluation'), 'config', 'real_car_v1.yaml')
    use_sim_time = {'use_sim_time': True}
    extra = actuator_overrides(LaunchConfiguration('actuator_params').perform(context))
    if extra:
        print('Actuator model overrides: %s' % extra, flush=True)
    actuator_file = LaunchConfiguration('actuator_params_file').perform(context)
    return [
        # `gz sim` can outlive the launch (the ruby wrapper does not forward SIGINT) and would break the next round
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: [ExecuteProcess(cmd=['pkill', '-INT', '-f', 'gz sim.*' + world])])])),   # a new action each time
        sim,
        # /cmd_vel_raw (you) -> servo and motor of the real car -> /cmd_vel (Gazebo)
        Node(package='robotek_evaluation', executable='sim_actuator_model', name='sim_actuator_model',
             output='screen',
             parameters=[real_car] + ([actuator_file] if actuator_file else []) + [extra, use_sim_time]),
        # Gazebo's LiDAR (/scan_gz) -> the rays the LD19 loses -> /scan
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scan_gz_bridge',
             arguments=['/scan_gz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             parameters=[use_sim_time], output='screen'),
        Node(package='robotek_evaluation', executable='sim_scan_model', name='sim_scan_model',
             parameters=[use_sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share('robotek_description'), 'rviz', 'open_camera.rviz')],
             parameters=[use_sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz with the scan and the camera'),
        DeclareLaunchArgument('direction', default_value='ccw',
                              description='ccw: the car faces the counter-clockwise lap; cw: the other way'),
        DeclareLaunchArgument('spawn_yaw', default_value='', description='override the start heading (rad)'),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='start x on the bottom straight (m); the Obstacle starts at -0.25 (+0.25 in CW)'),
        DeclareLaunchArgument('world', default_value='', description='a world .sdf of your own'),
        DeclareLaunchArgument('actuator_params', default_value='',
                              description='override the car: name=value,name=value of sim_actuator_model '
                                          '(steer_delay, steer_rate, steer_tau, steer_limit, speed_scale...)'),
        DeclareLaunchArgument('actuator_params_file', default_value='',
                              description='a yaml with sim_actuator_model parameters that replace real_car_v1.yaml'),
        OpaqueFunction(function=_nodes),
    ])
