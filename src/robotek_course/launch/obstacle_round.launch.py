"""One round of the Obstacle Challenge: a field by the rules, the simulator, your node and the referee.

    ros2 launch robotek_course obstacle_round.launch.py strategy:=my_obstacle seed:=0 direction:=ccw
    ros2 launch robotek_course obstacle_round.launch.py strategy:=my_obstacle seed:=3 direction:=cw parking:=false
    ros2 launch robotek_course obstacle_round.launch.py strategy:=my_obstacle gui:=false set:=my_obstacle.mission.CRUISE_SPEED=0.3

``seed`` chooses the layout of the signs with the procedure of the rules (the same seed is the same field);
``parking`` adds the parking lot (and moves the signs of the start section inward, as the rules say). The field is
written into the round folder (``~/robotek_runs/<date>-<name>/world.sdf`` and ``manifest.json``) before Gazebo
starts. Your node (``strategy`` package, ``executable`` = drive) reads /scan, /imu, /camera/image and /start and
publishes /cmd_vel_raw; it gets the parameter ``parking`` (true when the field has the lot). The referee publishes
/start, keeps the true pose of the car, the pose of the signs and the contacts, ends the round and prints the score
(``score_obstacle``). The launch ends by itself when the round does.
"""
import datetime
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _round(context):
    from robotek_evaluation.obstacle_field import build_world
    share = get_package_share_directory
    strategy = LaunchConfiguration('strategy').perform(context)
    direction = LaunchConfiguration('direction').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    parking = LaunchConfiguration('parking').perform(context).lower() in ('1', 'true', 'yes')
    name = LaunchConfiguration('name').perform(context) or 'obstacle-%s-%d-%s' % (strategy, seed, direction)
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    directory = os.path.expanduser(os.path.join(LaunchConfiguration('runs_dir').perform(context), '%s-%s' % (stamp, name)))
    manifest = build_world(directory, seed, parking, direction)
    world = os.path.join(directory, 'world.sdf')
    use_sim_time = {'use_sim_time': True}
    print('Ronda guardada en %s (señales: %d, aparcamiento: %s)' % (directory, len(manifest['pillars']),
                                                                    'sí' if parking else 'no'), flush=True)
    recorder = Node(package='robotek_course', executable='obstacle_round_recorder', output='screen',
                    parameters=[{'output_dir': directory, 'direction': direction, 'strategy': strategy, 'world': world},
                                use_sim_time])
    sets = [item for item in LaunchConfiguration('set').perform(context).split(';') if item.strip()]
    executable = LaunchConfiguration('executable').perform(context)
    mine_params = [use_sim_time, {'parking': parking}]
    if sets:
        print('Strategy constants changed: %s' % sets, flush=True)
        code = ('from robotek_evaluation.open_lab import apply_sets; apply_sets(%r); from %s.%s import main; main()'
                % (sets, strategy, executable))
        mine = ExecuteProcess(cmd=[sys.executable, '-c', code, '--ros-args', '-r', '__node:=strategy',
                                   '-p', 'use_sim_time:=true', '-p', 'parking:=%s' % str(parking).lower()],
                              output='screen', on_exit=Shutdown())
    else:
        mine = Node(package=strategy, executable=executable, name='strategy',
                    output='screen', on_exit=Shutdown(), parameters=mine_params)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('robotek_course'), 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
                              'direction': direction, 'world': world,
                              'spawn_x': str(manifest['spawn']['x'])}.items()),
        # Contact sensors (signs, lot limits, walls) and the pose of the signs -> the recorder (audit only)
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='contacts_bridge',
             arguments=['/obstacle/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/audit/robot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/model/traffic_signs/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/model/traffic_signs/pose', '/obstacle/pillars_gt')],
             parameters=[use_sim_time], output='screen'),
        Node(package='robotek_evaluation', executable='obstacle_recorder',
             parameters=[{'output_dir': directory}, use_sim_time], output='screen'),
        mine,
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[
            OpaqueFunction(function=lambda _: [
                ExecuteProcess(cmd=['pkill', '-INT', '-f', 'gz sim.*' + world]),
                ExecuteProcess(cmd=['pkill', '-INT', '-f', '__node:=(robotek_bridge|scan_gz_bridge|contacts_bridge)'])]),
            EmitEvent(event=ShutdownEvent(reason='the round is over'))])),
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='my_obstacle', description='the package with your node'),
        DeclareLaunchArgument('executable', default_value='drive', description='the executable of that package'),
        DeclareLaunchArgument('seed', default_value='0', description='layout of the signs (rules procedure)'),
        DeclareLaunchArgument('direction', default_value='ccw', description='ccw or cw (where the car starts facing)'),
        DeclareLaunchArgument('parking', default_value='true', description='the parking lot in the start section'),
        DeclareLaunchArgument('gui', default_value='true', description='the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz with the scan and the camera'),
        DeclareLaunchArgument('name', default_value='', description='name of the round folder'),
        DeclareLaunchArgument('runs_dir', default_value='~/robotek_runs', description='where the rounds are saved'),
        DeclareLaunchArgument('set', default_value='',
                              description='tuning: constants of your package to change, module.NAME=value;module.NAME=value'),
        OpaqueFunction(function=_round),
    ])
