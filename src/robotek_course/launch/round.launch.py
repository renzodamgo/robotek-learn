"""One round of the Open Challenge: the simulator, your node and the referee.

    ros2 launch robotek_course round.launch.py strategy:=my_open
    ros2 launch robotek_course round.launch.py strategy:=my_open direction:=cw gui:=false
    ros2 launch robotek_course round.launch.py strategy:=my_open actuator_params:=steer_delay=0.18 \\
        set:=my_open.mission.CRUISE_SPEED=1.0       # tuning experiments (step 12): a slower servo, a constant changed

Your node (``strategy`` package, ``executable`` = drive) reads /scan, /imu, /camera/image and /start and publishes
/cmd_vel_raw. ``round_recorder`` publishes /start, keeps the true pose of the car, ends the round after 180 s,
30 s without progress or three laps and a stop, and prints the score. Everything is saved in
``~/robotek_runs/<date>-<name>/``; the launch ends by itself when the round does.
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

WORLD = 'robotek_open_field.sdf'


def _round(context):
    share = get_package_share_directory
    strategy = LaunchConfiguration('strategy').perform(context)
    direction = LaunchConfiguration('direction').perform(context)
    name = LaunchConfiguration('name').perform(context) or '%s-%s' % (strategy, direction)
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    directory = os.path.expanduser(os.path.join(LaunchConfiguration('runs_dir').perform(context), '%s-%s' % (stamp, name)))
    world = LaunchConfiguration('world').perform(context) or os.path.join(share('robotek_gazebo'), 'worlds', WORLD)
    use_sim_time = {'use_sim_time': True}
    print('Ronda guardada en %s' % directory, flush=True)
    # The referee: Start, true pose, end of the round, score. When it ends, everything ends.
    recorder = Node(package='robotek_course', executable='round_recorder', output='screen',
                    parameters=[{'output_dir': directory, 'direction': direction, 'strategy': strategy, 'world': world},
                                use_sim_time])
    # Your strategy. With ``set:=`` it starts through a one-line wrapper that first changes constants of your package
    # (``module.NAME=value`` separated by ';', ``module.*.NAME`` = every submodule that has it); without it, as always.
    sets = [item for item in LaunchConfiguration('set').perform(context).split(';') if item.strip()]
    executable = LaunchConfiguration('executable').perform(context)
    if sets:
        print('Strategy constants changed: %s' % sets, flush=True)
        code = ('from robotek_evaluation.open_lab import apply_sets; apply_sets(%r); from %s.%s import main; main()'
                % (sets, strategy, executable))
        mine = ExecuteProcess(cmd=[sys.executable, '-c', code, '--ros-args', '-r', '__node:=strategy',
                                   '-p', 'use_sim_time:=true'], output='screen', on_exit=Shutdown())
    else:
        mine = Node(package=strategy, executable=executable, name='strategy',
                    output='screen', on_exit=Shutdown(), parameters=[use_sim_time])     # your node died: end the round now
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('robotek_course'), 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
                              'direction': direction, 'world': world,
                              'actuator_params': LaunchConfiguration('actuator_params'),
                              'actuator_params_file': LaunchConfiguration('actuator_params_file')}.items()),
        # Contact sensor of the walls -> contacts.jsonl
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='contacts_bridge',
             arguments=['/audit/robot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             parameters=[use_sim_time], output='screen'),
        Node(package='robotek_evaluation', executable='contact_recorder',
             parameters=[{'output_dir': directory}, use_sim_time], output='screen'),
        mine,
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[
            # Gazebo's ruby wrapper does not pass SIGINT on and its bridges can survive: they would break the next
            # round, so they are stopped by name before the launch ends.
            OpaqueFunction(function=lambda _: [
                ExecuteProcess(cmd=['pkill', '-INT', '-f', 'gz sim.*' + world]),
                ExecuteProcess(cmd=['pkill', '-INT', '-f', '__node:=(robotek_bridge|scan_gz_bridge|contacts_bridge)'])]),
            EmitEvent(event=ShutdownEvent(reason='the round is over'))])),
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='my_open', description='the package with your node'),
        DeclareLaunchArgument('executable', default_value='drive', description='the executable of that package'),
        DeclareLaunchArgument('direction', default_value='ccw', description='ccw or cw (where the car starts facing)'),
        DeclareLaunchArgument('gui', default_value='true', description='the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz with the scan and the camera'),
        DeclareLaunchArgument('name', default_value='', description='name of the round folder (default: strategy-direction)'),
        DeclareLaunchArgument('runs_dir', default_value='~/robotek_runs', description='where the rounds are saved'),
        DeclareLaunchArgument('world', default_value='', description='a world .sdf of your own'),
        DeclareLaunchArgument('actuator_params', default_value='',
                              description='tuning: override the car, name=value,name=value (see sim.launch.py)'),
        DeclareLaunchArgument('actuator_params_file', default_value='', description='tuning: a yaml for the car'),
        DeclareLaunchArgument('set', default_value='',
                              description='tuning: constants of your package to change, module.NAME=value;module.NAME=value'),
        OpaqueFunction(function=_round),
    ])
