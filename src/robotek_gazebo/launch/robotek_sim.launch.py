"""ROS 2-integrated simulation launch for Robotek (Gazebo + ros_gz bridge).

Starts Gazebo (via ros_gz_sim), the ros_gz bridge (clock, cmd_vel, odom,
sensors, ...), and spawns the Ackermann robot (URDF from urdf/ by default).

Usage:
    ros2 launch robotek_gazebo robotek_sim.launch.py
    ros2 launch robotek_gazebo robotek_sim.launch.py field:=square180
    ros2 launch robotek_gazebo robotek_sim.launch.py gui:=false
    ros2 launch robotek_gazebo robotek_sim.launch.py spawn:=false
    ros2 launch robotek_gazebo robotek_sim.launch.py robot:=/path/to/other.urdf
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# field name -> world file in share/robotek_gazebo/worlds ('open' is the default 1 m corridors, like square100)
FIELDS = {'open': 'robotek_open_field.sdf'}


def _launch(context):
    pkg_share = get_package_share_directory('robotek_gazebo')
    gz_share = get_package_share_directory('ros_gz_sim')

    field = LaunchConfiguration('field').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    gui_config = LaunchConfiguration('gui_config').perform(context) or os.path.join(
        pkg_share, 'config', 'gui_scene_only.config')
    verbose = LaunchConfiguration('verbose').perform(context)
    spawn = LaunchConfiguration('spawn').perform(context).lower() in ('1', 'true', 'yes')
    robot_arg = LaunchConfiguration('robot').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    if world_arg:
        world = world_arg
    else:
        world = os.path.join(pkg_share, 'worlds', FIELDS.get(field, field + '.sdf'))

    # Default robot description: robotek_description (shared with the real car)
    robot_file = robot_arg or os.path.join(
        get_package_share_directory('robotek_description'), 'urdf', 'robotek.urdf.xacro')

    # Make this package's models reachable via model:// URIs.
    models_dir = os.path.join(pkg_share, 'models')
    # package://robotek_description/meshes/... resolves against the parent of the package share.
    description_parent = os.path.dirname(get_package_share_directory('robotek_description'))
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join([models_dir, description_parent] + ([existing] if existing else []))

    # macOS: `gz sim` cannot run server and GUI in one process (gz-sim#44), so the server
    # runs headless and the GUI is a separate client.
    split_gui = gui == 'true' and sys.platform == 'darwin'
    gui_flags = f' --gui-config {gui_config}' if gui == 'true' and os.path.isfile(gui_config) else ''
    gz_args = (f'-v {verbose} -r' + ('' if gui == 'true' and not split_gui else ' -s')
               + ('' if split_gui else gui_flags) + f' {world}')

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='robotek_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_share, 'config', 'robotek_bridge.yaml'),
            }],
            output='screen',
        ),
    ]
    if split_gui:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', verbose] + gui_flags.split(),
                                      name='gazebo_gui', output='screen'))

    if spawn:
        is_xacro = robot_file.endswith('.xacro')
        if is_xacro:
            # Publish robot_description (URDF) and spawn from it.
            actions.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'robot_description': ParameterValue(
                            Command(['xacro ', robot_file, ' ackermann_velocity_limit:=',
                                     LaunchConfiguration('ackermann_velocity_limit'),
                                     ' ackermann_steer_p_gain:=',
                                     LaunchConfiguration('ackermann_steer_p_gain'),
                                     ' ackermann_acceleration:=',
                                     LaunchConfiguration('ackermann_acceleration'),
                                     ' sensor_noise:=',
                                     LaunchConfiguration('sensor_noise'),
                                     ' real_sensors:=',
                                     LaunchConfiguration('real_sensors')]),
                            value_type=str
                        ),
                        'use_sim_time': True,
                    }],
                    output='screen',
                )
            )
            create_args = ['-topic', 'robot_description']
        else:
            create_args = ['-file', robot_file]

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=create_args + [
                    '-name', 'robotek',
                    '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                    '-Y', spawn_yaw,
                ],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'field',
            default_value='square100',
            description=("Open field: square100, square180, rect100x180, rect180x100, shift140, east140x100 "
                         "(the corridors' widths) or 'open'."),
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Full path to an SDF world; overrides the field argument.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI (false = headless server).',
        ),
        DeclareLaunchArgument(
            'gui_config',
            default_value='',
            description=("Gazebo GUI layout; empty = config/gui_scene_only.config (only the 3D "
                         "view). Pass /path/to/gui.config for another layout."),
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='3',
            description='Gazebo verbosity level (0-4).',
        ),
        DeclareLaunchArgument(
            'spawn',
            default_value='true',
            description='Spawn the robot into the world.',
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description=('Robot .xacro/.urdf/.sdf to spawn '
                         '(default: package urdf/robotek.urdf.xacro).'),
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-1.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('ackermann_velocity_limit', default_value='1.0',
                              description='Gazebo shared linear/angular speed limit.'),
        DeclareLaunchArgument('ackermann_steer_p_gain', default_value='1.0',
                              description='Gazebo steering joint position P gain.'),
        DeclareLaunchArgument('real_sensors', default_value='false',
                              description='Real-car sensor rates, gyro noise and LiDAR field of view.'),
        DeclareLaunchArgument('sensor_noise', default_value='false',
                              description='LiDAR/gyro noise for honest simulation.'),
        DeclareLaunchArgument('ackermann_acceleration', default_value='2.0',
                              description='Gazebo Ackermann acceleration limit; gz-sim 8 applies it '
                                          'to both linear (m/s^2) and yaw-rate (rad/s^2) commands.'),
        OpaqueFunction(function=_launch),
    ])
