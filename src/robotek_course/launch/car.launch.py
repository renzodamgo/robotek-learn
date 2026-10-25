"""Your strategy on the real car: the hardware nodes of robotek_hardware and your node. No Gazebo, no simulated actuators.

    ros2 launch robotek_course car.launch.py strategy:=my_open
    ros2 launch robotek_course open_challenge.launch.py      (the same, with the Open challenge strategy my_open)
    ros2 launch robotek_course car.launch.py strategy:=my_open battery_guard:=false status_leds:=false

Your node (``strategy`` package, ``executable`` = drive) is the same one that runs in ``round.launch.py``. It reads /scan,
/imu, /camera/image and /start and publishes /cmd_vel_raw (linear.x = speed in m/s, angular.z = speed * tan(steer) / WHEELBASE).
Here those names are connected to the car:

  LiDAR LD19   -> /scan (driver) -> scan_adapter  -> /robotek/scan   (0 rad ahead, +-110 deg)  -> your /scan
  RRC Lite IMU -> /rrc/imu_raw   -> imu_adapter   -> /robotek/imu    (x forward, y left, z up) -> your /imu
  USB camera   -> /usb_cam/image_raw                                                           -> your /camera/image
  RRC Lite button -> start_button -> /start (first press) and /robotek/stop (any later press)
  your /cmd_vel_raw -> base_adapter -> servo (microseconds) on the RRC Lite + /motor_vel -> motor_driver (TB6612)

Started here (each driver is skipped with a message when its package is not installed, so the launch can be tried
on a laptop; see src/robotek_hardware/README.md for how to install them):
  - rrc_lite_driver (the RRC Lite board: IMU, button, servo, RGBs) and the LiDAR driver (ldlidar_stl_ros2, LD19)
  - the camera: robotek_hardware/camera_publisher (camera_driver:=usb_cam for the ROS package)
  - robotek_hardware: scan_adapter, imu_adapter, motor_driver, base_adapter, start_button, battery_guard, status_leds
  - robot_state_publisher with robotek_description (only if installed; useful to see the car in RViz on a laptop)
  - your strategy, with use_sim_time false

Press the button of the board once to start. Press it again to stop: base_adapter and motor_driver hold the motor
at 0 until you launch again. The parameters of the car (servo, motor, wheelbase, LiDAR, IMU axes) are in
robotek_hardware/config/robot.yaml.
"""
import os
import sys

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(package):
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return None


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _check_strategy(context):
    """Before starting anything: is the strategy package built? If not, say how to get one and exit with code 1."""
    package = _arg(context, 'strategy')
    try:
        get_package_prefix(package)
    except PackageNotFoundError:
        print('\n[car] No encuentro el paquete de la estrategia "%s" (strategy:=%s).\n'
              '  Hay dos caminos:\n'
              '    1) La estrategia de referencia (la terminada del paso 9):  bash tools/get-reference.sh\n'
              '       y despues:  source install/setup.bash\n'
              '    2) Tu propio paquete: crealo y compilalo como en el paso 0 de la guia\n'
              '       (ros2 pkg create ... my_open; colcon build --symlink-install; source install/setup.bash).\n'
              '  Si ya lo compilaste, falta cargar el entorno:  source install/setup.bash\n'
              % (package, package), file=sys.stderr)
        sys.exit(1)
    return []


def _drivers(context):
    """Third-party drivers (not in this repository): started when installed, a message when not."""
    actions = []
    if _arg(context, 'controller').lower() == 'true':
        share = _share('rrc_lite_driver')
        launch = os.path.join(share or '', 'launch', _arg(context, 'controller_launch'))
        if share and os.path.exists(launch):
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch)))
        else:
            actions.append(LogInfo(msg='[car] rrc_lite_driver launch not found: start the RRC Lite driver '
                                       '(IMU, button, servo, RGBs) yourself or follow src/robotek_hardware/README.md.'))
    package, launch_file = _arg(context, 'lidar_package'), _arg(context, 'lidar_launch')
    if package and launch_file:
        share = _share(package)
        if share and os.path.exists(os.path.join(share, 'launch', launch_file)):
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', launch_file))))
        else:
            actions.append(LogInfo(msg='[car] LiDAR launch %s/%s not found: the driver must publish %s.'
                                       % (package, launch_file, _arg(context, 'raw_scan_topic'))))
    else:
        actions.append(LogInfo(msg='[car] LiDAR driver not launched here: it must publish ' + _arg(context, 'raw_scan_topic')))
    if _arg(context, 'camera').lower() == 'true':
        device = _arg(context, 'video_device')
        if _arg(context, 'camera_driver') == 'usb_cam':
            if _share('usb_cam'):
                actions.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
                                    namespace='usb_cam', output='screen',
                                    parameters=[{'video_device': device or '/dev/video0', 'image_width': 640,
                                                 'image_height': 480, 'framerate': 30.0, 'pixel_format': 'yuyv'}]))
            else:
                actions.append(LogInfo(msg='[car] usb_cam not found (sudo apt install ros-jazzy-usb-cam): '
                                           'the camera must publish ' + _arg(context, 'camera_topic')))
        else:
            # robotek_hardware/camera_publisher: finds the camera by itself and keeps retrying if it is missing.
            actions.append(Node(package='robotek_hardware', executable='camera_publisher', name='camera_publisher',
                                output='screen', parameters=[{'device': device}]))
    return actions


def _description(context):
    """TF of the car for RViz, only if its packages are installed."""
    if _arg(context, 'description').lower() != 'true':
        return []
    if not (_share('robotek_description') and _share('robot_state_publisher') and _share('xacro')):
        return [LogInfo(msg='[car] robot_state_publisher, xacro or robotek_description not installed: no TF for RViz.')]
    urdf = os.path.join(_share('robotek_description'), 'urdf', 'robotek.urdf.xacro')
    return [Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)}])]


def generate_launch_description():
    hardware_config = os.path.join(get_package_share_directory('robotek_hardware'), 'config', 'robot.yaml')
    with open(hardware_config) as stream:
        default_max_pct = str(yaml.safe_load(stream)['base_adapter']['ros__parameters']['motor_max_pct'])
    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='my_open', description='the package with your node'),
        DeclareLaunchArgument('executable', default_value='drive', description='the executable of that package'),
        DeclareLaunchArgument('controller', default_value='true', description='start rrc_lite_driver (the board)'),
        DeclareLaunchArgument('controller_launch', default_value='controller.launch.py',
                              description='launch file of rrc_lite_driver'),
        DeclareLaunchArgument('lidar_package', default_value='ldlidar_stl_ros2',
                              description='package of the LiDAR driver; empty = start it yourself'),
        DeclareLaunchArgument('lidar_launch', default_value='ld19.launch.py', description='launch file of that package'),
        DeclareLaunchArgument('raw_scan_topic', default_value='/scan', description='what the LiDAR driver publishes'),
        DeclareLaunchArgument('imu_topic', default_value='/rrc/imu_raw', description='what the RRC Lite driver publishes'),
        DeclareLaunchArgument('camera', default_value='true', description='start the camera node'),
        DeclareLaunchArgument('camera_driver', default_value='camera_publisher',
                              description="'camera_publisher' (robotek_hardware, retries if the camera is missing) "
                                          "or 'usb_cam' (the ROS package)"),
        DeclareLaunchArgument('video_device', default_value='',
                              description='your camera; empty = the first /dev/v4l/by-id/*-video-index0, else '
                                          '/dev/video0. A stable by-id path is better than /dev/videoN'),
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw'),
        DeclareLaunchArgument('motor_max_pct', default_value=default_max_pct,
                              description='motor ceiling in % (robot.yaml). First run on the car: 30, on blocks'),
        DeclareLaunchArgument('battery_guard', default_value='true',
                              description='red blinking RGBs when the battery is low (does not stop the car)'),
        DeclareLaunchArgument('battery_min_v', default_value='6.8', description='battery voltage (V) under which to change it'),
        DeclareLaunchArgument('status_leds', default_value='true',
                              description='state on the RRC Lite RGBs (green = ready for Start)'),
        DeclareLaunchArgument('description', default_value='true', description='robot_state_publisher, if installed'),
        OpaqueFunction(function=_check_strategy),
        OpaqueFunction(function=_drivers),
        Node(package='robotek_hardware', executable='scan_adapter', name='scan_adapter', output='screen',
             parameters=[hardware_config],
             remappings=[('scan_in', LaunchConfiguration('raw_scan_topic')), ('scan_out', '/robotek/scan')]),
        Node(package='robotek_hardware', executable='imu_adapter', name='imu_adapter', output='screen',
             parameters=[hardware_config],
             remappings=[('imu_in', LaunchConfiguration('imu_topic')), ('imu_out', '/robotek/imu')]),
        Node(package='robotek_hardware', executable='motor_driver', name='tb6612fng_motor', output='screen',
             parameters=[hardware_config]),
        # Your /cmd_vel_raw is the base_adapter's input (in the simulator sim_actuator_model reads the same topic).
        Node(package='robotek_hardware', executable='base_adapter', name='base_adapter', output='screen',
             parameters=[hardware_config, {'motor_max_pct': ParameterValue(LaunchConfiguration('motor_max_pct'),
                                                                           value_type=float)}],
             remappings=[('cmd_vel', '/cmd_vel_raw')]),
        Node(package='robotek_hardware', executable='start_button', name='start_button', output='screen',
             remappings=[('start', '/start')]),
        Node(package='robotek_hardware', executable='battery_guard', name='battery_guard', output='screen',
             condition=IfCondition(LaunchConfiguration('battery_guard')),
             parameters=[{'min_voltage': ParameterValue(LaunchConfiguration('battery_min_v'), value_type=float)}]),
        Node(package='robotek_hardware', executable='status_leds', name='status_leds', output='screen',
             condition=IfCondition(LaunchConfiguration('status_leds')),
             parameters=[{'imu_topic': LaunchConfiguration('imu_topic'),
                          'camera_topic': LaunchConfiguration('camera_topic'),
                          'camera_required': ParameterValue(LaunchConfiguration('camera'), value_type=bool)}]),
        OpaqueFunction(function=_description),
        # Your strategy: real clock, waits for /start (the button of the board).
        Node(package=LaunchConfiguration('strategy'), executable=LaunchConfiguration('executable'), name='strategy',
             output='screen', parameters=[{'use_sim_time': False}],
             remappings=[('/scan', '/robotek/scan'), ('scan', '/robotek/scan'),
                         ('/imu', '/robotek/imu'), ('imu', '/robotek/imu'),
                         ('/camera/image', LaunchConfiguration('camera_topic'))]),
    ])
