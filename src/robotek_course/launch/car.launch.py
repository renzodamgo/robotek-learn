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
  GPIO 16 button  -> gpio_button  -> /robotek/gpio_button (true = pressed; only reported for now)
  VL53L0X (I2C)   -> rear_range   -> /robotek/rear_range (m) and /robotek/rear_range_mm (only reported for now)
  your /cmd_vel_raw -> base_adapter -> servo (microseconds) on the RRC Lite + /motor_vel -> motor_driver (TB6612)

Started here (each driver is skipped with a message when its package is not installed, so the launch can be tried
on a laptop; see src/robotek_hardware/README.md for how to install them):
  - rrc_lite_driver (the RRC Lite board: IMU, button, servo, RGBs) and the LiDAR driver (ldlidar_stl_ros2, LD19)
  - the camera: robotek_hardware/camera_publisher (camera_driver:=usb_cam for the ROS package)
  - robotek_hardware: scan_adapter, imu_adapter, motor_driver, base_adapter, start_button, gpio_button, rear_range, battery_guard, status_leds
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(package):
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return None


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _picam_states(context):
    """picam_active_states, or for the Open by default only until the direction is chosen (the camera is not used
    after that; stopping it saves the battery)."""
    states = _arg(context, 'picam_active_states')
    if states == 'auto':
        return 'WAIT_START,CALIBRATE,DETECT_DIRECTION' if _arg(context, 'strategy') == 'robotek_open' else ''
    return states


def _picam_size(context):
    """picam_width, picam_height, picam_fps; 'auto' = 1280x720 at 10 fps for the Open (its corner lines are ~1.5 m
    away and 1 px wide at 640x360) and 640x360 at 30 fps otherwise (the Obstacle's signs are large and need fresh
    frames while driving)."""
    is_open = _arg(context, 'strategy') == 'robotek_open'
    auto = dict(picam_width='1280' if is_open else '640', picam_height='720' if is_open else '360',
                picam_fps='10.0' if is_open else '30.0')
    return [_arg(context, name) if _arg(context, name) != 'auto' else value for name, value in auto.items()]


def _camera_topic(context):
    """camera_topic, or by default the topic of the chosen camera driver (picam: /picam/image_raw)."""
    topic = _arg(context, 'camera_topic')
    if topic:
        return topic
    return '/picam/image_raw' if _arg(context, 'camera_driver') == 'picam' else '/usb_cam/image_raw'


def _reporter_node(context):
    """debug:=true: the live report of the round, on the same camera as the strategy."""
    return [
    Node(package='robotek_course', executable='round_reporter', name='round_reporter', output='screen',
         condition=IfCondition(LaunchConfiguration('debug')),
         additional_env={'ROBOTEK_CAMERA_BOTTOM_CROP': LaunchConfiguration('camera_bottom_crop'),
                         'ROBOTEK_CAMERA_BLUE': LaunchConfiguration('camera_blue_hsv'),
                         'ROBOTEK_CAMERA_ORANGE': LaunchConfiguration('camera_orange_hsv'),
                         'ROBOTEK_CAMERA_WALL_MASK': PythonExpression(
                             ["'1' if '", LaunchConfiguration('camera_wall_mask'), "'.lower() == 'true' else '0'"])},
         remappings=[('/imu', '/robotek/imu'), ('imu', '/robotek/imu'),
                     ('/camera/image', _camera_topic(context))])]


def _node_params_file(values, node='strategy'):
    """A temporary params file with ``values`` under the node's own name (beats a params file's node entry)."""
    import tempfile
    import yaml
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='car_launch_', delete=False)
    yaml.safe_dump({node: {'ros__parameters': values}}, handle)
    handle.close()
    return handle.name


def _strategy_node(context):
    """Your node; strategy_params:=<file.yaml> adds its ROS parameters (e.g. inner_gap: 0.35 for the Open v5)."""
    parameters = [{'use_sim_time': False}]
    extra = _arg(context, 'strategy_params')
    if extra == 'auto':
        # The team's Obstacle v3 with the IMX708: its car file (camera, colours, 17.5 deg; 8 nov 2026: DONE 85.0 s).
        extra = ''
        if _arg(context, 'strategy') in ('robotek_obstacle_v3', 'robotek_open') and _arg(context, 'camera_driver') == 'picam':
            # the Open v5's: 3 laps in 23.4 s (11 nov 2026, record)
            extra = os.path.join(get_package_share_directory(_arg(context, 'strategy')), 'config', 'car_picam.yaml')
    if extra:
        path = os.path.expanduser(extra)
        if not os.path.isfile(path):
            raise RuntimeError(f'strategy_params: no such file {path}')
        parameters.append(path)
    if _arg(context, 'strategy') == 'robotek_obstacle_v3':
        parameters.append({'recover_back': float(_arg(context, 'recover_back'))})
    if _arg(context, 'strategy') == 'robotek_open':
        # Open v5 overrides from the command line. As a file keyed by the node name: a plain dict goes in as '/**',
        # which the params file's 'strategy:' entry overrides whatever the order (11 nov 2026: corner_lookahead:=0.55
        # stayed 0.45).
        values = {name: float(_arg(context, name)) for name in ('corner_lookahead', 'corner_speed', 'cruise_speed', 'brake_margin', 'turn_tolerance', 'lateral_kp', 'inner_gap')
                  if _arg(context, name)}
        if _arg(context, 'detect_crash'):
            values['detect_crash'] = _arg(context, 'detect_crash').lower() == 'true'
        if values:
            parameters.append(_node_params_file(values))
    if _arg(context, 'start_lot').lower() == 'true':
        # Obstacle v3: start inside the parking lot, leave it with the coach's pulses, then the laps (lot_exit.py).
        parameters.append({'start_lot': 1.0, 'lot_exit_speed': float(_arg(context, 'lot_exit_speed')),
                           'lot_exit_steps': _arg(context, 'lot_exit_steps')})
    return [
        Node(package=LaunchConfiguration('strategy'), executable=LaunchConfiguration('executable'), name='strategy',
             output='screen', parameters=parameters,
             # Open v5: when it ends (after a CRASH, its file written and the board reset) the whole launch ends.
             on_exit=[Shutdown(reason='strategy ended')] if _arg(context, 'strategy') == 'robotek_open' else None,
             condition=UnlessCondition(LaunchConfiguration('teleop')),
             # robotek_course.vision on the car: ignore the bottom of the image (the LiDAR) and look only inside the walls.
             additional_env={'ROBOTEK_CAMERA_BOTTOM_CROP': LaunchConfiguration('camera_bottom_crop'),
                             'ROBOTEK_CAMERA_BLUE': LaunchConfiguration('camera_blue_hsv'),
                             'ROBOTEK_CAMERA_ORANGE': LaunchConfiguration('camera_orange_hsv'),
                             'ROBOTEK_CAMERA_WALL_MASK': PythonExpression(
                                 ["'1' if '", LaunchConfiguration('camera_wall_mask'), "'.lower() == 'true' else '0'"])},
             remappings=[('/scan', '/robotek/scan'), ('scan', '/robotek/scan'),
                         ('/imu', '/robotek/imu'), ('imu', '/robotek/imu'),
                         ('/camera/image', _camera_topic(context))])]


def _check_strategy(context):
    """Before starting anything: is the strategy package built? If not, say how to get one and exit with code 1."""
    if _arg(context, 'teleop').lower() == 'true':
        return []
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
                                           'the camera must publish ' + _camera_topic(context)))
        elif _arg(context, 'camera_driver') == 'picam':
            # The Raspberry Pi camera on the CSI ribbon (IMX708): full-sensor field (size and rate: _picam_size).
            actions.append(Node(package='robotek_hardware', executable='pi_camera_publisher',
                                name='pi_camera_publisher', output='screen',
                                parameters=[{'topic': _camera_topic(context),
                                             'rotation': int(_arg(context, 'picam_rotation')),
                                             'width': int(_picam_size(context)[0]),
                                             'height': int(_picam_size(context)[1]),
                                             'fps': float(_picam_size(context)[2]),
                                             'active_states': _picam_states(context)}]))
        else:
            # robotek_hardware/camera_publisher: finds the camera by itself and keeps retrying if it is missing.
            actions.append(Node(package='robotek_hardware', executable='camera_publisher', name='camera_publisher',
                                output='screen', parameters=[
                                    os.path.join(get_package_share_directory('robotek_hardware'), 'config', 'robot.yaml'),
                                    {'device': device}]))
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
        base = yaml.safe_load(stream)['base_adapter']['ros__parameters']
        default_max_pct = str(base['motor_max_pct'])
        default_max_rps = str(base.get('motor_max_rps', 0.0))
        default_rps_per_mps = str(base.get('motor_rps_per_mps', 0.0))
        default_min_rps = str(base.get('motor_min_rps', 0.0))
        motor_backend = str(base.get('motor_backend', 'tb6612'))
        brake_defaults = {name: str(base.get(name, value)) for name, value in (
            ('brake_rps', 0.0), ('brake_hz', 10.0), ('brake_s', 1.5), ('brake_min_rps', 1.0))}
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
        DeclareLaunchArgument('camera_driver', default_value='picam',
                              description="'camera_publisher' (USB camera, robotek_hardware), 'usb_cam' (the ROS "
                                          "package) or 'picam' (the Raspberry Pi camera on the CSI ribbon, IMX708)"),
        DeclareLaunchArgument('picam_rotation', default_value='180',
                              description='picam: 180 = mounted upside down (the team car), 0 = upright'),
        # auto: 1280x720 at 10 fps for the Open (from the start its corner lines are ~1.5 m away and only 1 px wide at
        # 640x360, 4 nov 2026), 640x360 at 30 fps for other strategies (the Obstacle).
        DeclareLaunchArgument('picam_width', default_value='auto'),
        DeclareLaunchArgument('picam_height', default_value='auto'),
        DeclareLaunchArgument('picam_fps', default_value='auto'),
        DeclareLaunchArgument('picam_active_states', default_value='auto',
                              description="picam: /robotek/state values the camera runs in ('' = always; auto = "
                                          "the Open only until the direction is chosen)"),
        DeclareLaunchArgument('video_device', default_value='',
                              description='your camera; empty = the first /dev/v4l/by-id/*-video-index0, else '
                                          '/dev/video0. A stable by-id path is better than /dev/videoN'),
        DeclareLaunchArgument('camera_topic', default_value='',
                              description='the camera image for the strategy; empty = that of camera_driver '
                                          '(/usb_cam/image_raw, or /picam/image_raw with picam)'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true: no strategy, drive with keyboard_drive (hold:=true) from another terminal'),
        DeclareLaunchArgument('strategy_params', default_value='auto',
                              description='optional YAML file with ROS parameters for your node (strategy: ros__parameters: ...); '
                                          'auto = <strategy>/config/car_picam.yaml for the Obstacle v3 or the Open with the picam, '
                                          "else none; '' = none"),
        DeclareLaunchArgument('debug', default_value='false',
                              description='true: report states, direction, corners, laps and lines in this terminal'),
        DeclareLaunchArgument('button', default_value='true',
                              description='false: start by itself after start_delay s, without the button (tests only)'),
        DeclareLaunchArgument('gpio_button', default_value='true',
                              description='push button on a Pi GPIO -> /robotek/gpio_button (Bool, true = pressed); '
                                          'only reports for now, it does not start or stop the round'),
        DeclareLaunchArgument('gpio_button_pin', default_value='16',
                              description='BCM GPIO of the push button (other leg on GND, internal pull-up)'),
        DeclareLaunchArgument('rear_range', default_value='auto',
                              description='VL53L0X distance sensor on the Pi I2C bus -> /robotek/rear_range (Range, m) '
                                          'and /robotek/rear_range_mm (Int32, mm). auto = only for the Obstacle v3 (the lot '
                                          'exit); the Open does not use it (11 nov 2026: its I2C errors during Open rounds)'),
        DeclareLaunchArgument('start_delay', default_value='auto',
                              description='seconds from launch to the automatic start with button:=false; auto = 20 for '
                                          'the Obstacle v3 (ready only after 13-17 s), else 12'),
        DeclareLaunchArgument('camera_bottom_crop', default_value='0.12',
                              description='share of image rows at the bottom the line detector ignores (the LiDAR is in view)'),
        DeclareLaunchArgument('camera_blue_hsv', default_value='95,135,15,35,125',
                              description='blue line h_min,h_max,s_min,v_min,v_max under the room light (1 nov 2026)'),
        DeclareLaunchArgument('camera_orange_hsv', default_value='155,10,25,35,255',
                              description='orange line (looks red: hue wraps through 0), same format'),
        DeclareLaunchArgument('camera_wall_mask', default_value='true',
                              description='line detector only looks inside the black walls'),
        DeclareLaunchArgument('motor_rps_per_mps', default_value='auto',
                              description='motor rps per m/s of the car (depends on the gears); auto = robot.yaml '
                                          '(%s, 54:30 gears). 1:1 gears: ~6.8 (12 nov 2026: the 54:30 value made the '
                                          'car ~35 %% slower)' % default_rps_per_mps),
        DeclareLaunchArgument('motor_max_rps', default_value='auto',
                              description='encoder motor cap in rps; auto = 4 for the Obstacle v3 (8 nov 2026: DONE 85.0 s), 6.7 for the '
                                          'Open (11 nov 2026: 3 laps in 23.4 s), '
                                          'else robot.yaml (%s)' % default_max_rps),
        DeclareLaunchArgument('motor_min_rps', default_value=default_min_rps,
                              description='encoder motor floor in rps while moving (robot.yaml; 0 = off). The 310 motor '
                                          'does not start the car at ~1 rps: 1.5 for the Obstacle v3'),
        DeclareLaunchArgument('corner_lookahead', default_value='',
                              description='Open v5: m of look-ahead to the island end that starts a turn; higher = turns '
                                          'earlier ("" = the params file, 0.35 on the car; capped at the island distance - 0.08 m)'),
        DeclareLaunchArgument('corner_speed', default_value='',
                              description='Open v5: m/s in every corner (it slows down to it on the '
                                          'approach); "" = the params file (0.64 on the car)'),
        DeclareLaunchArgument('turn_tolerance', default_value='',
                              description='Open v5: rad short of 90 deg where a corner ends; higher = shorter turn ("" = '
                                          'the params file, 0.30 on the car)'),
        DeclareLaunchArgument('lateral_kp', default_value='',
                              description='Open v5: rad of heading per m off inner_gap on the straights ("" = params file, '
                                          '1.0 on the car)'),
        DeclareLaunchArgument('detect_crash', default_value='',
                              description='Open v5: true = stop, reset the board and save the last 5 s on a crash '
                                          '("" = params file: off)'),
        DeclareLaunchArgument('inner_gap', default_value='',
                              description='Open v5: m kept from the island on the straights ("" = params file, 0.35 on the car)'),
        DeclareLaunchArgument('brake_margin', default_value='',
                              description='Open v5: m; higher = slows down for a corner earlier ("" = the params file, '
                                          '0.35 on the car)'),
        DeclareLaunchArgument('cruise_speed', default_value='',
                              description='Open v5: m/s on the straights; "" = the params file (1.02 on the car)'),
        DeclareLaunchArgument('done_reset', default_value='auto',
                              description='base_adapter: states that send 0 and then reset the RRC board to stop the motor '
                                          '(auto = DONE and CRASH for the Open; none = off)'),
        DeclareLaunchArgument('board_button', default_value='false',
                              description="also start/stop with the RRC Lite's button (the GPIO 16 button is the one used "
                                          'since 10 nov 2026)'),
        DeclareLaunchArgument('start_lot', default_value='false',
                              description='Obstacle v3: the car starts inside the parking lot (against the wall, ~1 cm '
                                          'from the rear limit): direction from the LiDAR sides, four short full-lock '
                                          'pulses out (left fwd, right back, left fwd, right fwd; mirrored in CW), then '
                                          'the laps. The motor creeps with a kick and pulses only during LOT_EXIT'),
        DeclareLaunchArgument('recover_back', default_value='0.5',
                              description='Obstacle v3: s of reverse at full lock in the front recover (right in CCW, '
                                          'left in CW)'),
        DeclareLaunchArgument('lot_exit_speed', default_value='0.9',
                              description='m/s of the lot exit pulses (first full car round from the lot, 9 nov 2026: 0.9)'),
        DeclareLaunchArgument('lot_exit_steps', default_value='L+0.45 R-0.55 L+0.45 R-0.55 L+0.45 R-0.55 L+0.78 C-0.4',
                              description="the lot exit's pulses: <L|R|C><+|-><seconds>, L/R full lock (L = away from the wall, "
                                          'mirrored in CW), C wheels straight, + forward, - reverse; then the laps'),
        DeclareLaunchArgument('motor_kick_rps', default_value='0.0',
                              description='start kick: at least this many rps for motor_kick_s from a standstill (0 = off)'),
        DeclareLaunchArgument('motor_kick_s', default_value='0.2'),
        DeclareLaunchArgument('motor_pulse_rps', default_value='0.0',
                              description='requests below this many rps become pulses of it with the same mean (0 = off)'),
        DeclareLaunchArgument('motor_pulse_period_s', default_value='0.5'),
        # Counter-pulse brake (8 nov 2026 floor test at 1.2 rps: 2 rps reverse for 0.15 s cut the roll at full lock from
        # 0.18 to 0.05 rad): brake_rps:=2 brake_s:=0.15 brake_hz:=3 (one reverse pulse: half period > brake_s).
        *[DeclareLaunchArgument(name, default_value=value,
                                description='base_adapter brake (robot.yaml; brake_rps 0 = off): +-brake_rps at brake_hz '
                                            'for brake_s, reverse first, when stopping from at least brake_min_rps')
          for name, value in brake_defaults.items()],
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
        # The TB6612 bridge only with motor_backend tb6612 (robot.yaml): the car's motor is on the RRC Lite (M2).
        Node(package='robotek_hardware', executable='motor_driver', name='tb6612fng_motor', output='screen',
             parameters=[hardware_config], condition=IfCondition(str(motor_backend == 'tb6612'))),
        # Your /cmd_vel_raw is the base_adapter's input (in the simulator sim_actuator_model reads the same topic).
        Node(package='robotek_hardware', executable='base_adapter', name='base_adapter', output='screen',
             parameters=[hardware_config, {'motor_max_pct': ParameterValue(LaunchConfiguration('motor_max_pct'),
                                                                           value_type=float),
                                           'motor_rps_per_mps': ParameterValue(PythonExpression(
                                               ["(", default_rps_per_mps, ") if '", LaunchConfiguration('motor_rps_per_mps'),
                                                "' == 'auto' else ", LaunchConfiguration('motor_rps_per_mps')]),
                                               value_type=float),
                                           'motor_max_rps': ParameterValue(PythonExpression(
                                               ["(4.0 if '", LaunchConfiguration('strategy'), "' == 'robotek_obstacle_v3' else 6.7 if '",
                                                LaunchConfiguration('strategy'), "' == 'robotek_open' else ",
                                                default_max_rps, ") if '", LaunchConfiguration('motor_max_rps'),
                                                "' == 'auto' else ", LaunchConfiguration('motor_max_rps')]), value_type=float),
                                           # start_lot: 1.5 rps floor unless set (the 310 motor does not start the car
                                           # lower down; the first full round from the lot, 9 nov 2026)
                                           'motor_min_rps': ParameterValue(PythonExpression(
                                               ["(1.5 if '", LaunchConfiguration('start_lot'), "'.lower() == 'true' and float('",
                                                LaunchConfiguration('motor_min_rps'), "') == 0.0 else ",
                                                LaunchConfiguration('motor_min_rps'), ")"]), value_type=float),
                                           **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                                              for name in ('brake_rps', 'brake_hz', 'brake_s', 'brake_min_rps')},
                                           # start_lot: kick and pulses of 1.8 rps (with kick 0.15 s and period 0.4 s
                                           # the car crept 5-6 cm/s for 0.06 m/s asked, 9 nov 2026), only in LOT_EXIT,
                                           # unless set by hand.
                                           **{name: ParameterValue(PythonExpression(
                                               ["(", lot_value, " if '", LaunchConfiguration('start_lot'),
                                                "'.lower() == 'true' and float('", LaunchConfiguration(name),
                                                "') == 0.0 else ", LaunchConfiguration(name), ")"]), value_type=float)
                                              for name, lot_value in (('motor_kick_rps', '1.8'), ('motor_pulse_rps', '1.8'))},
                                           **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                                              for name in ('motor_kick_s', 'motor_pulse_period_s')},
                                           # done_reset:=auto -> DONE for the Open (its motor ran on after the
                                           # final 0, 11 nov 2026); '' = never; or a list of states.
                                           'done_reset_states': ParameterValue(PythonExpression(
                                               ["('DONE,CRASH' if '", LaunchConfiguration('strategy'), "' == 'robotek_open' else '') if '",
                                                LaunchConfiguration('done_reset'), "' == 'auto' else ('' if '",
                                                LaunchConfiguration('done_reset'), "' == 'none' else '",
                                                LaunchConfiguration('done_reset'), "')"]), value_type=str),
                                           # No decel_limit ramp at the end of the Open: a 0 right after the last
                                           # corner rolled ~34 cm at full lock into the island with the ramp (the
                                           # coach: brake straight to 0; 12 nov 2026 lab), and the lot exit's pulses.
                                           'no_ramp_states': ParameterValue(PythonExpression(
                                               ["'LOT_EXIT' if '", LaunchConfiguration('start_lot'),
                                                "'.lower() == 'true' else ('FINISH,DONE,CRASH,OPERATOR_STOP' if '",
                                                LaunchConfiguration('strategy'), "' == 'robotek_open' else '')"]),
                                               value_type=str),
                                           'low_speed_states': ParameterValue(PythonExpression(
                                               ["'LOT_EXIT' if '", LaunchConfiguration('start_lot'),
                                                "'.lower() == 'true' else ''"]), value_type=str)}],
             remappings=[('cmd_vel', '/cmd_vel_raw')]),
        Node(package='robotek_hardware', executable='start_button', name='start_button', output='screen',
             remappings=[('start', '/start')],
             # button:=false (quick tests): start by itself after start_delay s; the first press is then the stop.
             # The press comes from the GPIO 16 button; board_button:=true adds the RRC Lite's (10 nov 2026: not used).
             parameters=[{'board_button': ParameterValue(LaunchConfiguration('board_button'), value_type=bool),
                          'gpio_button': ParameterValue(LaunchConfiguration('gpio_button'), value_type=bool),
                          'auto_start_delay': ParameterValue(PythonExpression(
                 ["0.0 if '", LaunchConfiguration('button'), "'.lower() == 'true' else ((20.0 if '",
                  LaunchConfiguration('strategy'), "' == 'robotek_obstacle_v3' else 12.0) if '",
                  LaunchConfiguration('start_delay'), "' == 'auto' else ", LaunchConfiguration('start_delay'), ")"]),
                 value_type=float)}]),
        Node(package='robotek_hardware', executable='gpio_button', name='gpio_button', output='screen',
             condition=IfCondition(LaunchConfiguration('gpio_button')),
             parameters=[{'pin': ParameterValue(LaunchConfiguration('gpio_button_pin'), value_type=int)}]),
        Node(package='robotek_hardware', executable='rear_range', name='rear_range', output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('rear_range'), "'.lower() == 'true' or ('", LaunchConfiguration('rear_range'),
                  "' == 'auto' and '", LaunchConfiguration('strategy'), "' == 'robotek_obstacle_v3')"]))),
        Node(package='robotek_hardware', executable='battery_guard', name='battery_guard', output='screen',
             condition=IfCondition(LaunchConfiguration('battery_guard')),
             parameters=[{'min_voltage': ParameterValue(LaunchConfiguration('battery_min_v'), value_type=float)}]),
        # Green = ready for the button, blue = pressed (the round runs), red blinking = battery low.
        Node(package='robotek_hardware', executable='status_leds', name='status_leds', output='screen',
             condition=IfCondition(LaunchConfiguration('status_leds'))),
        OpaqueFunction(function=_description),
        # debug:=true: live report of the round (states, direction, corners, laps, lines) in this terminal.
        OpaqueFunction(function=_reporter_node),
        # Your strategy: real clock, waits for /start (the button of the board).
        OpaqueFunction(function=_strategy_node),
    ])
