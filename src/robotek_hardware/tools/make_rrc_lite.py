#!/usr/bin/env python3
"""Make rrc_lite_driver and rrc_lite_msgs from the public Hiwonder MentorPi repository.

The code of the RRC Lite board driver belongs to Hiwonder and its repository declares no licence, so it is not
copied into this repository. This script takes YOUR clone of MentorPi and writes, next to it, the two small
changes the course needs (they are ours, a few lines each):

  * names: ros_robot_controller -> rrc_lite_driver (node ``rrc_driver``), ros_robot_controller_msgs -> rrc_lite_msgs,
    topics ``~/imu_raw`` -> ``/rrc/imu_raw`` (the names robotek_hardware listens to);
  * the serial port: /dev/ttyACM0 (upstream expects a /dev/rrc symlink that only Hiwonder's image has);
  * the steering servo: message ``SetAckerServoState`` (position in microseconds, duration in seconds) on
    ``/rrc/acker_servo/set_state``, which moves PWM servo 2 of the board (plug the steering servo in port 2).

    git clone https://github.com/Hiwonder/MentorPi.git ~/MentorPi && git -C ~/MentorPi checkout fb6d9969
    python3 make_rrc_lite.py ~/MentorPi ~/ros2_ws/src
"""
import argparse
import shutil
import sys
from pathlib import Path

MENTORPI_COMMIT = 'fb6d9969'
OLD_DRIVER, OLD_MSGS, OLD_NODE = 'ros_robot_controller', 'ros_robot_controller_msgs', 'ros_robot_controller_node'
NEW_DRIVER, NEW_MSGS, NEW_NODE = 'rrc_lite_driver', 'rrc_lite_msgs', 'rrc_driver'


def rename_names(text):
    """Order matters: the longest names first."""
    text = text.replace(OLD_MSGS, NEW_MSGS).replace(OLD_NODE, NEW_NODE)
    text = text.replace("RosRobotController('ros_robot_controller')", "RosRobotController('rrc_driver')")
    return text.replace(OLD_DRIVER, NEW_DRIVER)


def replace_once(text, old, new, where):
    if old not in text:
        sys.exit('make_rrc_lite: did not find %r in %s: is this MentorPi commit %s?' % (old, where, MENTORPI_COMMIT))
    return text.replace(old, new, 1)


def make_msgs(source, out):
    target = out / NEW_MSGS
    shutil.copytree(source / 'driver' / OLD_MSGS, target, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
    (target / 'msg' / 'SetAckerServoState.msg').write_text('float64 duration\nuint16 position\n')
    for path in list((target / 'msg').glob('*.msg')) + list((target / 'srv').glob('*.srv')):
        path.write_text(path.read_text().replace(OLD_MSGS, NEW_MSGS))
    for name in ('CMakeLists.txt', 'package.xml'):
        text = (target / name).read_text().replace(OLD_MSGS, NEW_MSGS)
        if name == 'CMakeLists.txt':
            text = replace_once(text, '"msg/SetPWMServoState.msg"',
                                '"msg/SetPWMServoState.msg"\n    "msg/SetAckerServoState.msg"', name)
        (target / name).write_text(text)
    return target


def make_driver(source, out):
    old = source / 'driver' / OLD_DRIVER
    target = out / NEW_DRIVER
    shutil.copytree(old, target, dirs_exist_ok=True, ignore=shutil.ignore_patterns('__pycache__', 'test'))
    package = target / OLD_DRIVER
    package.rename(target / NEW_DRIVER)
    package = target / NEW_DRIVER
    (package / (OLD_NODE + '.py')).rename(package / (NEW_NODE + '.py'))
    (package / (OLD_DRIVER + '_sdk.py')).rename(package / (NEW_DRIVER + '_sdk.py'))
    (target / 'resource' / OLD_DRIVER).rename(target / 'resource' / NEW_DRIVER)
    (target / 'launch' / (OLD_DRIVER + '.launch.py')).rename(target / 'launch' / 'controller.launch.py')
    for path in [p for p in target.rglob('*') if p.is_file() and p.suffix in ('.py', '.xml', '.cfg', '.sh', '')]:
        if path.name == 'resource' or path.parent.name == 'resource' or path.name.startswith('99-'):
            continue
        text = path.read_text(errors='replace')
        new = rename_names(text)
        if new != text:
            path.write_text(new)
    # setup.py: the executable is called rrc_driver
    setup = target / 'setup.py'
    setup.write_text(replace_once(setup.read_text(), "'rrc_lite_driver = rrc_lite_driver.rrc_driver:main'",
                                  "'rrc_driver = rrc_lite_driver.rrc_driver:main'", 'setup.py'))
    launch = target / 'launch' / 'controller.launch.py'
    launch.write_text(launch.read_text().replace("executable='rrc_lite_driver'", "executable='rrc_driver'"))
    # topics: ~/imu_raw -> rrc/imu_raw (a node at the root of the graph, so /rrc/imu_raw)
    node = package / (NEW_NODE + '.py')
    text = node.read_text().replace("'~/", "'rrc/")
    text = replace_once(text, 'RGBStates, PWMServoState', 'RGBStates, PWMServoState, SetAckerServoState', node.name)
    text = replace_once(text, "        self.create_subscription(RGBStates, 'rrc/set_rgb', self.set_rgb_states, 10)",
                        "        self.create_subscription(RGBStates, 'rrc/set_rgb', self.set_rgb_states, 10)\n"
                        "        self.create_subscription(SetAckerServoState, 'rrc/acker_servo/set_state',\n"
                        "                                 self.set_acker_servo_state, 10)", node.name)
    text = replace_once(text, '    def get_pwm_servo_state(self, msg):',
                        '    def set_acker_servo_state(self, msg):\n'
                        '        self.board.pwm_acker_set_position(msg.duration, msg.position)\n\n'
                        '    def get_pwm_servo_state(self, msg):', node.name)
    node.write_text(text)
    sdk = package / (NEW_DRIVER + '_sdk.py')
    text = sdk.read_text()
    text = replace_once(text, '    def pwm_servo_set_position(self, duration, positions):',
                        '    def pwm_acker_set_position(self, duration, position):\n'
                        '        """Steering servo of the car: PWM servo 2, position in microseconds."""\n'
                        '        self.pwm_servo_set_position(duration, [[2, position]])\n\n'
                        '    def pwm_servo_set_position(self, duration, positions):', sdk.name)
    # The upstream default /dev/rrc is a symlink that only Hiwonder's image creates; the board is /dev/ttyACM0.
    text = replace_once(text, 'device="/dev/rrc"', 'device="/dev/ttyACM0"', sdk.name)
    sdk.write_text(text)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mentorpi', type=Path, help='your clone of https://github.com/Hiwonder/MentorPi')
    parser.add_argument('out', type=Path, help='the src/ directory of your workspace')
    args = parser.parse_args()
    for sub in (OLD_DRIVER, OLD_MSGS):
        if not (args.mentorpi / 'driver' / sub).is_dir():
            sys.exit('make_rrc_lite: %s/driver/%s not found' % (args.mentorpi, sub))
    args.out.mkdir(parents=True, exist_ok=True)
    print('written:', make_msgs(args.mentorpi, args.out))
    print('written:', make_driver(args.mentorpi, args.out))


if __name__ == '__main__':
    main()
