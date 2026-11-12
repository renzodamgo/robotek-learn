"""Raspberry Pi camera on the CSI ribbon (IMX708, Camera Module 3) -> sensor_msgs/Image, like camera_publisher.

The ROS libcamera driver (camera_ros, libcamera 0.7) does not see cameras on a Raspberry Pi 5; the Raspberry Pi build
of libcamera that rpicam-apps uses does. So this node runs `rpicam-vid` and reads its frames, already processed by the
Pi's ISP, as raw YUV420 from a pipe, then publishes them:

  <topic>              sensor_msgs/Image, bgr8 (default /picam/image_raw)
  <topic>/compressed   sensor_msgs/CompressedImage, JPEG, only while someone subscribes (RViz over Wi-Fi)

By default it uses the whole sensor (mode 2304:1296, the widest field; up to 56 fps) scaled to 640x360 at 30 fps.
With active_states (e.g. 'WAIT_START,CALIBRATE,DETECT_DIRECTION' for the Open) the camera runs only while
/robotek/state is one of them (or not published yet): the Open needs it only to choose the direction, and after Start
rpicam is stopped to save the battery (4 nov 2026). It starts again when the state is one of them again.
Measured on 4 nov 2026 against the USB camera: about 1.8 times the horizontal field, 56 fps against 13, and about a
third of the CPU and energy per frame.

  ros2 run robotek_hardware pi_camera_publisher --ros-args -p width:=640 -p height:=360 -p fps:=30.0
"""
import os
import select
import shutil
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


FRAME_TIMEOUT = 3.0     # s without a frame: rpicam is restarted (4 nov 2026: a loose ribbon gave "Camera frontend
                        # has timed out" and rpicam-vid hung without frames; the Open waited 15 s and failed)
LOG = '/tmp/pi_camera_rpicam.log'


def _die_with_parent():
    """rpicam-vid gets SIGTERM when this node dies, however it dies (12 nov 2026: a node killed by a timeout left
    rpicam running, and the next launch could not open the camera: 'failed to acquire camera')."""
    try:
        import ctypes
        import signal
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG
    except OSError:
        pass


class PiCameraPublisher(Node):
    def __init__(self):
        super().__init__('pi_camera_publisher')
        p = {name: self.declare_parameter(name, value).value for name, value in (
            ('width', 640),
            ('height', 360),
            ('fps', 30.0),
            ('mode', '2304:1296:10:P'),     # the full-sensor mode: the widest field ('' = let rpicam choose)
            ('rotation', 0),               # 0 or 180 (the camera mounted upside down)
            ('topic', '/picam/image_raw'),
            ('frame_id', 'camera'),
            ('jpeg_quality', 70),
            ('extra_args', ''),            # more rpicam-vid options, e.g. '--shutter 8000'
            ('active_states', ''))}        # comma-separated /robotek/state values to run in ('' = always)
        self.p = p
        self.width, self.height = int(p['width']), int(p['height'])
        if self.width % 64 or self.height % 2:
            self.get_logger().warning('width should be a multiple of 64 and height even: rpicam may pad the rows.')
        self.frame_bytes = self.width * self.height * 3 // 2
        self.image_pub = self.create_publisher(Image, p['topic'], 2)
        self.jpeg_pub = self.create_publisher(CompressedImage, p['topic'] + '/compressed', 2)
        self.process = None
        self.running = True
        self.active_states = {state.strip() for state in str(p['active_states']).split(',') if state.strip()}
        self.state = ''
        if self.active_states:
            self.create_subscription(String, '/robotek/state', self.on_state,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.frames, self.rate_since = 0, time.monotonic()
        if shutil.which('rpicam-vid') is None:
            self.get_logger().error('rpicam-vid not found: install rpicam-apps (Raspberry Pi camera tools).')
            return
        threading.Thread(target=self.capture_loop, daemon=True).start()
        self.create_timer(10.0, self.report_rate)

    def on_state(self, msg):
        was = self.active()
        self.state = msg.data
        if self.active() != was:
            self.get_logger().info(f'Pi camera {"on" if self.active() else "off"} (state {self.state}).')

    def active(self):
        return not self.active_states or not self.state or self.state in self.active_states

    def command(self):
        p = self.p
        cmd = ['rpicam-vid', '-t', '0', '-n', '--codec', 'yuv420', '-o', '-',
               '--width', str(self.width), '--height', str(self.height), '--framerate', str(float(p['fps']))]
        if p['mode']:
            cmd += ['--mode', str(p['mode'])]
        if int(p['rotation']):
            cmd += ['--rotation', str(int(p['rotation']))]
        cmd += str(p['extra_args']).split()
        return cmd

    @staticmethod
    def camera_env():
        """The environment without ROS's library paths: /opt/ros/jazzy/lib has the ROS libcamera (0.7), which does not
        see the Pi 5's cameras, and rpicam-vid would load it instead of the Raspberry Pi one (4 nov 2026)."""
        env = dict(os.environ)
        paths = [d for d in env.get('LD_LIBRARY_PATH', '').split(':') if d and '/opt/ros/' not in d]
        env['LD_LIBRARY_PATH'] = ':'.join(paths)
        return env

    def capture_loop(self):
        """Run rpicam-vid and publish every frame; start it again if it stops (camera busy, cable)."""
        while self.running and rclpy.ok():
            if not self.active():
                time.sleep(0.2)
                continue
            cmd = self.command()
            self.get_logger().info('Pi camera: ' + ' '.join(cmd))
            with open(LOG, 'w') as log:
                self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=log, bufsize=0,
                                                env=self.camera_env(), preexec_fn=_die_with_parent)
            fd = self.process.stdout.fileno()
            while self.running and self.active():
                data = self.read_frame(fd)
                if data is None:
                    break
                self.publish(data)
            if self.process.poll() is None:
                self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:      # a hung rpicam would block the restart for good
                self.process.kill()
                self.process.wait()
            if self.running and self.active():
                self.get_logger().warning(f'Pi camera stopped ({self.rpicam_error()}); retrying in 2 s. Busy, or the '
                                          'ribbon (rpicam-hello --list-cameras).', throttle_duration_sec=30.)
                time.sleep(2.0)

    def read_frame(self, fd):
        """One whole frame from the pipe, or None if rpicam ended or gave nothing for FRAME_TIMEOUT s."""
        chunks, missing = [], self.frame_bytes
        while missing:
            if not select.select([fd], [], [], FRAME_TIMEOUT)[0]:
                self.get_logger().error(f'Pi camera: no frame in {FRAME_TIMEOUT:.0f} s ({self.rpicam_error()}); '
                                        'restarting rpicam.', throttle_duration_sec=10.)
                return None
            chunk = os.read(fd, missing)
            if not chunk:
                return None
            chunks.append(chunk)
            missing -= len(chunk)
        return b''.join(chunks)

    @staticmethod
    def rpicam_error():
        """The last ERROR line of rpicam-vid (its output goes to LOG)."""
        try:
            with open(LOG) as log:
                errors = [line.strip() for line in log if 'ERROR' in line]
        except OSError:
            return 'no rpicam log'
        return errors[-1][-160:] if errors else 'no error from rpicam'

    def publish(self, data):
        yuv = np.frombuffer(data, np.uint8).reshape(self.height * 3 // 2, self.width)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, str(self.p['frame_id'])
        msg.height, msg.width, msg.encoding, msg.is_bigendian = self.height, self.width, 'bgr8', 0
        msg.step = self.width * 3
        msg.data = bgr.tobytes()
        self.image_pub.publish(msg)
        if self.jpeg_pub.get_subscription_count():
            ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.p['jpeg_quality'])])
            if ok:
                out = CompressedImage(format='jpeg', data=jpeg.tobytes())
                out.header = msg.header
                self.jpeg_pub.publish(out)
        self.frames += 1

    def report_rate(self):
        now = time.monotonic()
        if not self.active():
            self.frames, self.rate_since = 0, now
            return
        rate = self.frames / (now - self.rate_since)
        self.frames, self.rate_since = 0, now
        self.get_logger().info(f'Pi camera: {rate:.1f} frames/s at {self.width}x{self.height}.',
                               throttle_duration_sec=60.)

    def stop(self):
        self.running = False
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = PiCameraPublisher()
    stop = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.append(True))
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
