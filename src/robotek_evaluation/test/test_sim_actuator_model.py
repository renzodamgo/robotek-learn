import math
from collections import deque

from robotek_evaluation.sim_actuator_model import SimActuatorModel, slew, table_speed, wheel_angle


def test_table_speed_is_identity_without_table_and_interpolates_with_one():
    assert table_speed(.5, [], []) == .5
    cmd, real = [.15, .3, .4, .6], [.18, .27, .3, .42]
    assert math.isclose(table_speed(.3, cmd, real), .27)
    assert math.isclose(table_speed(.5, cmd, real), .36)
    assert math.isclose(table_speed(-.4, cmd, real), -.3)          # odd
    assert math.isclose(table_speed(.8, cmd, real), .54)           # last slope beyond
    assert math.isclose(table_speed(.075, cmd, real), .09)         # from (0, 0)


def test_wheel_angle_with_play():
    assert wheel_angle(.2, 0., 1.) == .2
    play = math.radians(5.)
    assert math.isclose(math.degrees(wheel_angle(math.radians(10.), play, .95)), 4.75)
    assert math.isclose(math.degrees(wheel_angle(math.radians(-25.), play, .95)), -19.)
    assert wheel_angle(math.radians(3.), play, .95) == 0.


def test_slew_uses_accel_up_and_decel_down():
    assert math.isclose(slew(0., 1., 1., .5, .1), .1)
    assert math.isclose(slew(1., 0., 1., .5, .1), .95)


class Pub:
    def __init__(self):
        self.out = []

    def publish(self, msg):
        self.out.append((msg.linear.x, msg.angular.z))


def model(**params):
    n = SimActuatorModel.__new__(SimActuatorModel)
    n.p = dict(rate_hz=100., wheelbase=.13, steer_limit=.4363, steer_delay=0., steer_rate=100., steer_tau=0.,
               accel_max=100., decel_max=100., drive_delay=0., drive_tau=0., steer_deadband=0., steer_gain=1.)
    n.p.update(params)
    n.table = ([], [])
    n.steer_table = ([], [], [])
    n.v_cmd = n.steer_cmd = n.v = n.steer = n.v_out = 0.
    n.queue, n.drive_queue, n.last, n.pub, n._t = deque(), deque(), None, Pub(), 0.
    n.now = lambda: n._t
    return n


def test_drive_delay_and_lag_shape_a_speed_step():
    n = model(drive_delay=.06, drive_tau=.15)
    n.v_cmd = .3
    speeds = []
    for k in range(60):
        n._t = k*.01
        n.tick()
        speeds.append(n.pub.out[-1][0])
    assert speeds[5] == 0.                       # still inside the 60 ms dead time
    assert .3*.55 < speeds[6+15] < .3*.7         # ~63 % one tau after the delay
    assert speeds[-1] > .28


def test_scan_dropouts_follow_the_rate():
    import random
    from robotek_evaluation.sim_scan_model import drop_rays
    out = drop_rays([1.]*20000, .043, random.Random(1))
    rate = sum(math.isinf(r) for r in out)/len(out)
    assert abs(rate-.043) < .005
    assert drop_rays([1., 2.], 0., random.Random(1)) == [1., 2.]


def test_real_motor_saturates_at_full_power_in_both_directions():
    for sign in (1., -1.):
        n = model(drive_command_limit=1.28)
        n.table = ([.6, 1.28], [.42, .6])
        n.v_cmd = sign * 4.
        for k in range(100):
            n._t = k * .01
            n.tick()
        assert math.isclose(n.pub.out[-1][0], sign * .6)
        # Match base_adapter: its command ramp precedes the PWM saturation.
        assert math.isclose(n.v, sign * 4.)
        n.p['decel_max'] = .6
        n.v_cmd = 0.
        n._t += .01
        n.tick()
        assert abs(n.v) < 4.
        assert math.isclose(n.pub.out[-1][0], sign * .6)


def test_steering_command_inside_motor_deadband_keeps_motor_stopped():
    n = model(drive_deadband=.005)
    n.v_cmd = .002
    n.queue.append((0., .2))
    for k in range(10):
        n._t = k * .01
        n.tick()
    assert n.steer == .2
    assert n.pub.out[-1][0] == 0.


def test_steer_table_per_side():
    n = model()
    n.steer_table = ([math.radians(10.), math.radians(25.)], [math.radians(8.7), math.radians(19.7)],
                     [math.radians(8.6), math.radians(17.6)])
    assert math.isclose(math.degrees(n.wheel(math.radians(25.))), 19.7)
    assert math.isclose(math.degrees(n.wheel(math.radians(-25.))), -17.6)
    assert math.isclose(math.degrees(n.wheel(math.radians(5.))), 4.35)
    assert n.wheel(0.) == 0.
