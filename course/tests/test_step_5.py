import math
from functools import partial

from helpers import jump_with_damping, run_arc
from my_open.arc import arc_steer, ramp

# step 4 of the old guide: the corner with the ramp only
ramp_only = partial(arc_steer, feed=False, damping=0.0)

def test_the_reference_climbs_at_the_given_rate():
    assert math.isclose(ramp(0.0, 1.0, 2.0, 0.1), 0.2)
    assert ramp(0.0, 1.0, 2.0, 1.0) == 1.0                    # arrives and stops: it never passes the goal
    assert ramp(1.0, 1.0, 2.0, 0.1) == 1.0


def test_the_reference_goes_down_or_the_short_way_round():
    assert math.isclose(ramp(0.0, -1.0, 2.0, 0.1), -0.2)
    assert ramp(3.0, -3.0, 1.0, 0.1) > 3.0 or ramp(3.0, -3.0, 1.0, 0.1) < -3.0       # across +-pi, not back through 0


def test_a_left_corner_is_finished_and_not_overshot():
    result = run_arc(ramp_only)
    assert result['t_close'] is not None and result['t_close'] < 1.6
    assert result['overshoot'] < math.radians(3)


def test_the_ramp_alone_also_mirrors_a_right_corner():
    left, right = run_arc(ramp_only, sign=1), run_arc(ramp_only, sign=-1)
    assert math.isclose(left['t_close'], right['t_close'], abs_tol=0.04)
    assert right['yaw'][-1] < -1.4


def test_the_wheels_do_not_move_beyond_full_lock():
    assert max(abs(s) for s in run_arc(ramp_only)['steer']) <= 0.30718 + 1e-9



def deg(radians):
    return math.degrees(radians)


def test_with_the_real_servo_the_final_corner_is_clean_and_not_slower_than_step_4():
    final, plain = run_arc(arc_steer), run_arc(ramp_only)
    assert deg(final['overshoot']) < 2.0
    assert final['t_close'] <= plain['t_close'] + 1e-9


def test_a_slower_servo_overshoots_and_damping_is_what_stops_it():
    slow = dict(servo_delay=0.15, servo_tau=0.10)
    without = run_arc(partial(arc_steer, damping=0.0), **slow)
    with_damping = run_arc(arc_steer, **slow)
    assert deg(without['overshoot']) > 5.0
    assert deg(with_damping['overshoot']) < 2.0


def test_when_the_car_turns_faster_than_expected_the_fade_is_what_stops_the_overshoot():
    no_fade = run_arc(partial(arc_steer, fade=False, damping=0.0), rate_factor=0.7)
    fade = run_arc(partial(arc_steer, damping=0.0), rate_factor=0.7)
    assert deg(no_fade['overshoot']) > 10.0
    assert deg(fade['overshoot']) < 2.0


def test_the_feed_forward_makes_the_corner_faster_than_the_ramp_alone():
    # when the strategy believes the car is slower (rate_factor 0.7) the ramp alone drags behind
    assert run_arc(arc_steer, rate_factor=0.7)['t_close'] < run_arc(ramp_only, rate_factor=0.7)['t_close']


def test_the_ramp_is_not_what_avoids_the_overshoot_but_it_does_not_make_the_corner_slower():
    slow = dict(servo_delay=0.15, servo_tau=0.10)
    jump = run_arc(jump_with_damping, **slow)            # no ramp: the goal is the reference at once, damping against 0
    final = run_arc(arc_steer, **slow)
    assert deg(jump['overshoot']) < 2.0 and deg(final['overshoot']) < 2.0
    assert final['t_close'] <= jump['t_close']


def test_a_right_corner_is_the_mirror_of_a_left_one():
    left = run_arc(arc_steer, sign=1)
    right = run_arc(partial(arc_steer, sign=-1), sign=-1)
    assert math.isclose(left['t_close'], right['t_close'], abs_tol=0.04)
    assert deg(right['overshoot']) < 2.0 and right['yaw'][-1] < -1.4
