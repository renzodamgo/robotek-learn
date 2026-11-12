import math
from functools import partial

from helpers import run_arc
from my_open.arc import DAMPING, TURN_TOLERANCE, arc_steer, damped_steer

FULL_LOCK = 0.30718
SLOW_SERVO = dict(servo_delay=0.15, servo_tau=0.10)


def deg(radians):
    return math.degrees(radians)


def test_far_from_the_goal_the_wheels_are_at_full_lock_towards_it():
    assert arc_steer(math.pi / 2, 0.0, 0.0) == (FULL_LOCK, False)          # a left corner
    assert arc_steer(-math.pi / 2, 0.0, 0.0) == (-FULL_LOCK, False)        # a right corner
    assert arc_steer(math.pi / 2, 1.0, 1.6) == (FULL_LOCK, False)          # still 0.57 rad to go


def test_the_short_way_round_across_pi():
    assert arc_steer(-2.9, 2.9, 0.0)[0] == FULL_LOCK                       # 0.48 rad to the left, not 5.8 to the right


def test_the_corner_is_done_within_the_tolerance():
    steer, done = arc_steer(math.pi / 2, math.pi / 2 - 0.29, 0.0)
    assert done and 0.0 < steer < FULL_LOCK
    assert arc_steer(math.pi / 2, math.pi / 2 - 0.31, 0.0)[1] is False
    assert arc_steer(math.pi / 2, math.pi / 2 - 0.2, 0.0, turn_tolerance=0.1)[1] is False


def test_the_damping_takes_steering_off_while_the_car_is_turning():
    still = damped_steer(0.2, 0.0, 0.0)
    assert math.isclose(still, 0.8 * 0.2)                                  # the heading law of step 2
    assert math.isclose(damped_steer(0.2, 0.0, 1.0), still - DAMPING * 1.0)
    assert damped_steer(0.2, 0.0, -1.0) > still                            # turning the other way: steer more
    assert damped_steer(0.0, 0.0, 50.0) == -FULL_LOCK                      # never beyond full lock


def test_inside_the_tolerance_the_damped_law_steers():
    assert arc_steer(1.0, 0.9, 0.7)[0] == damped_steer(1.0, 0.9, 0.7)


def test_a_left_corner_is_finished_and_not_overshot():
    result = run_arc(arc_steer)
    assert deg(result['overshoot']) < 2.0
    assert result['t_close'] is not None and result['t_close'] < 1.6


def test_a_right_corner_is_the_mirror_of_a_left_one():
    left, right = run_arc(arc_steer, sign=1), run_arc(arc_steer, sign=-1)
    assert math.isclose(left['t_close'], right['t_close'], abs_tol=0.06)
    assert deg(right['overshoot']) < 2.0 and right['yaw'][-1] < -1.5


def test_the_wheels_do_not_move_beyond_full_lock():
    assert max(abs(s) for s in run_arc(arc_steer)['steer']) <= FULL_LOCK + 1e-9


def test_letting_go_too_late_overshoots():
    # The servo takes time to straighten the wheels: at full lock until 0.05 rad from the goal, the car passes it.
    late = run_arc(partial(arc_steer, turn_tolerance=0.05))
    assert deg(late['overshoot']) > 8.0
    assert deg(run_arc(partial(arc_steer, turn_tolerance=TURN_TOLERANCE))['overshoot']) < 2.0


def test_letting_go_too_early_is_slower():
    early = run_arc(partial(arc_steer, turn_tolerance=0.8))
    assert early['t_close'] > run_arc(arc_steer)['t_close'] + 0.2


def test_with_a_slower_servo_the_damping_is_what_cuts_the_overshoot():
    without = run_arc(partial(arc_steer, damping=0.0), **SLOW_SERVO)
    damped = run_arc(arc_steer, **SLOW_SERVO)
    assert deg(without['overshoot']) > deg(damped['overshoot']) + 1.5
    wide = run_arc(partial(arc_steer, turn_tolerance=0.5), **SLOW_SERVO)          # a slower servo needs to let go earlier
    assert deg(wide['overshoot']) < 2.0
