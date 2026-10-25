import math

from robotek_course.minisim import Scene
from my_open.turn_gate import TurnGate


def scans(gate, count, front=1.1, island_end=0.2, inner=0.3, travelled=0.5, arc=False):
    """Feed the gate ``count`` different scans (new Scene objects) and return its last answer."""
    answer = None
    for _ in range(count):
        scene = Scene(left=inner, front_axis=front, island_end=island_end)
        answer = gate.update(scene, inner, travelled, arc)
    return answer


def test_all_conditions_met_after_two_scans():
    gate = TurnGate()
    assert scans(gate, 1) is False                    # the island's end must be seen in 2 different scans
    assert scans(gate, 1) is True


def test_the_same_scan_repeated_counts_once():
    gate = TurnGate()
    scene = Scene(left=0.3, front_axis=1.1, island_end=0.2)
    assert [gate.update(scene, 0.3, 0.5, False) for _ in range(5)] == [False] * 5


def test_the_wall_ahead_must_be_near():
    assert scans(TurnGate(), 3, front=1.4) is False


def test_the_island_must_have_ended():
    assert scans(TurnGate(), 3, island_end=math.inf) is False        # the end is not in view
    assert scans(TurnGate(), 3, island_end=0.9) is False             # too far ahead still


def test_an_open_side_is_enough_without_seeing_the_end():
    assert scans(TurnGate(), 1, island_end=math.inf, inner=1.6) is True


def test_a_lost_sighting_restarts_the_count():
    gate = TurnGate()
    scans(gate, 1)
    scans(gate, 1, island_end=math.inf)                              # one scan without it
    assert scans(gate, 1) is False


def test_the_car_must_have_driven_since_the_last_corner():
    assert scans(TurnGate(), 3, travelled=0.1) is False


def test_no_new_corner_while_one_is_in_progress():
    assert scans(TurnGate(), 3, arc=True) is False


def test_after_a_corner_the_gate_stays_shut_until_the_island_is_seen_again():
    gate = TurnGate()
    assert scans(gate, 2) is True
    gate.corner_started()
    assert scans(gate, 3, travelled=0.1, inner=1.6, island_end=math.inf) is False        # still at the corner
    assert scans(gate, 3, travelled=0.5, inner=1.6, island_end=math.inf) is False        # open side, not yet re-armed
    assert scans(gate, 1, travelled=0.5, inner=0.3, island_end=math.inf, front=1.5) is False     # island seen: armed
    assert scans(gate, 2) is True


def test_in_a_narrow_lane_the_lookahead_shrinks():
    gate = TurnGate()
    # the island wall is 0.3 m away: the end must be within 0.22 m, not 0.35 m
    assert scans(gate, 3, island_end=0.30, inner=0.30) is False
    assert scans(gate, 2, island_end=0.20, inner=0.30) is True
