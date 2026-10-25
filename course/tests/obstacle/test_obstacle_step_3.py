"""Step 3: the signs, by seat (my_obstacle/signs.py)."""
import pytest

from my_obstacle.signs import SECTOR, SWAP, SeatMemory, sign_colour, sign_position

CAR = (-0.25, -1.0, 0.0)        # on the start straight, CCW


def test_a_sign_ahead_is_placed_on_the_lane():
    progress, lateral = sign_position(dict(x=0.75, y=0.1), CAR, 0.25, 1.0, 1.8)   # (0.5, -0.9): seat 1.0, island row
    assert progress == pytest.approx(1.0, abs=0.02)
    assert lateral == pytest.approx(0.1, abs=0.01)


def test_behind_or_too_far_is_not_a_sign():
    assert sign_position(dict(x=-0.2, y=0.1), CAR, 0.25, 1.0, 1.8) is None
    assert sign_position(dict(x=2.5, y=0.0), CAR, 0.25, 1.0, 1.8) is None


def test_cw_mirrors_the_cluster_and_swaps_colours():
    progress, lateral = sign_position(dict(x=0.75, y=-0.1), CAR, 0.25, -1.0, 1.8)
    assert lateral == pytest.approx(0.1, abs=0.01)                 # island row in the mirror
    observation = [dict(x=0.75, y=-0.1, color='red')]
    assert sign_colour(dict(x=0.75, y=-0.1), observation, 1.0, 1.0, -1.0) == 'green'
    assert sign_colour(dict(x=0.75, y=-0.1), observation, 1.0, 1.0, 1.0) == 'red'
    assert SWAP['green'] == 'red'


def test_old_observations_colour_nothing():
    observation = [dict(x=0.75, y=0.1, color='red')]
    assert sign_colour(dict(x=0.75, y=0.1), observation, 0.0, 1.0, 1.0) is None


def test_the_memory_files_signs_by_seat_and_votes_colours():
    memory = SeatMemory()
    cluster = [dict(x=0.75, y=0.1)]
    memory.remember(cluster, [], -1.0, 0.0, CAR, 0.25, 1.0, 1.8)
    assert memory.next_sign(0.25, 1.8) is None                      # one sighting is not enough
    memory.remember(cluster, [dict(x=0.75, y=0.1, color='green')], 0.1, 0.1, CAR, 0.25, 1.0, 1.8)
    assert memory.next_sign(0.25, 1.8) == pytest.approx((0.1, 1.0, 'green'), abs=1e-9) or \
        memory.next_sign(0.25, 1.8)[2] == 'green'
    lateral, progress, colour = memory.next_sign(0.25, 1.8)
    assert (lateral, progress, colour) == (0.1, pytest.approx(1.0), 'green')
    assert list(memory.seats) == [2]                                 # half-unit index of seat 1.0


def test_a_colour_is_remembered_a_lap_later():
    memory = SeatMemory()
    for k in range(3):
        memory.remember([dict(x=0.75, y=-0.1)], [dict(x=0.75, y=-0.1, color='red')], k, k, CAR, 0.25, 1.0, 1.8)
    lateral, progress, colour = memory.next_sign(8.25, 1.8)
    assert (lateral, colour) == (-0.1, 'red') and progress == pytest.approx(9.0)
    assert memory.ahead(8.25, 1.5, 4.0) == {9.0: (-0.1, 'red')}


def test_clusters_off_the_rows_or_between_seats_are_ignored():
    memory = SeatMemory()
    for k in range(3):
        memory.remember([dict(x=0.6, y=-0.3), dict(x=0.5, y=0.1)], [], -1, k, CAR, 0.25, 1.0, 1.8)
    assert SECTOR < 0.3 and memory.seats == {}                     # a lot limit at -0.3; a cluster at progress 0.75
