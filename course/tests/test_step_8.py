from my_open.direction import DirectionVote


def feed(vote, frames):
    return [vote.update(blue, orange) for blue, orange in frames]


def test_five_clear_frames_in_a_row_decide():
    vote = DirectionVote()
    assert feed(vote, [(300, 60)] * 5) == [0, 0, 0, 0, +1]


def test_orange_means_clockwise():
    assert feed(DirectionVote(), [(50, 280)] * 5)[-1] == -1


def test_similar_lines_do_not_decide():
    assert feed(DirectionVote(), [(300, 280)] * 20)[-1] == 0                # confidence 0.07 < 0.25


def test_short_lines_do_not_decide():
    assert feed(DirectionVote(), [(30, 5)] * 20)[-1] == 0                   # below 40 px


def test_one_bad_frame_restarts_the_count():
    vote = DirectionVote()
    out = feed(vote, [(300, 60)] * 4 + [(20, 5)] + [(300, 60)] * 4)
    assert out[-1] == 0
    assert vote.update(300, 60) == +1


def test_changing_its_mind_needs_five_frames_again():
    vote = DirectionVote()
    feed(vote, [(300, 60)] * 5)
    assert vote.direction == +1
    out = feed(vote, [(60, 300)] * 5)
    assert out[:4] == [0, 0, 0, 0] and out[-1] == -1


def test_no_lines_no_decision():
    assert feed(DirectionVote(), [(0, 0)] * 10)[-1] == 0
