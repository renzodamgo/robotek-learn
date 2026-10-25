import io
import json

from robotek_evaluation.contact_log import EpisodeLog, compact, message_count, read


def contact(t, robot='robotek::front_right_wheel::c', other='game_field::field::wall_int_south_col',
            depth=1e-4, source='/audit/robot_contacts'):
    return dict(sim_time=t, source=source, pillar=None, other=other, kind='inner_wall',
                robot=robot, max_depth=depth, points=2)


def test_one_episode_per_continuous_contact_and_pair():
    messages = [contact(1 + i*.001) for i in range(500)]                     # 0.5 s pressed
    messages += [contact(1.2 + i*.001, robot='robotek::front_left_wheel::c') for i in range(3)]
    messages += [contact(2.0)]                                               # after a gap
    episodes = compact(sorted(messages, key=lambda c: c['sim_time']))
    assert [(e['robot'].split('::')[1], e['sim_time'], e['messages']) for e in episodes] == [
        ('front_right_wheel', 1.0, 500), ('front_left_wheel', 1.2, 3), ('front_right_wheel', 2.0, 1)]
    assert abs(episodes[0]['end_sim_time'] - 1.499) < 1e-9
    assert message_count(episodes) == len(messages)


def test_max_depth_and_points_are_maxima():
    messages = [contact(1.0, depth=1e-4), contact(1.001, depth=3e-4), contact(1.002, depth=None)]
    (episode,) = compact(messages)
    assert episode['max_depth'] == 3e-4 and episode['points'] == 2


def test_written_log_keeps_first_contact_even_if_recorder_is_killed():
    stream = io.StringIO()
    log = EpisodeLog(stream)
    for i in range(3000):                       # 3 s pressed against the wall
        log.add(contact(5 + i*.001))
    killed = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert killed[0]['sim_time'] == 5 and len(killed) <= 4   # start + one checkpoint per second
    log.close()
    lines = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert lines[-1]['messages'] == 3000


def test_read_accepts_new_and_old_logs(tmp_path):
    messages = [contact(1 + i*.001) for i in range(10)] + [contact(3.0, other='game_field::field::wall_ext_n')]
    old = tmp_path/'old.jsonl'
    old.write_text(''.join(json.dumps(m)+'\n' for m in messages))
    new = tmp_path/'new.jsonl'
    with new.open('w') as stream:
        log = EpisodeLog(stream)
        for m in messages:
            log.add(m)
        log.close()
    for path in (old, new):
        episodes = read(path)
        assert [(e['sim_time'], e['messages']) for e in episodes] == [(1.0, 10), (3.0, 1)]
